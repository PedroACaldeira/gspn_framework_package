#! /usr/bin/env python3
# Standard libs
from concurrent.futures.thread import ThreadPoolExecutor
import os
import sys
import numpy as np
from datetime import datetime
import time
import random
import json
import ast
# ROS libs
import rospy
import actionlib
import gspn_framework_package.msg
from gspn_framework_package.msg import ExecGSPNAction
from gspn_framework_package.msg import GSPNFiringData
# Files from my package
from gspn_framework_package import policy
from gspn_framework_package import gspn as pn
from gspn_framework_package import gspn_tools


class GSPNExecutionROS(object):

    def __init__(self, gspn, place_to_client_mapping, policy, project_path, initial_place, robot_id, full_synchronization):
        '''
        :param gspn: a previously created gspn
        :param place_to_client_mapping: dictionary where key is the place and the value is the function
        :param policy: Policy object
        :param project_path: string with project path
        :param initial_place: string with the name of the robot's initial place

        self.__token_positions is a list with the places where each token is ['p1', 'p2', 'p2'] means that token 1 is on p1, token 2
        is on p2 and token 3 is on p2;
        self.__number_of_tokens is the current number of tokens;
        self.__action_clients is a list with all the action clients;
        self.__client_node is the node where the clients will be connected to.



        self.__current_place indicates where the robot is
        self.__action_client is the action client of the robot
        self.__robot_id is the id of the robot
        self.__full_synchronization is a boolean which represents the type of synch that the user wants
        '''
        self.__gspn = gspn
        self.__token_positions = []

        self.__place_to_client_mapping = place_to_client_mapping

        self.__policy = policy
        self.__project_path = project_path

        self.__number_of_tokens = 0
        # should be removed after correcting many to many/many to 1/1 to many
        self.__action_clients = []
        # --
        self.__client_node = 0
        self.__client_node_subscriber = 0

        self.__current_place = initial_place
        self.__action_client = 0
        self.__robot_id = robot_id
        self.__full_synchronization = full_synchronization

        self.__publisher = 0
        self.__subscriber = 0

    def get_path(self):
        return self.__project_path

    def get_policy(self):
        return self.__policy

    def convert_to_tuple(self, marking, order):
        '''
        :param marking: dictionary with key= places; value= number of tokens in place
        :param order: tuple of strings with the order of the marking on the policy
        :return: tuple with the marking
        '''
        marking_list = []
        for element in order:
            for key in marking:
                if element == key:
                    marking_list.append(marking[key])
        return tuple(marking_list)

    def get_transitions(self, marking, policy_dictionary):
        '''
        :param marking: tuple with the current marking (should already be ordered)
        :param policy_dictionary: dictionary where key=Transition name; value=probability of transition
        :return: transition dictionary if marking is in policy_dictionary; False otherwise
        '''
        for mark in policy_dictionary:
            if marking == mark:
                return policy_dictionary[mark]
        return False

    def translate_arcs_to_marking(self, arcs):
        '''
        : param arcs: set of connected add_arcs
        : return: dictionary where key is the place and the value is always 1
        This function essentially is used to determine which places are connected to the next transition.
        The purpose of this is to later on compare it with the current marking.
        '''
        translation = {}
        for i in arcs[0]:
            place = self.__gspn.index_to_places[i]
            translation[place] = 1
        return translation


    '''
    Callback functions section:
    In this section, we included every callback function that is being used both by our Action Clients and
    our publishers/subscribers:
    - topic_listener_callback;
    - topic_talker_callback;
    - action_get_result_callback;
    - action_goal_response_callback;
    - action_feedback_callback;
    - action_send_goal
    '''

    def topic_listener_callback(self, msg):
        if msg.robot_id != self.__robot_id:
            rospy.loginfo('I heard Robot %s firing %s' % (msg.robot_id, msg.transition))
            print("BEFORE", self.__gspn.get_current_marking())
            self.fire_execution(msg.transition)
            print("AFTER", self.__gspn.get_current_marking())
        else:
            rospy.loginfo('I heard myself firing %s' % msg.transition)


    def topic_talker_callback(self, fired_transition):
        # the topic message is composed by four elements:
        # - fired transition;
        # - current marking;
        # - robot_id;
        # - timestamp.
        msg = GSPNFiringData()
        current_time = rospy.get_rostime()
        msg.transition = str(fired_transition)
        msg.marking = str(self.__gspn.get_current_marking())
        msg.robot_id = self.__robot_id
        print("current time ", current_time)
        msg.timestamp = str(current_time)

        self.__publisher.publish(msg)
        rospy.loginfo('Robot %s firing %s'% (msg.robot_id, msg.transition))


    def service_return_current_place_callback(self, request, response):
        response.current_place = self.__current_place
        response.robot_id = self.__robot_id
        rospy.loginfo('Returning %s from robot %s' % (self.__current_place, self.__robot_id))
        return response


    def service_send_request(self):
        self.__client_node.future = self.__client_node.cli.call_async(self.__client_node.req)


    def action_get_result_callback(self, status, result):

        # status == 3 means SUCCESS
        if status == 3:
            print(': Goal succeeded! Result: {0}'.format(result.transition))

            bool_output_arcs = self.check_output_arcs(self.__current_place)

            if bool_output_arcs:
                print("The place has output arcs.")
                print("BEFORE", self.__gspn.get_current_marking())
                if result.transition == 'None':
                    print("Immediate transition")

                    if self.__full_synchronization == False:
                        self.service_send_request()
                        robot_places = []
                        while True:
                            rclpy.spin_once(self.__client_node)
                            if self.__client_node.future.done():
                                robot_places.append(self.__client_node.future.result().current_place)
                        print("robot places ", robot_places)

                    else:
                        imm_transition_to_fire = self.get_policy_transition()
                        if imm_transition_to_fire == False:
                            reason = "The policy does not include this case: " + self.__gspn.get_current_marking()
                            rospy.signal_shutdown(reason)
                            return
                        else:
                            self.fire_execution(imm_transition_to_fire)
                            self.topic_talker_callback(imm_transition_to_fire)
                else:
                    print("exponential transition")
                    print(result.transition)
                    self.fire_execution(result.transition)
                    self.topic_talker_callback(result.transition)

                print("AFTER", self.__gspn.get_current_marking())

                action_type = self.__place_to_client_mapping[self.__current_place][0]
                server_name = self.__place_to_client_mapping[self.__current_place][1]
                self.__action_client = actionlib.SimpleActionClient(server_name, action_type)
                self.action_send_goal(self.__current_place, action_type, server_name)

            else:
                rospy.signal_shutdown("The place has no output arcs.")

        else:
            print(self.__action_client._action_name + ': Goal failed with status: {0}'.format(status))


    def action_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            print('Goal rejected :( '+ self.__action_client._action_name)
            return

        print('Goal accepted :) '+ self.__action_client._action_name)

        self.__action_client._get_result_future = goal_handle.get_result_async()
        self.__action_client._get_result_future.add_done_callback(self.action_get_result_callback)


    def action_feedback_callback(self, feedback):
        print('Received feedback: {0}'.format(feedback.time_passed))


    def action_send_goal(self, current_place, action_type, server_name):
        self.__action_client.wait_for_server()
        print('Waiting for action server '+ server_name)
        goal = gspn_framework_package.msg.ExecGSPNGoal(current_place)
        print('Sending goal request to '+ server_name)
        self.__action_client.send_goal(goal, done_cb=self.action_get_result_callback, feedback_cb=self.action_feedback_callback)


    '''
    GSPN Execution core functions section:
    In this section, we included the four main functions that are responsible for the execution of our
    GSPNs:
    - fire execution;
    - get_immediate_transition_result;
    - check_output_arcs;
    - ros_gspn_execution.
    '''

    def fire_execution(self, transition):
        '''
        Fires the selected transition.
        :param transition: string with transition that should be fired
        '''
        arcs = self.__gspn.get_connected_arcs(transition, 'transition')
        index = self.__gspn.transitions_to_index[transition]
        marking = self.__gspn.get_current_marking()

        # 1 to 1
        if len(arcs[0]) == 1 and len(arcs[1][index]) == 1:
            new_place = self.__gspn.index_to_places[arcs[1][index][0]]
            self.__gspn.fire_transition(transition)
            self.__current_place = new_place

        # 1 to many
        elif len(arcs[0]) == 1 and len(arcs[1][index]) > 1:
            i = 0
            for i in range(len(arcs[1][index])):
                if i == 0:
                    new_place = self.__gspn.index_to_places[arcs[1][index][i]]
                    # self.__token_positions[token_id] = new_place
                else:
                    new_place = self.__gspn.index_to_places[arcs[1][index][i]]
                    self.__token_positions.append(new_place)
                    self.__number_of_tokens = self.__number_of_tokens + 1
                    self.__action_clients.append(client.MinimalActionClient("provisional", node=self.__client_node, server_name="provisional"))
            self.__gspn.fire_transition(transition)

        # many to 1
        elif len(arcs[0]) > 1 and len(arcs[1][index]) == 1:
            translation_marking = self.translate_arcs_to_marking(arcs)
            check_flag = True

            # We go through the marking and check it
            for el in translation_marking:
                if marking[el] < translation_marking[el]:
                    check_flag = False

            # We go through the states and see if all of them are 'Waiting'
            number_of_waiting = 0
            for place in translation_marking:
                for pos_index in range(len(self.__token_positions)):
                    if self.__token_positions[pos_index] == place:
                        if self.__action_clients[pos_index].get_state() == 'Waiting':
                            number_of_waiting = number_of_waiting + 1
                            break
            # -1 porque só precisas de ter x-1 a espera. quando o numero x
            # aparece, podes disparar
            if number_of_waiting == len(translation_marking) - 1:
                check_flag = True
            else:
                check_flag = False

            if check_flag:
                new_place = self.__gspn.index_to_places[arcs[1][index][0]]
                # old_place = self.__token_positions[token_id]
                # self.__token_positions[token_id] = new_place
                self.__gspn.fire_transition(transition)
                for place_index in arcs[0]:
                    place_with_token_to_delete = self.__gspn.index_to_places[place_index]
                    if place_with_token_to_delete != old_place:
                        for j in range(len(self.__token_positions)):
                            if place_with_token_to_delete == self.__token_positions[j]:
                                index_to_del = j
                                self.__token_positions[index_to_del] = "null"
                                self.__action_clients[index_to_del].set_state("VOID")
                                break
            else:
                print("many to one")
                # self.__action_clients[token_id].set_state("Waiting")

        # many to many
        elif len(arcs[0]) > 1 and len(arcs[1][index]) > 1:
            translation_marking = self.translate_arcs_to_marking(arcs)
            check_flag = True
            for el in translation_marking:
                if marking[el] < translation_marking[el]:
                    check_flag = False
            if check_flag:
                # Create tokens on next places
                i = 0
                for i in range(len(arcs[1][index])):
                    if i == 0:
                        new_place = self.__gspn.index_to_places[arcs[1][index][i]]
                        # self.__token_positions[token_id] = new_place
                    else:
                        new_place = self.__gspn.index_to_places[arcs[1][index][i]]
                        self.__token_positions.append(new_place)
                        self.__number_of_tokens = self.__number_of_tokens + 1
                        self.__action_clients.append(client.MinimalActionClient("provisional", node=self.__client_node, server_name="provisional"))
                        self.__gspn.fire_transition(transition)

                # Delete tokens from previous places
                for place_index in arcs[0]:
                    place_with_token_to_delete = self.__gspn.index_to_places[place_index]
                    for j in range(len(self.__token_positions)):
                        if place_with_token_to_delete == self.__token_positions[j]:
                            index_to_del = j
                            self.__token_positions[index_to_del] = "null"
                            self.__action_clients[index_to_del].set_state("VOID")
                            break
            else:
                print("many to many")
                # self.__action_clients[token_id].set_state("Waiting")


    def get_policy_transition(self):
        execution_policy = self.get_policy()
        current_marking = self.__gspn.get_current_marking()
        order = execution_policy.get_places_tuple()
        marking_tuple = self.convert_to_tuple(current_marking, order)
        pol_dict = execution_policy.get_policy_dictionary()
        transition_dictionary = self.get_transitions(marking_tuple, pol_dict)
        if transition_dictionary == False:
            return False
        transition_list = []
        probability_list = []
        for transition in transition_dictionary:
            transition_list.append(transition)
            probability_list.append(transition_dictionary[transition])
        transition_to_fire = np.random.choice(transition_list, 1, False, probability_list)[0]
        print("TRANSITION TO FIRE", transition_to_fire)
        return transition_to_fire

    def check_output_arcs(self, place):
        arcs = self.__gspn.get_connected_arcs(place, 'place')
        arcs_out = arcs[1]
        if len(arcs_out) >= 1:
            return True
        else:
            return False

    def ros_gspn_execution(self):
        '''
        Setup of the execution:
        1- project path;
        2- number of initial tokens;
        3- token_positions list;
        4- action servers;
        5- initial action clients.
        '''
        # Initialize ROS node
        node_name = "executor_" + str(self.__robot_id)
        rospy.init_node(node_name)

        # Setup project path
        path_name = self.get_path()
        self.__project_path = os.path.join(path_name)
        sys.path.append(self.__project_path)

        # Setup number of (initial) tokens
        self.__number_of_tokens = self.__gspn.get_number_of_tokens()

        # Setup publisher and subscriber
        self.__publisher = rospy.Publisher('/TRANSITIONS_FIRED', GSPNFiringData, queue_size=10)
        self.__subscriber = rospy.Subscriber('/TRANSITIONS_FIRED', GSPNFiringData, self.topic_listener_callback, queue_size=10)

        #Setup action client
        action_type = self.__place_to_client_mapping[self.__current_place][0]
        server_name = self.__place_to_client_mapping[self.__current_place][1]
        self.__action_client = actionlib.SimpleActionClient(server_name, action_type)

        if self.__full_synchronization == False:
            # Setup client node with service and service client
            self.__client_node.cli = self.__client_node.create_client(CurrentPlace, 'current_place')
            self.__client_node.srv = self.__client_node.create_service(CurrentPlace, 'current_place', self.service_return_current_place_callback)
            self.__client_node.req = CurrentPlace.Request()

        self.action_send_goal(self.__current_place, action_type, server_name)
        rospy.spin()


def main():

    #json_file = input("Insert JSON file with GSPN Execution general elements: ")
    #file_to_open = str(json_file)
    project_path = "/home/pedroac/ros2_ws/src/gspn_framework/gspn_framework"

    sys.path.append(os.path.join(project_path))

    with open('/home/pedro/catkin_ws/src/gspn_framework_package/ros/src/gspn_framework_package_ros/gspn_execution_input.json') as f:
        data = json.load(f)

    tool = gspn_tools.GSPNtools()
    to_open = '/home/pedro/catkin_ws/src/gspn_framework_package/ros/src/gspn_framework_package_ros/' + data["gspn"]
    my_pn = tool.import_xml(to_open)[0]

    p_to_c_mapping = ast.literal_eval(data["place_to_client_mapping"])
    # On the JSON I have to include Simple inside a string in order to work well.
    # This is due to the fact that JSON only accepts strings.
    # And so, now I need to parse it and change its value.
    for place in p_to_c_mapping:
        if p_to_c_mapping[place][0] == 'ExecGSPN':
            p_to_c_mapping[place][0] = ExecGSPNAction

    places_tuple = ast.literal_eval(data["places_tuple"])
    policy_dictionary = ast.literal_eval(data["policy_dictionary"])
    full_synchronization = ast.literal_eval(data["full_synchronization"])

    user_robot_id = input("Please insert this robot's id: ")
    user_current_place = input("Please insert the robot's current place: ")

    my_execution = GSPNExecutionROS(my_pn, p_to_c_mapping, policy, project_path, str(user_current_place), int(user_robot_id), full_synchronization)
    my_execution.ros_gspn_execution()


if __name__ == "__main__":
    main()
