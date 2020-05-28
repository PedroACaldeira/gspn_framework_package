import os

from flask import Flask, render_template, jsonify, request, flash
import gspn as pn
import gspn_tools
from werkzeug.utils import secure_filename, redirect

app = Flask(__name__)  # create an app instance
my_pn = pn.GSPN()

ALLOWED_EXTENSIONS = set(['xml'])


def allowed_file(filename):
    return '.' in filename and filename.rsplit('.', 1)[1].lower() in ALLOWED_EXTENSIONS


@app.route("/")
def home():
    return render_template("gspn_visualization_open_gspn.html")


@app.route("/use_code")
def use_code():
    # Write your code here
    my_pn.add_places(['p1', 'p2', 'p3', 'p4', 'p5', 'p6', 'p7', 'p8', 'p9', 'p10', 'p11', 'p12'],
                     [0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1])
    my_pn.add_transitions(['t1', 't2', 't3', 't4', 't5', 't6', 't7', 't8', 't9', 't10'],
                          ['exp', 'exp', 'exp', 'exp', 'exp', 'imm', 'imm', 'exp', 'exp', 'exp'],
                          [1, 1, 1, 1, 1, 1, 1, 1, 1, 1])

    arc_in = {'p1': ['t1'], 'p2': ['t2'], 'p3': ['t3'], 'p5': ['t4'], 'p6': ['t4'], 'p7': ['t5'],
              'p8': ['t6', 't7'], 'p9': ['t8', 't9'], 'p10': ['t10'], 'p11': ['t4'], 'p12': ['t5']}

    arc_out = {'t1': ['p2'], 't2': ['p3'], 't3': ['p4', 'p5', 'p6'], 't4': ['p7'], 't5': ['p8', 'p9'], 't6': ['p1'],
               't7': ['p9'], 't8': ['p2'], 't9': ['p10'], 't10': ['p1']}

    my_pn.add_arcs(arc_in, arc_out)
    return render_template("gspn_visualization_home.html", data=my_pn)


@app.route("/use_xml", methods=['POST'])
def use_xml():
    filename = ""
    # Get file from user's explorer
    # I saved the source for this code in the folder TESE
    if request.method == 'POST':

        file = request.files['file']
        if file.filename == '':
            flash('No file selected for uploading')
            return redirect(request.url)
        if file and allowed_file(file.filename):
            filename = secure_filename(file.filename)
        else:
            flash('Allowed file types are txt, pdf, png, jpg, jpeg, gif')
            return redirect(request.url)

    tool = gspn_tools.GSPNtools()
    gspn = tool.import_xml(filename)[0]

    # Begin First Part: add_places
    marking = gspn.get_current_marking()
    places = []
    tokens = []
    for place in marking:
        places.append(place)
        tokens.append(marking[place])
    my_pn.add_places(places, tokens)

    # Begin Second Part: add_transitions
    transitions = gspn.get_transitions()
    transition_names = []
    transition_types = []
    transition_rates = []
    for transition in transitions:
        transition_names.append(transition)
        transition_types.append(transitions[transition][0])
        # HEADS UP! THE RATE IS ALWAYS 1
        transition_rates.append(1)
    my_pn.add_transitions(transition_names, transition_types, transition_rates)

    # Begin Third Part: arc_in
    arc_in = {}
    arc_in_aux = gspn.get_arcs_dict()[0]
    for place in arc_in_aux:
        place_name = gspn.index_to_places[place]
        for transition in arc_in_aux[place]:
            if place_name in arc_in:
                arc_in[place_name].append(gspn.index_to_transitions[transition])
            else:
                arc_in[place_name] = [gspn.index_to_transitions[transition]]

    # Begin Fourth Part: arc_out
    arc_out = {}
    arc_out_aux = gspn.get_arcs_dict()[1]
    for transition in arc_out_aux:
        transition_name = gspn.index_to_transitions[transition]
        for place in arc_out_aux[transition]:
            if transition_name in arc_out:
                arc_out[transition_name].append(gspn.index_to_places[place])
            else:
                arc_out[transition_name] = [gspn.index_to_places[place]]
    my_pn.add_arcs(arc_in, arc_out)

    return render_template("gspn_visualization_home.html", data=my_pn)


@app.route('/background_process_test')
def background_process_test():
    marking_and_transition_list = my_pn.simulate()
    print("marking and trans ", marking_and_transition_list)
    return jsonify(my_pn.get_current_marking(), marking_and_transition_list[-1][0], my_pn.get_enabled_transitions())


@app.route('/background_simulate_n_steps', methods=['GET', 'POST'])
def background_simulate_n_steps():
    if request.method == 'POST':
        text = request.form['n_steps_text']
        if text == '':
            return jsonify("NONE")
        else:
            processed_text = int(text)
            if processed_text == 0:
                return jsonify("ZERO")
            else:
                marking_and_transition_list = my_pn.simulate(nsteps=processed_text)
                print("marking and trans ", marking_and_transition_list)
                return jsonify(marking_and_transition_list, my_pn.get_enabled_transitions())


@app.route('/background_fire_chosen_transition', methods=['GET', 'POST'])
def background_fire_chosen_transition():
    if request.method == 'POST':
        text = request.form['chosenTransitionDropdown']
        processed_text = str(text)
        my_pn.fire_transition(processed_text)
        return jsonify(my_pn.get_current_marking(), processed_text, my_pn.get_enabled_transitions())


@app.route('/background_reset_simulation')
def background_reset_simulation():
    my_pn.reset_simulation()
    return jsonify(my_pn.get_current_marking(), my_pn.get_enabled_transitions())


@app.route('/background_check_liveness', methods=['GET', 'POST'])
def background_check_liveness():
    my_pn.init_analysis()
    if request.method == 'POST':
        liveness_check = my_pn.liveness()
        return jsonify(liveness_check)


@app.route('/background_check_throughputrate', methods=['GET', 'POST'])
def background_check_throughputrate():
    my_pn.init_analysis()
    if request.method == 'POST':
        text = request.form['throughput_rate_dropdown']
        processed_text = str(text)
        print("processed text", processed_text)
        throughput = my_pn.transition_throughput_rate(processed_text)
        rounded_value = round(throughput, 1)
        print("rounded_value ", rounded_value)
        return jsonify(processed_text)


@app.route('/background_check_probntokens', methods=['GET', 'POST'])
def background_check_probntokens():
    my_pn.init_analysis()
    text = request.form.get('prob_n_tokens_text', None)
    processed_text = str(text)
    return jsonify("to-do")


@app.route('/background_expected_n_tokens', methods=['GET', 'POST'])
def background_expected_n_tokens():
    my_pn.init_analysis()
    if request.method == 'POST':
        text = request.form.get('expected_n_tokens_dropdown', None)
        processed_text = str(text)
        return jsonify(processed_text)


@app.route('/background_check_transitionprobevo', methods=['GET', 'POST'])
def background_check_transitionprobevo():
    my_pn.init_analysis()
    text = request.form.get('transition_prob_evo_text', None)
    processed_text = str(text)
    return jsonify("to-do")


@app.route('/background_check_mean_wait_time', methods=['GET', 'POST'])
def background_check_mean_wait_time():
    if request.method == 'POST':
        my_pn.init_analysis()
        text = request.form.get('mean_wait_time_dropdown', None)
        processed_text = str(text)
        return jsonify(processed_text)


@app.route("/about")
def about():
    return render_template("gspn_visualization_about.html")


if __name__ == "__main__":
    app.run(debug=True)
