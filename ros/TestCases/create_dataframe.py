import matplotlib.pyplot as plt
'''
# NoGazeboTokens Test
x1 = [1,2,3,4,5,6,7,8,9,10,11,12,13,14,15]
y1 = [16.4,18.1,21.4,23.2,24,26.2,28,28.2,29.7,32.3,33.4,40.2,41.5,42.4,47.7]
# plotting the line 1 points
plt.plot(x1, y1, label = "CPU %")
# line 2 points
x2 = [1,2,3,4,5,6,7,8,9,10,11,12,13,14,15]
y2 = [30.1,32.1,34,36.1,38,38.2,39.9,41.9,43.9,45.7,47.7,49.8,51.3,51.8,53.6]
# plotting the line 2 points
plt.plot(x2, y2, label = "Virtual memory %")
plt.xlabel('tokens')
# Set the y axis label of the current axis.
plt.ylabel('percentage (%)')
# Set a title of the current axes.
plt.title('% of CPU and virtual memory for an increasing number of tokens')
# show a legend on the plot
plt.legend()
# Display a figure.
plt.show()
'''
'''
# NoGazeboPlaces Test
x1 = [2,3,4,5,6,7,8,9,10,11,12,13,14,15,16]
y1 = [21.5,22.1,23.1,23.4,23.5,24.6,24.8,25,25.3,26.3,26.7,27,28.3,30.2,32.1]
# plotting the line 1 points
plt.plot(x1, y1, label = "CPU %")
# line 2 points
x2 = [2,3,4,5,6,7,8,9,10,11,12,13,14,15,16]
y2 = [31.9,32.7,33.8,34.7,35.1,35.3,36.5,37.2,38.3,38.4,39,39,40,40.5,42]
# plotting the line 2 points
plt.plot(x2, y2, label = "Virtual memory %")
plt.xlabel('places')
# Set the y axis label of the current axis.
plt.ylabel('percentage (%)')
# Set a title of the current axes.
plt.title('% of CPU and virtual memory for an increasing number of places')
# show a legend on the plot
plt.legend()
# Display a figure.
plt.show()
'''
'''
# Combination of both Test
x1 = [7,14,21,28,35,42,49,56,63,70,77,84,91,98,105]
y1 = [16.4,18.1,21.4,23.2,24,26.2,28,28.2,29.7,32.3,33.4,40.2,41.5,42.4,47.7]
# plotting the line 1 points
plt.plot(x1, y1, label = "Increasing tokens")
# line 2 points
x2 = [10,15,20,25,30,35,40,45,50,55,60,65,70,75,80]
y2 = [21.5,22.1,23.1,23.4,23.5,24.6,24.8,25,25.3,26.3,26.7,27,28.3,30.2,32.1]
# plotting the line 2 points
plt.plot(x2, y2, label = "Increasing places")
plt.xlabel('action servers')
# Set the y axis label of the current axis.
plt.ylabel('percentage (%)')
# Set a title of the current axes.
plt.title('% of CPU for an increasing number of action servers')
# show a legend on the plot
plt.legend()
# Display a figure.
plt.show()
'''


# NoGazeboPlacesOneToken Test
x1 = [2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,40,45,50,55,60,65,70,75,80,85,90,95,100]
y1 = [14.6,15,15.4,15.7,16.3,16.6,17.5,17.6,17.6,17.8,17.8,17.8,18,18.6,19.2,19.2,19.3,19.5,19.6,19.7,19.8,19.8,19.9,20.2,20.6,20.7,20.7,20.7,
      20.7,20.7,21.3,21.5,21.8,21.8,21.8,22.2,23.2,24.3,25.1,25.6,26.9,27.5,28.5,29.1,29.6,31.3,33]
# plotting the line 1 points
plt.plot(x1, y1, label = "CPU %")
# line 2 points
x2 = [2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,40,45,50,55,60,65,70,75,80,85,90,95,100]
y2 = [24.6,25.3,25.4,25.9,26.3,26.8,27.2,27.4,27.4,27.5,27.5,27.7,28.1,28.2,28.4,28.4,28.5,28.7,28.7,28.7,28.8,28.8,28.9,29,29.1,29.2,29.6,29.6,
      30.1,30.8,31.2,31.6,31.7,32.2,32.3,32.5,33,35.9,35.9,35.9,36.8,37.5,39,39.8,40.3,41.7,42]
# plotting the line 2 points
plt.plot(x2, y2, label = "Virtual memory %")
plt.xlabel('places')
# Set the y axis label of the current axis.
plt.ylabel('percentage (%)')
# Set a title of the current axes.
plt.title('% of CPU and virtual memory for an increasing number of places with one token')
# show a legend on the plot
plt.legend()
# Display a figure.
plt.show()
