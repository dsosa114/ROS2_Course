import matplotlib.pyplot as plt
import numpy as np
import os
# Read the data from the file

# Get the home directory
home_directory = os.path.expanduser('~')
# Join the home directory with the file name
file_path = os.path.join(home_directory, 'recorded_path.txt')
try:
    
    data = np.loadtxt(file_path, delimiter=',')
    x = data[:, 0]
    y = data[:, 1]

    # Plot the path
    plt.figure()
    plt.plot(x, y)
    plt.title('Recorded Path from Odometry')
    plt.xlabel('X (m)')
    plt.ylabel('Y (m)')
    plt.grid(True)
    plt.axis('equal')  # Ensure equal scaling
    plt.show()

except IOError:
    print("Error: 'recorded_path.txt' not found. Please run the ROS 2 node first.")
