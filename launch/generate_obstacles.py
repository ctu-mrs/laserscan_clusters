import random
import math

# Parameters
num_obstacles = 200
x_min = -10.0
x_max = 10.0
y_min = -32.0
y_max = 32.0
min_distance = 1.5

# Function to check if the new position is at least `min_distance` away from existing ones
def is_valid_position(new_x, new_y, obstacles):
    for (x, y) in obstacles:
        if math.sqrt((x - new_x)**2 + (y - new_y)**2) < min_distance:
            return False
    return True

# Generate obstacle positions
obstacles_x = []
obstacles_y = []
obstacles = []

while len(obstacles_x) < num_obstacles:
    new_x = random.uniform(x_min, x_max)
    new_y = random.uniform(y_min, y_max)

    if is_valid_position(new_x, new_y, obstacles):
        obstacles_x.append(new_x)
        obstacles_y.append(new_y)
        obstacles.append((new_x, new_y))

# Print obstacles for ROS param
print('<rosparam param="obstacles_x">', obstacles_x, '</rosparam>')
print('<rosparam param="obstacles_y">', obstacles_y, '</rosparam>')
