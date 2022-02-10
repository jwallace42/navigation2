from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rcl_interfaces.msg import ParameterValue, ParameterType, Parameter
from rcl_interfaces.srv import SetParameters
import time
import math
import os 

#Data visualization 
import numpy as np
import seaborn as sns
import matplotlib.pylab as plt

# generate random integer values
from random import seed
from random import randint

def getPlannerResults(navigator, initial_pose, goal_pose, planners):
    results = []
    for planner in planners:
        path = navigator.getPath(initial_pose, goal_pose, planner)

        if path is not None:
            results.append(path)
    return results

def getPaths(results):
    paths = [] 
    for result in results:
        paths.append(result.path)
    return paths

def getPathLength(path):
    path_length = 0
    x_prev = path.poses[0].pose.position.x
    y_prev = path.poses[0].pose.position.y
    for i in range(1, len(path.poses)):
        x_curr = path.poses[i].pose.position.x
        y_curr = path.poses[i].pose.position.y
        path_length = path_length + math.sqrt((x_curr-x_prev)**2 + (y_curr-y_prev)**2)
        x_prev = x_curr
        y_prev = y_curr
    return path_length

def getMapCoordsFromPaths(paths, resolution, map_origin):
    coords = []
    for path in paths:
        x = []
        y = []
        for pose in path.poses:
            x.append(pose.pose.position.x/resolution + map_origin[0]/resolution)
            y.append(pose.pose.position.y/resolution + map_origin[1]/resolution)
        coords.append(x)
        coords.append(y)
    return coords

def getRandomGoals(num_of_goals, min_dist, data, res, map_origin):
    rand_goals = []
    while len(rand_goals) < num_of_goals:
        row = randint(10, data.shape[0]-10)
        col = randint(10, data.shape[1]-10)

        x_diff = col*res - map_origin[0]
        y_diff = row*res - map_origin[1]
        dist = math.sqrt( x_diff ** 2 + y_diff ** 2)
        if data[row,col] < 210 and dist > min_dist:
            rand_goals.append([col, row])
    return rand_goals

# Important notes: need to remove the localization from the robot navigator
def main():
    rclpy.init()
    print("hello")

    navigator = BasicNavigator()
    map_origin = [5.0, 10.0] # Not sure how to get this out of the yaml 

    #Set initial pose
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    # Why does this not set the initial pose correctly
    initial_pose.pose.position.x = 0.0
    initial_pose.pose.position.y = 0.0
    initial_pose.pose.orientation.z = 1.0
    initial_pose.pose.orientation.w = 0.0
    navigator.setInitialPose(initial_pose)

    # Wait for navigation to fully activate, since autostarting nav2
    navigator.waitUntilNav2Active()

    # If desired, you can change or load the map as well
    # navigator.changeMap('/path/to/map.yaml')

    # Get the costmap for validating goals
    costmap = navigator.getGlobalCostmap()
    data = np.asarray(costmap.data)
    data.resize(costmap.metadata.size_y, costmap.metadata.size_x)

    # Get random goals
    seed(2)
    goals = 100 
    min_dist = 3.0
    rand_goals = getRandomGoals(goals, min_dist, data, costmap.metadata.resolution, map_origin)

    
    #Get the paths for the following planners 
    planners = ['NavFn', 'SmacLattice', 'Smac2D', 'SmacHybrid']
    results = [] 
    paths = []

    count = 0
    for rand_goal in rand_goals:
        count = count + 1
        print(count, rand_goal)
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = rand_goal[0]*costmap.metadata.resolution - map_origin[0]
        goal_pose.pose.position.y = rand_goal[1]*costmap.metadata.resolution - map_origin[1]
        goal_pose.pose.orientation.w = 1.0
        results = getPlannerResults(navigator, initial_pose, goal_pose, planners)
        paths = getPaths(results)

    #Post proccessing, data needs to be stored for later
    coords = getMapCoordsFromPaths(paths, costmap.metadata.resolution, map_origin)
        

    #Path lengths
    # path_lengths = []
    # for path in paths:
    #     path_lengths.append(getPathLength(path))

    # Visualize the costmap better
    # data = np.where(data == 255, 512, data)
    # data = np.where(data == 254, 385, data)

    #Plot results
    data = np.where(data < 254, 0, data)
    data = data * -1
    plt.figure(3)
    ax = sns.heatmap(data, cbar=False)
    for i in range(0, len(coords), 2):
        ax.plot(coords[i], coords[i+1], linewidth=0.7)
    ax.legend(planners)
    plt.axis('off')
    plt.show()

    # Clean up 
    navigator.lifecycleShutdown()
    exit(0)

if __name__ == '__main__':
    main()