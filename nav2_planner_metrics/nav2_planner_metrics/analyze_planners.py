from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rcl_interfaces.msg import ParameterValue, ParameterType, Parameter
from rcl_interfaces.srv import SetParameters
import time
import math

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
        results.append(navigator.getPath(initial_pose, goal_pose, planner))
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

def getMapCoordsFromPaths(paths, resolution):
    coords = []
    for path in paths:
        x = []
        y = []
        for pose in path.poses:
            x.append(pose.pose.position.x/resolution + 5.0/resolution)
            y.append(pose.pose.position.y/resolution + 10.0/resolution)
        coords.append(x)
        coords.append(y)
    return coords

def getRandomGoals(num_of_goals, data):
    seed(1)
    rand_goals = []
    while len(rand_goals) < num_of_goals:
        row = randint(0, data.shape[0])
        col = randint(0, data.shape[1])
        if data[row,col] < 240:
            rand_goals.append([row, col])
    return rand_goals

# Important notes: need to remove the localization from the robot navigator
def main():
    rclpy.init()
    print("hello")

    navigator = BasicNavigator()

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

    # Get the costmap for plotting
    costmap = navigator.getGlobalCostmap()
    data = np.asarray(costmap.data)
    data.resize(costmap.metadata.size_y, costmap.metadata.size_x)
    print(data.shape)

    # Set our goal
    rand_goals = getRandomGoals(10, data)

    
    #Get the paths for the following planners 
    planners = ['NavFn', 'SmacLattice', 'Smac2D', 'SmacHybrid']
    results = [] 
    paths = []

    for rand_goal in rand_goals:
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = rand_goal[1]*costmap.metadata.resolution
        goal_pose.pose.position.y = rand_goal[0]*costmap.metadata.resolution
        goal_pose.pose.orientation.w = 1.0
        results = getPlannerResults(navigator, initial_pose, goal_pose, planners)
        paths = getPaths(results)

    coords = getMapCoordsFromPaths(paths, costmap.metadata.resolution)
        

    #Path lengths
    # path_lengths = []
    # for path in paths:
    #     path_lengths.append(getPathLength(path))

    # plot the costmap
    # time_ms = []
    # for time in times:
    #     time_ms.append(time.nanosec*0.000001)

    #Bar chart for data 
    # plt.figure(1)
    # planners = ['NavFn', 'StateLattice', 'SmacPlanner2D', 'SmacPlannerHybrid']
    # plt.bar(planners, time_ms)
    # plt.xlabel('Planner type')
    # plt.ylabel('Planner time (ms)')

    # plt.figure(2)
    # planners = ['NavFn', 'StateLattice', 'SmacPlanner2D', 'SmacPlannerHybrid']
    # plt.bar(planners, path_length)
    # plt.xlabel('Planner type')
    # plt.ylabel('Path length (m)')
    
    # Plot paths 
    #plot the cost 
    data = np.where(data < 254, 0, data)
    # data = np.where(data == 255, 512, data)
    # data = np.where(data == 254, 385, data)
    data = data * -1
    plt.figure(3)
    ax = sns.heatmap(data, cbar=False)
    color = (128/255, 128/255, 128/255)
    ax.plot(coords[0],coords[1], 'r', linewidth=0.7)
    ax.plot(coords[2],coords[3], 'b', linewidth=0.7)
    ax.plot(coords[4],coords[5], 'g', linewidth=0.7)
    ax.plot(coords[6],coords[7], linewidth=0.7)
    ax.legend(['NavFn', 'StateLattice', 'SmacPlanner2D', 'SmacPlannerHybrid'])
    plt.axis('off')
    plt.show()


    navigator.lifecycleShutdown()
    exit(0)

if __name__ == '__main__':
    main()