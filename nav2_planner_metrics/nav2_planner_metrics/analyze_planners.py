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
import pickle

#Data visualization 
import numpy as np
import seaborn as sns
import matplotlib.pylab as plt

# generate random integer values
from random import seed
from random import randint
from random import uniform

def getPlannerResults(navigator, initial_pose, goal_pose, planners):
    results = []
    for planner in planners:
        print("Initial Pose:", initial_pose.pose.position.y)
        path = navigator.getPath(initial_pose, goal_pose, planner)
        if path is not None:
            print('GetPlannerResults', path.path.poses[0].pose.position.y)
            results.append(path)
    return results

def getRandomGoals(num_of_goals, min_dist, data, res, map_origin):
    rand_goals = []
    while len(rand_goals) < num_of_goals:
        row = randint(10, data.shape[0]-10)
        col = randint(10, data.shape[1]-10)
        theta = uniform(0,1) * 2*math.pi

        x_diff = col*res - map_origin[0]
        y_diff = row*res - map_origin[1]
        dist = math.sqrt( x_diff ** 2 + y_diff ** 2)
        if data[row,col] < 210 and dist > min_dist:
            rand_goals.append([col, row, theta])
    return rand_goals

def get_quaternion_from_euler(roll, pitch, yaw):
    quad = []
    """
    Convert an Euler angle to a quaternion.
    
    Input
        :param roll: The roll (rotation around x-axis) angle in radians.
        :param pitch: The pitch (rotation around y-axis) angle in radians.
        :param yaw: The yaw (rotation around z-axis) angle in radians.
    
    Output
        :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
    """
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    quad = [qx, qy, qz, qw]
    return quad

# Important notes: need to remove the localization from the robot navigator
def main():
    rclpy.init()

    navigator = BasicNavigator()

    #Set initial pose
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    # TODO(jwallace42): Why does this not set the initial pose correctly? (steve) Need to launch whole stack?
    initial_pose.pose.position.x = 0.0
    initial_pose.pose.position.y = -2.0
    initial_pose.pose.orientation.z = 1.0
    initial_pose.pose.orientation.w = 0.0
    navigator.setInitialPose(initial_pose)

    map_origin = [5.0, 10.0]

    # Wait for navigation to fully activate, since autostarting nav2
    navigator.waitUntilNav2Active()

    # If desired, you can change or load the map as well
    # navigator.changeMap('/home/josh/nav2_ws/install/nav2_planner_metrics/share/nav2_planner_metrics/random_obs_map.yaml')

    # Get the costmap for validating goals
    costmap = navigator.getGlobalCostmap()
    data = np.asarray(costmap.data)
    data.resize(costmap.metadata.size_y, costmap.metadata.size_x)

    # Get random goals
    print("random goal")
    print("Start pose3:", initial_pose.pose.position.y)
    seed(27)
    goals = 10
    min_dist = 3.0
    rand_goals = getRandomGoals(goals, min_dist, data, costmap.metadata.resolution, map_origin)
    
    #Get the results for the following planners 
    planners = ['NavFn', 'SmacLattice', 'Smac2D', 'SmacHybrid']
    results = [] 

    print("Start pose4:", initial_pose.pose.position.y)
    count = 0
    print("find paths")
    for rand_goal in rand_goals:
        count = count + 1
        print(count, rand_goal)
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = rand_goal[0]*costmap.metadata.resolution - map_origin[0]
        goal_pose.pose.position.y = rand_goal[1]*costmap.metadata.resolution - map_origin[1]
        quad = get_quaternion_from_euler(0, 0, rand_goal[2])
        goal_pose.pose.orientation.x = quad[0]
        goal_pose.pose.orientation.y = quad[1]
        goal_pose.pose.orientation.z = quad[2]
        goal_pose.pose.orientation.w = quad[3]
        result = getPlannerResults(navigator, initial_pose, goal_pose, planners)
        if len(result) == len(planners):
            print('The result', result[0].path.poses[0].pose.position.y)    
            results.append(result)
        else:
            print("One of the planners was invalid")


    # #Pickle data for post proccessing 
    # # TODO(jwallace42): need to write these files to the correct location
    print("write with pickle")
    with open('/home/josh/nav2_ws/src/navigation2/nav2_planner_metrics/data/results.pickle', 'wb') as f:
        pickle.dump(results, f, pickle.HIGHEST_PROTOCOL)

    with open('/home/josh/nav2_ws/src/navigation2/nav2_planner_metrics/data/costmap.pickle', 'wb') as f:
        pickle.dump(costmap, f, pickle.HIGHEST_PROTOCOL)

    # data = np.where(data < 254, 0, data)
    # data = data * -1
    # plt.figure(3)
    # ax = sns.heatmap(data, cbar=False)
    # plt.show()

    # Clean up 
    navigator.lifecycleShutdown()
    exit(0)

if __name__ == '__main__':
    main()