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
import numpy as np

# generate random integer values
from random import seed
from random import randint
from random import uniform

def getPlannerResults(navigator, initial_pose, goal_pose, planners):
    results = []
    for planner in planners:
        path = navigator.getPath(initial_pose, goal_pose, planner)
        if path is not None:
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


def getRandomStart(costmap, max_cost, side_buffer, time_stamp, res, map_origin):
    start = PoseStamped()
    start.header.frame_id = 'map'
    start.header.stamp = time_stamp
    while True:
        row = randint(side_buffer, costmap.shape[0]-side_buffer)
        col = randint(side_buffer, costmap.shape[1]-side_buffer)

        if costmap[row, col] < max_cost:
            start.pose.position.x = col*res - map_origin[0]
            start.pose.position.y = row*res - map_origin[1]

            yaw = uniform(0,1) * 2*math.pi
            quad = get_quaternion_from_euler(0.0, 0.0, yaw)
            start.pose.orientation.x = quad[0] 
            start.pose.orientation.y = quad[1]
            start.pose.orientation.z = quad[2]
            start.pose.orientation.w = quad[3]
            break
    return start

def getRandomGoal(costmap, start, max_cost, side_buffer, time_stamp, res, map_origin):
    goal = PoseStamped()
    goal.header.frame_id = 'map'
    goal.header.stamp = time_stamp
    while True:
        row = randint(side_buffer, costmap.shape[0]-side_buffer)
        col = randint(side_buffer, costmap.shape[1]-side_buffer)

        start_x = start.pose.position.x 
        start_y = start.pose.position.y
        goal_x = col*res - map_origin[0]
        goal_y = row*res - map_origin[0]
        x_diff = goal_x - start_x
        y_diff = goal_y - start_y
        dist = math.sqrt( x_diff ** 2 + y_diff ** 2)

        if costmap[row, col] < max_cost and dist > 3.0:
            goal.pose.position.x = goal_x
            goal.pose.position.y = goal_y

            yaw = uniform(0,1) * 2*math.pi
            quad = get_quaternion_from_euler(0.0, 0.0, yaw)
            goal.pose.orientation.x = quad[0] 
            goal.pose.orientation.y = quad[1]
            goal.pose.orientation.z = quad[2]
            goal.pose.orientation.w = quad[3]
            break
    return goal

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

    map_origin = [0.0, 0.0]

    navigator = BasicNavigator()

    #Set initial pose
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 0.0
    initial_pose.pose.position.y = -2.0
    initial_pose.pose.orientation.z = 1.0
    initial_pose.pose.orientation.w = 0.0
    navigator.setInitialPose(initial_pose)

    # Wait for navigation to fully activate, since autostarting nav2
    navigator.waitUntilNav2Active()

    costmap = navigator.getGlobalCostmap()
    costmap_data = np.asarray(costmap.data)
    costmap_data.resize(costmap.metadata.size_y, costmap.metadata.size_x)

    max_cost = 210
    side_buffer = 10
    # TODO (jwallace): SmacLattice causes segfaults..... Not sure why
    # planners = ['NavFn', 'SmacLattice', 'Smac2D', 'SmacHybrid']
    planners = ['NavFn', 'Smac2D', 'SmacHybrid']
    seed(1)
    results = []
    for i in range(50):
        time_stamp = navigator.get_clock().now().to_msg()
        start = getRandomStart(costmap_data, max_cost, side_buffer, time_stamp, costmap.metadata.resolution, map_origin)
        goal = getRandomGoal(costmap_data, start, max_cost, side_buffer, time_stamp, costmap.metadata.resolution, map_origin)
        result = getPlannerResults(navigator, start, goal, planners)
        if len(result) == len(planners):  
            results.append(result)
        else:
            print("One of the planners was invalid")

    # # #Pickle data for post proccessing 
    print("Write Results...")
    with open('/home/josh/nav2_ws/src/navigation2/nav2_planner_metrics/data/results.pickle', 'wb') as f:
        pickle.dump(results, f, pickle.HIGHEST_PROTOCOL)

    with open('/home/josh/nav2_ws/src/navigation2/nav2_planner_metrics/data/costmap.pickle', 'wb') as f:
        pickle.dump(costmap, f, pickle.HIGHEST_PROTOCOL)

    with open('/home/josh/nav2_ws/src/navigation2/nav2_planner_metrics/data/planners.pickle', 'wb') as f:
        pickle.dump(planners, f, pickle.HIGHEST_PROTOCOL)

    # Clean up 
    navigator.lifecycleShutdown()
    rclpy.shutdown()
    exit(0)

if __name__ == '__main__':
    main()