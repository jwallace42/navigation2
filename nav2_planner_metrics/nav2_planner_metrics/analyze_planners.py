from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rcl_interfaces.msg import ParameterValue, ParameterType, Parameter
from rcl_interfaces.srv import SetParameters
import time

#Data visualization 
import numpy as np
import seaborn as sns
import matplotlib.pylab as plt


def main():
    rclpy.init()
    print("hello")

    navigator = BasicNavigator()

    # Set our demo's initial pose
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 3.45
    initial_pose.pose.position.y = 2.15
    initial_pose.pose.orientation.z = 1.0
    initial_pose.pose.orientation.w = 0.0
    navigator.setInitialPose(initial_pose)


    # Wait for navigation to fully activate, since autostarting nav2
    navigator.waitUntilNav2Active()

    # If desired, you can change or load the map as well
    # navigator.changeMap('/path/to/map.yaml')

    # Get the costmap for plotting
    costmap = navigator.getGlobalCostmap()

    # Go to our demos first goal pose
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = 42.0
    goal_pose.pose.position.y = 6.0
    goal_pose.pose.orientation.w = 1.0

    #Get the path
    path = navigator.getPath(initial_pose, goal_pose)
    
    #Plot the path
    x = []
    y = []

    for pose in path.poses:
        x.append(pose.pose.position.x/costmap.metadata.resolution + costmap.metadata.size_x/2)
        y.append(pose.pose.position.y/costmap.metadata.resolution + costmap.metadata.size_y/2)

    

    # plot the costmap
    print('Plotting the costmap and paths')

    data = np.asarray(costmap.data)
    data = np.where(data == 255, 512, data)
    data = np.where(data == 254, 385, data)
    data = data * -1
    data.resize(costmap.metadata.size_y, costmap.metadata.size_x)
    ax = sns.heatmap(data, cbar=False)
    ax.scatter(x,y, s=0.1, color='blue')
    plt.axis('off')
    plt.show()


    navigator.lifecycleShutdown()
    exit(0)

if __name__ == '__main__':
    main()