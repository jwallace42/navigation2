from nav2_msgs.action import ComputePathToPose
import numpy as np
import seaborn as sns
import matplotlib.pylab as plt
import math

import os
from ament_index_python.packages import get_package_share_directory
import pickle

def getPaths(results):
    paths = [] 
    for result in results:
        for path in result:
            paths.append(path.path)
    return paths

def getTimes(results):
    times = []
    for result in results:
        for time in result:
            times.append(time.planning_time.nanosec)
    return times

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

def plotResults(costmap, paths, map_origin):
    coords = getMapCoordsFromPaths(paths, costmap.metadata.resolution, map_origin)
    data = np.asarray(costmap.data)
    data.resize(costmap.metadata.size_y, costmap.metadata.size_x)
    data = np.where(data < 254, 0, data)
    data = data * -1
    plt.figure(3)
    ax = sns.heatmap(data, cbar=False)
    for i in range(0, len(coords), 2):
        ax.plot(coords[i], coords[i+1], linewidth=0.7)
    plt.axis('off')
    plt.show()

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

def main():
    map_origin = [5.0, 10.0]

    nav2_planner_metrics_dir = get_package_share_directory('nav2_planner_metrics')
    costmap_path = os.path.join(nav2_planner_metrics_dir, 'costmap.pickle')
    results_path = os.path.join(nav2_planner_metrics_dir, 'results.pickle')

    with open(costmap_path, 'rb') as f:
        costmap = pickle.load(f)

    with open(results_path, 'rb') as f:
        results = pickle.load(f)

    # Path results
    paths = getPaths(results)
    path_lengths = []
    for path in paths:
        path_lengths.append( getPathLength(path))
    path_lengths = np.asarray(path_lengths)
    print(len(path_lengths))
    path_lengths = np.transpose(path_lengths)
    path_lengths.resize((4,100))
    path1 = np.average(path_lengths[0])
    path2 = np.average(path_lengths[1])
    path3 = np.average(path_lengths[2])
    path4 = np.average(path_lengths[3])

    # Time results 

    times = getTimes(results)
    times = np.asarray(times)
    times = times * 0.000001; # convert nano to milli
    times.resize((100,4))
    times = np.transpose(times)
    time1 = np.average(times[0])
    time2 = np.average(times[1])
    time3 = np.average(times[2])
    time4 = np.average(times[3])

    print(path1, time1)
    print(path2, time2)
    print(path3, time3)
    print(path4, time4)
    print(times[1])

    # plotResults(costmap, paths, map_origin)

if __name__ == '__main__':
    main()