#!/usr/bin/env python

## AIDO localization server side postprocessing
# Author: Josefine Quack, ETHZ, jquack@ethz.ch

## This script converts relative positions of the localization tool
# into a abslute positions


## coordinates: Origin in top left corner, theta counterclockwise from x-axis
#  y
#  ^
#  |
#  |
#  O -----> x

## Input:
#   - Global poses compute by absolute_from_relative_position node

## Output:
#    - Optimize pose of Duckiebot
#
#    Statistics file
#    | time | Bot ID | x | y | theta | camera observed | reference Apriltags


import rospkg
import rospy

import yaml
import numpy as np
from datetime import datetime
import math
import global_pose_functions as gposf
import copy

from duckietown_msgs.msg import RemapPoseArray, RemapPose, GlobalPoseArray, GlobalPose

# A class for optimize and managing pose of a Duckiebot
# One could in the future move this class to an independent file so that it could be used in other node.

def get_optimized_pose(x, y, thetas):

    poses_x = x
    poses_y = y
    poses_theta = thetas

    optimized_pose_x = np.mean(poses_x)
    optimized_pose_delta_x = np.sqrt(np.mean(poses_x - optimized_pose_x)**2) # Take RMSE of pose_x for delta_x
    optimized_pose_y = np.mean(poses_y)
    optimized_pose_delta_y = np.sqrt(np.mean(poses_y - optimized_pose_y)**2) # Take RMSE of pose_y for delta_y
    # Need some special way to calculate mean for angles
    # The range of input angle is +- pi
    optimized_pose_theta = average_angle(poses_theta)
    optimized_pose_delta_theta = rms_angle(poses_theta, optimized_pose_theta)

    return [optimized_pose_x, optimized_pose_y, optimized_pose_theta, optimized_pose_delta_x, optimized_pose_delta_y, optimized_pose_delta_theta]

def average_angle(thetas):

    #Ref
    # https://en.wikipedia.org/wiki/Mean_of_circular_quantities
    # https://greek0.net/blog/2016/06/14/working_with_angles/

    x = 0
    y = 0
    for theta in thetas:
        x += math.cos(theta)
        y += math.sin(theta)

    if x == 0:
        return math.copysign(math.pi/2, y)

    return math.atan2(y, x)

def rms_angle(thetas, mean):

    # It's the RMSE of angles
    # sqrt(sum d(angle_i, mean)^2)
    # d(angle1, angle2) = acos(cos(angle1-angle2))

    d_sq = []

    for theta in thetas:
        d = math.acos(math.cos(theta - mean))
        d_sq.append(d**2)

    return np.sqrt(np.mean(d_sq))

class pose_optimization(object):
    """"""
    def __init__(self):
        self.node_name = 'pose_optimization'

        # Open Output .csv file
        self.output_file_name = rospy.get_param("~output_file", "testing") + "_optimize"
        self.output_file = self.init_output_file(self.output_file_name)

        #Add Subscriber
        self.sub_pos = rospy.Subscriber("bot_global_poses", GlobalPoseArray, self.poses_callback, queue_size=1)

        #Add Publisher
        self.pub_opt_pos = rospy.Publisher("~bot_global_poses_optimized", GlobalPoseArray, queue_size=1)


    def poses_callback(self, poses):

        output_poses = GlobalPoseArray()

        bots = dict()
        for bot_pose in poses.poses:
            if bot_pose.bot_id not in bots.keys():
                bots[bot_pose.bot_id] = dict()
                bots[bot_pose.bot_id]["pose"] = GlobalPose()
                bots[bot_pose.bot_id]["all_x"] = []
                bots[bot_pose.bot_id]["all_y"] = []
                bots[bot_pose.bot_id]["all_theta"] = []
            bots[bot_pose.bot_id]["pose"].header = bot_pose.header
            bots[bot_pose.bot_id]["pose"].bot_id = bot_pose.bot_id
            for c_id in bot_pose.cam_id:
                if c_id not in bots[bot_pose.bot_id]["pose"].cam_id:
                    bots[bot_pose.bot_id]["pose"].cam_id.append(c_id)
            for t_id in bot_pose.reference_tag_id:
                if t_id not in bots[bot_pose.bot_id]["pose"].reference_tag_id:
                    bots[bot_pose.bot_id]["pose"].reference_tag_id.append(t_id)
            bots[bot_pose.bot_id]["all_x"].append(bot_pose.pose.x)
            bots[bot_pose.bot_id]["all_y"].append(bot_pose.pose.y)
            bots[bot_pose.bot_id]["all_theta"].append(bot_pose.pose.theta)

        for id in bots:
            result = get_optimized_pose(bots[id]["all_x"], bots[id]["all_y"], bots[id]["all_theta"])
            bots[id]["pose"].pose.x = result[0]
            bots[id]["pose"].pose.y = result[1]
            bots[id]["pose"].pose.theta = result[2]
            bots[id]["pose"].delta_x = result[3]
            bots[id]["pose"].delta_y = result[4]
            bots[id]["pose"].delta_theta = result[5]
            output_poses.poses.append(bots[id]["pose"])

        self.pub_opt_pos.publish(output_poses)

### ------------------ INITIALIZATION FUNCTIONS -------------------------#####

    # creates and initilizes the output file
    # INPUT:    none
    # OUTPUT:   output_file object
    def init_output_file(self, filename):
        time = "{:%Y%m%d-%H%M%S}".format(datetime.now())
        filename_dates = filename +time + ".csv"
        output_file_name = rospkg.RosPack().get_path('auto_localization') + "/config/csv/" + filename_dates
        print output_file_name
        output_file = open(output_file_name, 'w+')
        output_file.write('time, bot_ID, x, y, theta, camera_id, reference_tag_id\n')
        return output_file

### --------------------- USER OUTPUT FUNCTIONS --------------------#####
    # writes new data to the output file
    def write_data_to_output_file(self,new_data):
        for idx in xrange( 0, (len(new_data)-1) ):
            self.output_file.write(str(new_data[idx]) + ', ')
        self.output_file.write(str(new_data[len(new_data)-1]) + '\n')



### ------------------- ------- MAIN -------------------------------#####
if __name__ == '__main__':
    rospy.init_node('pose_optimization',anonymous=False)
    node = pose_optimization()
    rospy.spin()
