#include <ros/ros.h>
#include <stdio.h>
#include <string>
#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include <algorithm>
#include <isam/isam.h>
#include <apriltags2_ros/AprilTagDetection.h>
#include <apriltags2_ros/AprilTagDetectionArray.h>
#include "geometry_msgs/PoseStamped.h"
#include <visualization_msgs/Marker.h>
#include <std_msgs/Int64.h>
#include <boost/math/distributions/chi_squared.hpp>
#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <boost/algorithm/string.hpp>
//#include <tf.h>
//#include <Vector3.h>
//#include <Quaternion.h>
#include <tf/LinearMath/Transform.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Vector3.h>


using namespace std;
using namespace isam;
using namespace Eigen;

const int Pose2d::dim;
const int Pose3d::dim;

int opcount = 0;
Slam *slam_ptr;
int current_index = 0;
int last_odom_index = 0;

int draw_rate = 5;

// 2D noise
//Noise noise3 = Information(100. * eye(3));
//Noise odom_noise3 = Information(100. * eye(3));
//Noise noise2 = Information(0.1 * eye(2));
// 3D noise
Noise noise3_3D = Information(100.*eye(6));
Noise odom_noise3_3D = Information(100.*eye(6));
Noise noise2_3D = Information(1. * eye(3));
Noise noise2_rot_3D = Information(1. * eye(6));

// Check if origin pose is set
bool origin_set = false;

//Important data here
std::vector<Pose3d_Node*> pose_nodes; //odometry nodes
std::vector<Point3d_Node*> point_nodes; //measurement nodes
//std::vector<Point3d_Node*> watchtower_point_nodes; //watchtower measurement nodes
std::vector<Pose3d_Node*> watchtower_nodes;

std::vector<int> watchtower_id; //the AprilTags ID
std::vector<int> watchtower_timestamp; //tuple with watchtower_id
std::vector<Pose3d_Pose3d_Factor*> watchtower_factors;
std::vector<std::vector<int> > watchtower_factors_timestamp;

uint32_t shape = visualization_msgs::Marker::SPHERE;
visualization_msgs::Marker *marker_ptr;
ros::Publisher *marker_pub_ptr;

void set_origin() {
  Pose3d_Node* new_pose_node = new Pose3d_Node();
  slam_ptr->add_node(new_pose_node);
  Pose3d origin(0, 0, 0, 0, 0, 0);
  pose_nodes.push_back(new_pose_node);
  Pose3d_Factor* prior = new Pose3d_Factor(pose_nodes[0], origin, noise2_rot_3D);
  slam_ptr->add_factor(prior);

  origin_set = true;
}

void odo_callback(const geometry_msgs::PoseStamped::ConstPtr& msg){

  //calculate pose
  double x = msg->pose.position.x/1000;
  double y = msg->pose.position.y/1000;
  double z = msg->pose.position.z/1000;
  tf::Quaternion orie(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
  double roll, pitch, yaw;
  tf::Matrix3x3 m(orie);
  m.getRPY(roll, pitch, yaw);

  if(origin_set == false){
    set_origin();
    return;
  }

  //cout << "wheel odom: " << x << " " << y << " " << yaw << endl;
  //add node to slam graph
  Pose3d_Node* new_pose_node = new Pose3d_Node();
  slam_ptr->add_node(new_pose_node);

  pose_nodes.push_back(new_pose_node);
  Pose3d odometry(x, y, z, yaw, pitch, roll);
  //cout << last_odom_index << " " << current_index << " " << int(pose_nodes.end()-pose_nodes.begin()) << endl;
  //cout << pose_nodes[last_odom_index]->value() << " " << pose_nodes[current_index]->value() << endl;
  Pose3d_Pose3d_Factor* constraint = new Pose3d_Pose3d_Factor(pose_nodes[last_odom_index], pose_nodes[current_index], odometry, odom_noise3_3D);
  slam_ptr->add_factor(constraint);
  last_odom_index = current_index;
  current_index++;

  for (int it=0; it< pose_nodes.end()-pose_nodes.begin(); it++){
    cout << "pose " << it << ": ";
    cout << "x=" << pose_nodes[it]->value().x();
    cout << ", y=" << pose_nodes[it]->value().y();
    cout << ", z=" << pose_nodes[it]->value().z()<<endl;
  }

}

void process(){
  if(opcount >= draw_rate){
    //optimize slam
    ros::Time optimize_start_time = ros::Time::now();
    cout << "batch_optimization" << endl;
    slam_ptr->batch_optimization();
    ros::Time optimize_end_time = ros::Time::now();
    slam_ptr->save("output.graph");
    opcount = 0;
  }
  opcount++;

  ros::Time draw_start_time = ros::Time::now();
  int it=0;
  //marker_ptr->action = 3; //DELETEALL
  //marker_pub_ptr->publish(*marker_ptr);
  for (it=0; it< pose_nodes.end()-pose_nodes.begin(); it+=1){

    marker_ptr->id = it;
    marker_ptr->action = visualization_msgs::Marker::ADD;
    marker_ptr->pose.position.x = pose_nodes[it]->value().x();
    marker_ptr->pose.position.y = pose_nodes[it]->value().y();
    marker_ptr->pose.position.z = pose_nodes[it]->value().z();
    marker_ptr->pose.orientation.x = 0.0;
    marker_ptr->pose.orientation.y = 0.0;
    marker_ptr->pose.orientation.z = 0.0;
    marker_ptr->pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker_ptr->scale.x = 0.1;
    marker_ptr->scale.y = 0.1;
    marker_ptr->scale.z = 0.1;

    // Set the color -- be sure to set alpha to something non-zero!
    marker_ptr->color.r = 1.0f;
    marker_ptr->color.g = 0.0f;
    marker_ptr->color.b = 1.0f;
    marker_ptr->color.a = 1.0;

    marker_ptr->lifetime = ros::Duration();
    marker_pub_ptr->publish(*marker_ptr);


    //cout << pose_nodes[it]->value().x() << endl;
  }

  ros::Time draw_end_time = ros::Time::now();
  //cout << "drawing duration: " << draw_end_time-draw_start_time << endl;
}

void spinOnce(){
   process();
   ros::spinOnce();
}

void spin(){
  ros::Rate rate(draw_rate);
  while (ros::ok())
  {
    spinOnce();
    rate.sleep();
   }
}

int main(int argc, char **argv){
  Slam slam;
  slam_ptr = &slam;

  // add init node
  // We now add the origin pose while first see the robot
  // by watchtower.
  // See watchtowercallback

  current_index++;
  ros::init(argc, argv, "pose_fusing");

  ros::NodeHandle n;

  visualization_msgs::Marker marker;
  marker_ptr = &marker;
  marker.header.frame_id = "/map";
  marker.header.stamp = ros::Time::now();
  marker.ns = "odom";
  marker.type = shape;

  ros::Subscriber odomSub = n.subscribe("/odom_uwb/odom", 10, odo_callback);

  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_odom_marker", 1);
  marker_pub_ptr = &marker_pub;
  spin();
  return 0;
}
