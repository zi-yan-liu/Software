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
#include <duckietown_msgs/GlobalPoseArray.h>
#include <duckietown_msgs/GlobalPose.h>
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
int encoderL, encoderR, last_encoderL = 0, last_encoderR = 0;
int current_index = 0;
int last_odom_index = 0;
//double tick2dist = ((3.14159265 * 0.13) / 5940)*0.3/0.635;
double tick2dist = ((3.1415962*0.065)/990);
double length = 0.17;

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
std::vector<bool> robots_origin_set;

//For camera calibration
tf::Transform tf_atlas2tagA(tf::Quaternion(0.57420250536, 0.612767044771,-0.393220624989, 0.374413637258), tf::Vector3(0.0117409572805,0.0312471025323,0.461244193264));
tf::Transform tf_watchtower2tagA(tf::Quaternion(0.300654994718, 0.884010597326,-0.324709439412, 0.150650648601), tf::Vector3(0.169161593855,0.0125807445743,0.80034142719));
tf::Transform tf_watchtower2tagB(tf::Quaternion(0.870910770084, 0.392341929211,-0.103810166332, 0.277138395934), tf::Vector3(-0.175778692516,0.0936732249608,0.391266091833));
tf::Transform tf_atlas2tagB;

//Important data here

//std::vector<Pose3d_Node*> pose_nodes; //odometry nodes
//std::vector<Point3d_Node*> point_nodes; //measurement nodes
//std::vector<Pose3d_Node*> watchtower_nodes;

std::vector<std::vector<Pose3d_Node*> > robots_pose_nodes; //odometry nodes for multiple robots
std::vector<std::vector<Pose3d_Node*> > watchtowers_pose_nodes;

//std::vector<int> watchtower_id; //the AprilTags ID
//std::vector<int> watchtower_timestamp; //tuple with watchtower_id
std::vector<std::vector<int> > watchtower_seq_id;
std::vector<std::vector<int> > watchtower_seq_timestamp;

//std::vector<Pose3d_Pose3d_Factor*> watchtower_factors;
//std::vector<std::vector<int> > watchtower_factors_timestamp;
std::vector<std::vector<Pose3d_Pose3d_Factor*> > watchtower_seq_factors;
std::vector<std::vector<std::vector<int> > >  watchtower_seq_factors_timestamp;

std::vector<Pose2d> ground_truth_node;

uint32_t shape = visualization_msgs::Marker::SPHERE;
//visualization_msgs::Marker *marker_ptr;
//visualization_msgs::Marker *watchtower_marker_ptr;
std::vector<visualization_msgs::Marker *> marker_seq_ptr;
std::vector<visualization_msgs::Marker *> watchtower_seq_marker_ptr;
visualization_msgs::Marker *ground_truth_marker_ptr;
ros::Publisher *marker_pub_ptr;

int robot_num = 0;

/*
void save_watchtower() {
  string filename = "watchtower_pose.txt";

  ofstream thefile;
  thefile.open ("watchtower_pose.txt");

  for (int it=0; it< watchtower_nodes.end()-watchtower_nodes.begin(); it++)
  {
    thefile << "watchtower" << watchtower_id[it] << ",";
    thefile << watchtower_nodes[it]->value().x() << ",";
    thefile << watchtower_nodes[it]->value().y() << ",";
    thefile << watchtower_nodes[it]->value().z() << ",";
    thefile << watchtower_nodes[it]->value().yaw() << ",";
    thefile << watchtower_nodes[it]->value().pitch() << ",";
    thefile << watchtower_nodes[it]->value().roll() << ",";
    thefile << endl;
  }
  thefile.close();
}
*/

void encoderLcallback(const std_msgs::Int64::ConstPtr& msg){
  encoderL = (msg->data);
  if (last_encoderL==0) last_encoderL = encoderL;
}

void encoderRcallback(const std_msgs::Int64::ConstPtr& msg){

  encoderR = (msg->data);
  if (last_encoderR==0) last_encoderR = encoderR;

  cout << "encoderRcallback" << endl;

  //calculate pose
  double x = 0;
  double y = ((encoderR - last_encoderR) + (encoderL-last_encoderL)) * tick2dist / 2;
  double yaw = ((encoderR - last_encoderR) - (encoderL-last_encoderL))* tick2dist  / length; //approx atan(x) = x when x is small
  last_encoderR = encoderR;
  last_encoderL = encoderL;
  //cout << "wheel odom: " << x << " " << y << " " << yaw << endl;
  //add node to slam graph
  Pose3d_Node* new_pose_node = new Pose3d_Node();
  slam_ptr->add_node(new_pose_node);

  robots_pose_nodes[robot_num].push_back(new_pose_node);
  Pose3d odometry(x, y, 0, yaw, 0, 0);
  //cout << last_odom_index << " " << current_index << " " << int(pose_nodes.end()-pose_nodes.begin()) << endl;
  //cout << pose_nodes[last_odom_index]->value() << " " << pose_nodes[current_index]->value() << endl;
  Pose3d_Pose3d_Factor* constraint = new Pose3d_Pose3d_Factor(robots_pose_nodes[robot_num][last_odom_index], robots_pose_nodes[robot_num][current_index], odometry, odom_noise3_3D);
  slam_ptr->add_factor(constraint);
  last_odom_index = current_index;
  current_index++;

}

void watchtowercallback(const apriltags2_ros::AprilTagDetectionArray::ConstPtr& msg, int& wt_id){

  apriltags2_ros::AprilTagDetectionArray tag_detection_array = *msg;
  int size = tag_detection_array.detections.end()-tag_detection_array.detections.begin();
  //cout << "size: " << size << endl;
  if (size > 0){
    for(int it=0; it< tag_detection_array.detections.end()-tag_detection_array.detections.begin(); it++){
      if(tag_detection_array.detections[it].id[0] != 424)
        continue; //We only care the tag for the robot

        apriltags2_ros::AprilTagDetection detection = tag_detection_array.detections[it];
        //cout << "apriltag detected id: " << detection.id << endl;
        int index = find (watchtower_seq_id[robot_num].begin(), watchtower_seq_id[robot_num].end(), wt_id) - watchtower_seq_id[robot_num].begin();

        tf::Vector3 point(detection.pose.pose.pose.position.x, detection.pose.pose.pose.position.y, detection.pose.pose.pose.position.z);
        tf::Quaternion orie(detection.pose.pose.pose.orientation.x, detection.pose.pose.pose.orientation.y, detection.pose.pose.pose.orientation.z, detection.pose.pose.pose.orientation.w);
        tf::Transform tf_wt_bot(orie, point);

        double pose_x = tf_wt_bot.inverse().getOrigin().getX();
        double pose_y = tf_wt_bot.inverse().getOrigin().getY();
        double pose_z = tf_wt_bot.inverse().getOrigin().getZ();
        double roll, pitch, yaw;
        tf::Matrix3x3 m(tf_wt_bot.inverse().getRotation());
        m.getRPY(roll, pitch, yaw);

        //cout << "Origin: " << detection.pose.pose.pose.position.x << detection.pose.pose.pose.position.y << detection.pose.pose.pose.position.z << endl;
        //cout << "Inverse: " << pose_x << pose_y << pose_z << endl;

        // Use the transformation from bot to the watchtower as a measurement
        if (find (watchtower_seq_id[robot_num].begin(), watchtower_seq_id[robot_num].end(), wt_id) == watchtower_seq_id[robot_num].end()){
          cout << "if first look" << endl;
          //cout << "new landmark " << detection.id << endl;
          //cout << "watchtower_id.push_back(wt_id): " << wt_id << endl;
          watchtower_seq_id[robot_num].push_back(wt_id);
          //cout << "Pose3d_Node* new_pose_node = new Pose3d_Node();" << endl;
          Pose3d_Node* new_pose_node = new Pose3d_Node();
          //cout << "watchtower_nodes.push_back(new_pose_node);" << endl;
          watchtowers_pose_nodes[robot_num].push_back(new_pose_node);
          //cout << "watchtower_timestamp.push_back(detection.pose.header.stamp.sec);" << endl;
          watchtower_seq_timestamp[robot_num].push_back(detection.pose.header.stamp.sec);
          //cout << "slam_ptr->add_node(new_pose_node);" << endl;
          slam_ptr->add_node(new_pose_node);

          //cout << "apriltag: " << detection.id << " " << -detection.pose.pose.position.x << " " << -detection.pose.pose.position.y << endl;
          //cout << "Pose3d measurement" << endl;
          Pose3d measurement(pose_x, pose_y, pose_z, yaw, pitch, roll);
          //cout << "Pose3d_Pose3d_Factor* constraint" << endl;
          Pose3d_Pose3d_Factor* constraint = new Pose3d_Pose3d_Factor(robots_pose_nodes[robot_num][last_odom_index], watchtowers_pose_nodes[robot_num][index], measurement, noise2_rot_3D);
          //cout << "slam_ptr->add_factor(constraint);" << endl;
          slam_ptr->add_factor(constraint);
          //landmark_factors.push_back(constraint);
        }
        else{
          cout << "if second look" << endl;
          //cout << "apriltag: " << landmarks_id[index] << " " << -detection.pose.pose.position.x << " " << -detection.pose.pose.position.y << endl;
          //Point2d measurement(-detection.pose.pose.pose.position.x, -detection.pose.pose.pose.position.y);
          Pose3d measurement(pose_x, pose_y, pose_z, yaw, pitch, roll);
          Pose3d_Pose3d_Factor* constraint = new Pose3d_Pose3d_Factor(robots_pose_nodes[robot_num][last_odom_index], watchtowers_pose_nodes[robot_num][index], measurement, noise2_rot_3D);

          slam_ptr->add_factor(constraint);

          watchtower_seq_factors[robot_num].push_back(constraint);
          vector<int> timestamp;
          timestamp.push_back(watchtower_seq_timestamp[robot_num][index]);
          timestamp.push_back(detection.pose.header.stamp.sec);
          watchtower_seq_factors_timestamp[robot_num].push_back(timestamp);
        }
        slam_ptr->batch_optimization();
        slam_ptr->save("output.graph");
    }
  }
}

void ground_truth_callback(const duckietown_msgs::GlobalPoseArray::ConstPtr& msg){

  cout << "in ground_truth_callback" << endl;

  // extract pose
  duckietown_msgs::GlobalPoseArray pose_array = *msg;
  duckietown_msgs::GlobalPose pose = pose_array.poses[0]; //only one robot in the town
  double pose_x = pose.pose.x;
  double pose_y = pose.pose.y;
  double pose_yaw = pose.pose.theta;

  if(robots_origin_set[robot_num] != true){
    cout << "set origin" << endl;
    Pose3d_Node* new_pose_node = new Pose3d_Node();
    slam_ptr->add_node(new_pose_node);
    Pose3d origin(pose_x, pose_y, 0, pose_yaw, 0, 0);
    robots_pose_nodes[robot_num].push_back(new_pose_node);
    Pose3d_Factor* prior = new Pose3d_Factor(robots_pose_nodes[robot_num][0], origin, noise2_rot_3D);
    slam_ptr->add_factor(prior);

    robots_origin_set[robot_num] = true;
  }

  Pose2d new_ground_truth(pose_x, pose_y, pose_yaw);
  ground_truth_node.push_back(new_ground_truth);
  cout << "finished" << endl;
}

void robot_num_callback(const std_msgs::Int64::ConstPtr& msg) {

  last_encoderL = 0, last_encoderR = 0;
  current_index = 1;
  last_odom_index = 0;

  robots_origin_set.push_back(false);

  std::vector<Pose3d_Node*> pose_nodes;
  robots_pose_nodes.push_back(pose_nodes);

  std::vector<Pose3d_Node*> watchtower_nodes;
  watchtowers_pose_nodes.push_back(watchtower_nodes);

  std::vector<int> watchtower_id;
  watchtower_seq_id.push_back(watchtower_id);
  std::vector<int> watchtower_timestamp;
  watchtower_seq_timestamp.push_back(watchtower_timestamp);

  std::vector<Pose3d_Pose3d_Factor*> watchtower_factors;
  watchtower_seq_factors.push_back(watchtower_factors);
  std::vector<std::vector<int> > watchtower_factors_timestamp;
  watchtower_seq_factors_timestamp.push_back(watchtower_factors_timestamp);

  visualization_msgs::Marker marker;
  visualization_msgs::Marker* marker_ptr = &marker;
  marker.header.frame_id = "/map";
  marker.header.stamp = ros::Time::now();
  marker.ns = "odom";
  marker.type = shape;
  marker_seq_ptr.push_back(marker_ptr);


  visualization_msgs::Marker watchtower_marker;
  visualization_msgs::Marker* watchtower_marker_ptr = &watchtower_marker;
  watchtower_marker.header.frame_id = "/map";
  watchtower_marker.header.stamp = ros::Time::now();
  watchtower_marker.ns = "watchtower";
  watchtower_marker.type = shape;
  watchtower_seq_marker_ptr.push_back(watchtower_marker_ptr);

  //robot_num = (msg->data);
  // The robot number should be determine by the msg.
  // But let use robot_num++ for temporary usage
  robot_num ++;
  cout << "The robot number: " << robot_num << endl;
}

void process(){
  if(opcount >= 1){
    //optimize slam
    ros::Time optimize_start_time = ros::Time::now();
    //slam_ptr->update();
    cout << "batch_optimization" << endl;
    slam_ptr->batch_optimization();
    ros::Time optimize_end_time = ros::Time::now();
    slam_ptr->save("output.graph");

    //save_watchtower();
    //cout << "optimizing duration: " << optimize_end_time-optimize_start_time << endl;
    opcount = 0;
  }
  opcount++;

  /*
  ros::Time draw_start_time = ros::Time::now();
  int it=0;
  for (int num=0; num <= robot_num; num++) {
    for (it=0; it< robots_pose_nodes[num].end()-robots_pose_nodes[num].begin(); it+=5){

      marker_seq_ptr[num]->id = it;
      marker_seq_ptr[num]->action = visualization_msgs::Marker::ADD;
      marker_seq_ptr[num]->pose.position.x = robots_pose_nodes[num][it]->value().x();
      marker_seq_ptr[num]->pose.position.y = robots_pose_nodes[num][it]->value().y();
      marker_seq_ptr[num]->pose.position.z = robots_pose_nodes[num][it]->value().z();
      marker_seq_ptr[num]->pose.orientation.x = 0.0;
      marker_seq_ptr[num]->pose.orientation.y = 0.0;
      marker_seq_ptr[num]->pose.orientation.z = 0.0;
      marker_seq_ptr[num]->pose.orientation.w = 1.0;

      // Set the scale of the marker -- 1x1x1 here means 1m on a side
      marker_seq_ptr[num]->scale.x = 0.01;
      marker_seq_ptr[num]->scale.y = 0.01;
      marker_seq_ptr[num]->scale.z = 0.01;

      // Set the color -- be sure to set alpha to something non-zero!
      marker_seq_ptr[num]->color.r = 1.0f;
      marker_seq_ptr[num]->color.g = 0.0f;
      marker_seq_ptr[num]->color.b = 1.0f;
      marker_seq_ptr[num]->color.a = 1.0;

      marker_seq_ptr[num]->lifetime = ros::Duration();
      marker_pub_ptr->publish(*marker_seq_ptr[num]);
    }

    for (it=0; it< watchtowers_pose_nodes[num].end()-watchtowers_pose_nodes[num].begin(); it++)
    {

      watchtower_seq_marker_ptr[num]->id = it;
      watchtower_seq_marker_ptr[num]->action = visualization_msgs::Marker::ADD;
      watchtower_seq_marker_ptr[num]->pose.position.x = watchtowers_pose_nodes[num][it]->value().x();
      watchtower_seq_marker_ptr[num]->pose.position.y = watchtowers_pose_nodes[num][it]->value().y();
      watchtower_seq_marker_ptr[num]->pose.position.z = watchtowers_pose_nodes[num][it]->value().z();
      watchtower_seq_marker_ptr[num]->pose.orientation.x = 0.0;
      watchtower_seq_marker_ptr[num]->pose.orientation.y = 0.0;
      watchtower_seq_marker_ptr[num]->pose.orientation.z = 0.0;
      watchtower_seq_marker_ptr[num]->pose.orientation.w = 1.0;

      // Set the scale of the marker -- 1x1x1 here means 1m on a side
      watchtower_seq_marker_ptr[num]->scale.x = 0.05;
      watchtower_seq_marker_ptr[num]->scale.y = 0.05;
      watchtower_seq_marker_ptr[num]->scale.z = 0.05;

      // Set the color -- be sure to set alpha to something non-zero!
      watchtower_seq_marker_ptr[num]->color.r = 1.0f;
      watchtower_seq_marker_ptr[num]->color.g = 1.0f;
      watchtower_seq_marker_ptr[num]->color.b = 0.0f;
      watchtower_seq_marker_ptr[num]->color.a = 1.0;

      watchtower_seq_marker_ptr[num]->lifetime = ros::Duration();
      marker_pub_ptr->publish(*watchtower_seq_marker_ptr[num]);
    }
  }

  for (it=0; it< ground_truth_node.end()-ground_truth_node.begin(); it++)
  {

    ground_truth_marker_ptr->id = it;
    ground_truth_marker_ptr->action = visualization_msgs::Marker::ADD;
    ground_truth_marker_ptr->pose.position.x = ground_truth_node[it].x();
    ground_truth_marker_ptr->pose.position.y = ground_truth_node[it].y();
    ground_truth_marker_ptr->pose.position.z = 0;
    ground_truth_marker_ptr->pose.orientation.x = 0.0;
    ground_truth_marker_ptr->pose.orientation.y = 0.0;
    ground_truth_marker_ptr->pose.orientation.z = 0.0;
    ground_truth_marker_ptr->pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    ground_truth_marker_ptr->scale.x = 0.01;
    ground_truth_marker_ptr->scale.y = 0.01;
    ground_truth_marker_ptr->scale.z = 0.01;

    // Set the color -- be sure to set alpha to something non-zero!
    ground_truth_marker_ptr->color.r = 0.0f;
    ground_truth_marker_ptr->color.g = 1.0f;
    ground_truth_marker_ptr->color.b = 1.0f;
    ground_truth_marker_ptr->color.a = 1.0;

    ground_truth_marker_ptr->lifetime = ros::Duration();
    marker_pub_ptr->publish(*ground_truth_marker_ptr);

    //cout << pose_nodes[it]->value().x() << endl;
    //cout << pose_nodes[it]->value().y() << endl;
  }

  ros::Time draw_end_time = ros::Time::now();
  //cout << "drawing duration: " << draw_end_time-draw_start_time << endl;
  */
}

/*
void spinOnce(){
   process();
   ros::spinOnce();
}
*/

/*
void spin(){
  ros::Rate rate(1);
  while (ros::ok())
  {
    spinOnce();
    rate.sleep();
   }
}
*/

void process_draw(const std_msgs::Int64::ConstPtr& msg){

  int it=0;
  for (int num=0; num <= robot_num; num++) {
    cout << "Robot amount: " << robot_num << endl;
    cout << "Robot Number: " << num << endl;
    cout << "robots_pose_nodes size: " << robots_pose_nodes.size() << endl;
    cout << "marker_seq_ptr size: " << marker_seq_ptr.size() << endl;
    cout << "First element" << robots_pose_nodes[num][0]->value().x() << endl;
    cout << "End first ele" << endl;
    for (it=0; it< robots_pose_nodes[num].end()-robots_pose_nodes[num].begin(); it+=5){

      marker_seq_ptr[num]->id = it;
      marker_seq_ptr[num]->action = visualization_msgs::Marker::ADD;
      marker_seq_ptr[num]->pose.position.x = robots_pose_nodes[num][it]->value().x();
      marker_seq_ptr[num]->pose.position.y = robots_pose_nodes[num][it]->value().y();
      marker_seq_ptr[num]->pose.position.z = robots_pose_nodes[num][it]->value().z();
      marker_seq_ptr[num]->pose.orientation.x = 0.0;
      marker_seq_ptr[num]->pose.orientation.y = 0.0;
      marker_seq_ptr[num]->pose.orientation.z = 0.0;
      marker_seq_ptr[num]->pose.orientation.w = 1.0;

      // Set the scale of the marker -- 1x1x1 here means 1m on a side
      marker_seq_ptr[num]->scale.x = 0.01;
      marker_seq_ptr[num]->scale.y = 0.01;
      marker_seq_ptr[num]->scale.z = 0.01;

      // Set the color -- be sure to set alpha to something non-zero!
      marker_seq_ptr[num]->color.r = 1.0f;
      marker_seq_ptr[num]->color.g = 0.0f;
      marker_seq_ptr[num]->color.b = (1.0f - 1.0*num/(robot_num+1));
      marker_seq_ptr[num]->color.a = 1.0;

      marker_seq_ptr[num]->lifetime = ros::Duration();
      marker_pub_ptr->publish(*marker_seq_ptr[num]);
      ros::Duration(0.02).sleep();
    }
    cout << "watchtowers_pose_nodes size: " << watchtowers_pose_nodes.size() << endl;
    for (it=0; it< watchtowers_pose_nodes[num].end()-watchtowers_pose_nodes[num].begin(); it++){

      watchtower_seq_marker_ptr[num]->id = it;
      watchtower_seq_marker_ptr[num]->action = visualization_msgs::Marker::ADD;
      watchtower_seq_marker_ptr[num]->pose.position.x = watchtowers_pose_nodes[num][it]->value().x();
      watchtower_seq_marker_ptr[num]->pose.position.y = watchtowers_pose_nodes[num][it]->value().y();
      watchtower_seq_marker_ptr[num]->pose.position.z = watchtowers_pose_nodes[num][it]->value().z();
      watchtower_seq_marker_ptr[num]->pose.orientation.x = 0.0;
      watchtower_seq_marker_ptr[num]->pose.orientation.y = 0.0;
      watchtower_seq_marker_ptr[num]->pose.orientation.z = 0.0;
      watchtower_seq_marker_ptr[num]->pose.orientation.w = 1.0;

      // Set the scale of the marker -- 1x1x1 here means 1m on a side
      watchtower_seq_marker_ptr[num]->scale.x = 0.05;
      watchtower_seq_marker_ptr[num]->scale.y = 0.05;
      watchtower_seq_marker_ptr[num]->scale.z = 0.05;

      // Set the color -- be sure to set alpha to something non-zero!
      watchtower_seq_marker_ptr[num]->color.r = 1.0f;
      watchtower_seq_marker_ptr[num]->color.g = 1.0f;
      watchtower_seq_marker_ptr[num]->color.b = 0.0f;
      watchtower_seq_marker_ptr[num]->color.a = 1.0;

      watchtower_seq_marker_ptr[num]->lifetime = ros::Duration();
      marker_pub_ptr->publish(*watchtower_seq_marker_ptr[num]);
    }
  }

  cout << "ground_truth " << endl;
  for (it=0; it< ground_truth_node.end()-ground_truth_node.begin(); it++){

    ground_truth_marker_ptr->id = it;
    ground_truth_marker_ptr->action = visualization_msgs::Marker::ADD;
    ground_truth_marker_ptr->pose.position.x = ground_truth_node[it].x();
    ground_truth_marker_ptr->pose.position.y = ground_truth_node[it].y();
    ground_truth_marker_ptr->pose.position.z = 0;
    ground_truth_marker_ptr->pose.orientation.x = 0.0;
    ground_truth_marker_ptr->pose.orientation.y = 0.0;
    ground_truth_marker_ptr->pose.orientation.z = 0.0;
    ground_truth_marker_ptr->pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    ground_truth_marker_ptr->scale.x = 0.01;
    ground_truth_marker_ptr->scale.y = 0.01;
    ground_truth_marker_ptr->scale.z = 0.01;

    // Set the color -- be sure to set alpha to something non-zero!
    ground_truth_marker_ptr->color.r = 0.0f;
    ground_truth_marker_ptr->color.g = 1.0f;
    ground_truth_marker_ptr->color.b = 1.0f;
    ground_truth_marker_ptr->color.a = 1.0;

    ground_truth_marker_ptr->lifetime = ros::Duration();
    marker_pub_ptr->publish(*ground_truth_marker_ptr);

    //cout << pose_nodes[it]->value().x() << endl;
    //cout << pose_nodes[it]->value().y() << endl;
  }
}

int main(int argc, char **argv){

  Slam slam;
  slam_ptr = &slam;

  // add init node
  // We now add the origin pose while first see the robot
  // by watchtower.
  // See watchtowercallback

  // Transformation from bot camera to bot

  current_index++;

  // Initialize vectors for the first robots
  // The process should be done every time when a robot join the group
  robots_origin_set.push_back(false);

  std::vector<Pose3d_Node*> pose_nodes;
  robots_pose_nodes.push_back(pose_nodes);

  std::vector<Pose3d_Node*> watchtower_nodes;
  watchtowers_pose_nodes.push_back(watchtower_nodes);

  std::vector<int> watchtower_id;
  watchtower_seq_id.push_back(watchtower_id);
  std::vector<int> watchtower_timestamp;
  watchtower_seq_timestamp.push_back(watchtower_timestamp);

  std::vector<Pose3d_Pose3d_Factor*> watchtower_factors;
  watchtower_seq_factors.push_back(watchtower_factors);
  std::vector<std::vector<int> > watchtower_factors_timestamp;
  watchtower_seq_factors_timestamp.push_back(watchtower_factors_timestamp);
  /////////////////////////////////////////////////////////////////////////

  ros::init(argc, argv, "pose_fusing");

  ros::NodeHandle n;

  visualization_msgs::Marker marker;
  visualization_msgs::Marker* marker_ptr = &marker;
  marker.header.frame_id = "/map";
  marker.header.stamp = ros::Time::now();
  marker.ns = "odom";
  marker.type = shape;
  marker_seq_ptr.push_back(marker_ptr);


  visualization_msgs::Marker watchtower_marker;
  visualization_msgs::Marker* watchtower_marker_ptr = &watchtower_marker;
  watchtower_marker.header.frame_id = "/map";
  watchtower_marker.header.stamp = ros::Time::now();
  watchtower_marker.ns = "watchtower";
  watchtower_marker.type = shape;
  watchtower_seq_marker_ptr.push_back(watchtower_marker_ptr);

  visualization_msgs::Marker ground_truth_marker;
  ground_truth_marker_ptr = &ground_truth_marker;
  ground_truth_marker.header.frame_id = "/map";
  ground_truth_marker.header.stamp = ros::Time::now();
  ground_truth_marker.ns = "ground_truth";
  ground_truth_marker.type = shape;

  ros::Subscriber watchtowerSub = n.subscribe("/erickietop/pose_optimization/bot_global_poses_optimized", 10, ground_truth_callback);
  ros::Subscriber encoderLsub = n.subscribe("/encoder_L", 1000, encoderLcallback);
  ros::Subscriber encoderRsub = n.subscribe("/encoder_R", 1000, encoderRcallback);
  //ros::Subscriber detectorLsub = n.subscribe("/atlas/tag_detections", 10, apriltagscallback);

  ros::Subscriber watchtower12 = n.subscribe<apriltags2_ros::AprilTagDetectionArray> ("/watchtower12/tag_detections", 10, boost::bind(watchtowercallback, _1, 12));
  ros::Subscriber watchtower13 = n.subscribe<apriltags2_ros::AprilTagDetectionArray> ("/watchtower13/tag_detections", 10, boost::bind(watchtowercallback, _1, 13));
  ros::Subscriber watchtower25 = n.subscribe<apriltags2_ros::AprilTagDetectionArray> ("/watchtower25/tag_detections", 10, boost::bind(watchtowercallback, _1, 25));
  ros::Subscriber watchtower26 = n.subscribe<apriltags2_ros::AprilTagDetectionArray> ("/watchtower26/tag_detections", 10, boost::bind(watchtowercallback, _1, 26));
  ros::Subscriber watchtower29 = n.subscribe<apriltags2_ros::AprilTagDetectionArray> ("/watchtower29/tag_detections", 10, boost::bind(watchtowercallback, _1, 29));
  //ros::Subscriber watchtower32 = n.subscribe<apriltags2_ros::AprilTagDetectionArray> ("/watchtower32/tag_detections", 10, boost::bind(watchtowercallback, _1, 32));
  ros::Subscriber watchtower33 = n.subscribe<apriltags2_ros::AprilTagDetectionArray> ("/watchtower33/tag_detections", 10, boost::bind(watchtowercallback, _1, 33));
  ros::Subscriber watchtower35 = n.subscribe<apriltags2_ros::AprilTagDetectionArray> ("/watchtower35/tag_detections", 10, boost::bind(watchtowercallback, _1, 35));
  ros::Subscriber watchtower36 = n.subscribe<apriltags2_ros::AprilTagDetectionArray> ("/watchtower36/tag_detections", 10, boost::bind(watchtowercallback, _1, 36));
  ros::Subscriber watchtower37 = n.subscribe<apriltags2_ros::AprilTagDetectionArray> ("/watchtower37/tag_detections", 10, boost::bind(watchtowercallback, _1, 37));
  ros::Subscriber watchtower38 = n.subscribe<apriltags2_ros::AprilTagDetectionArray> ("/watchtower38/tag_detections", 10, boost::bind(watchtowercallback, _1, 38));
  ros::Subscriber watchtower40 = n.subscribe<apriltags2_ros::AprilTagDetectionArray> ("/watchtower40/tag_detections", 10, boost::bind(watchtowercallback, _1, 40));
  ros::Subscriber watchtower42 = n.subscribe<apriltags2_ros::AprilTagDetectionArray> ("/watchtower42/tag_detections", 10, boost::bind(watchtowercallback, _1, 42));
  ros::Subscriber watchtower43 = n.subscribe<apriltags2_ros::AprilTagDetectionArray> ("/watchtower43/tag_detections", 10, boost::bind(watchtowercallback, _1, 43));
  ros::Subscriber watchtower47 = n.subscribe<apriltags2_ros::AprilTagDetectionArray> ("/watchtower47/tag_detections", 10, boost::bind(watchtowercallback, _1, 47));

  ros::Subscriber robot_num_sub = n.subscribe("/robot_number", 1, robot_num_callback);

  ros::Subscriber draw_sub = n.subscribe("/draw", 1, process_draw);

  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_odom_marker", 1);
  marker_pub_ptr = &marker_pub;
  //spin();
  ros::spin();
  return 0;
}
