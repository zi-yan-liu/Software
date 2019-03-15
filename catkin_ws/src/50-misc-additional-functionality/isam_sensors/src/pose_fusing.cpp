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
int VERBOSE = 0;

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

// Check if origin pose is set
bool origin_set = false;

//For camera calibration
tf::Transform tf_atlas2tagA(tf::Quaternion(0.57420250536, 0.612767044771,-0.393220624989, 0.374413637258), tf::Vector3(0.0117409572805,0.0312471025323,0.461244193264));
tf::Transform tf_watchtower2tagA(tf::Quaternion(0.300654994718, 0.884010597326,-0.324709439412, 0.150650648601), tf::Vector3(0.169161593855,0.0125807445743,0.80034142719));
tf::Transform tf_watchtower2tagB(tf::Quaternion(0.870910770084, 0.392341929211,-0.103810166332, 0.277138395934), tf::Vector3(-0.175778692516,0.0936732249608,0.391266091833));
tf::Transform tf_atlas2tagB;

//Important data here

// 2D version
/*
vector<Pose2d_Node*> pose_nodes;
vector<Point2d_Node*> point_nodes;
std::vector<Point2d_Node*> watchtower_point_node;
vector<int> landmarks_id;//the apriltag id of landmarks
vector<int> landmarks_timestamp;//tuple with landmarks_id

vector<Pose2d_Point2d_Factor*> landmark_factors;
vector<int> good_landmark_factors;
vector< vector<int> > landmarks_factors_timestamp;

vector<int> watchtower_id;//the watchtower id of watchtower
vector<int> watchtower_timestamp;//tuple with watchtower_id
vector<Pose2d_Point2d_Factor*> watchtower_factors;
vector< vector<int> > watchtower_factors_timestamp;
*/
std::vector<Pose3d_Node*> pose_nodes; //odometry nodes
std::vector<Point3d_Node*> point_nodes; //measurement nodes
std::vector<Point3d_Node*> watchtower_point_nodes; //watchtower measurement nodes

std::vector<int> landmarks_id; //the AprilTags ID
std::vector<int> landmarks_timestamp; //tuple with landmarks_id
std::vector<int> watchtower_id; //the AprilTags ID
std::vector<int> watchtower_timestamp; //tuple with watchtower_id

std::vector<Pose3d_Point3d_Factor*> landmark_factors;
std::vector<std::vector<int> > landmarks_factors_timestamp;
std::vector<Pose3d_Point3d_Factor*> watchtower_factors;
std::vector<std::vector<int> > watchtower_factors_timestamp;

vector<Pose2d> ground_truth_node;

uint32_t shape = visualization_msgs::Marker::SPHERE;
visualization_msgs::Marker *marker_ptr;
ros::Publisher *marker_pub_ptr;
visualization_msgs::Marker *landmark_marker_ptr;
ros::Publisher *landmark_marker_pub_ptr;
visualization_msgs::Marker *watchtower_marker_ptr;
ros::Publisher *watchtower_marker_pub_ptr;
visualization_msgs::Marker *ground_truth_marker_ptr;
ros::Publisher *ground_truth_marker_pub_ptr;
int countL=0, countR=0;

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

  pose_nodes.push_back(new_pose_node);
  Pose3d odometry(x, y, 0, yaw, 0, 0);
  //cout << last_odom_index << " " << current_index << " " << int(pose_nodes.end()-pose_nodes.begin()) << endl;
  //cout << pose_nodes[last_odom_index]->value() << " " << pose_nodes[current_index]->value() << endl;
  Pose3d_Pose3d_Factor* constraint = new Pose3d_Pose3d_Factor(pose_nodes[last_odom_index], pose_nodes[current_index], odometry, odom_noise3_3D);
  slam_ptr->add_factor(constraint);
  last_odom_index = current_index;
  current_index++;

}

void apriltagscallback(const apriltags2_ros::AprilTagDetectionArray::ConstPtr& msg){

  apriltags2_ros::AprilTagDetectionArray tag_detection_array = *msg;
  int size = tag_detection_array.detections.end()-tag_detection_array.detections.begin();
  //cout << "size: " << size << endl;
  if (size > 0){
    apriltags2_ros::AprilTagDetection detection = tag_detection_array.detections[0];
    //cout << "apriltag detected id: " << detection.id << endl;
    int index = find (landmarks_id.begin(), landmarks_id.end(), detection.id[0]) - landmarks_id.begin();

    tf::Vector3 point(detection.pose.pose.pose.position.x, detection.pose.pose.pose.position.y, detection.pose.pose.pose.position.z);
    tf::Quaternion orie(detection.pose.pose.pose.orientation.x, detection.pose.pose.pose.orientation.y, detection.pose.pose.pose.orientation.z, detection.pose.pose.pose.orientation.w);
    tf::Transform tf_atlas_tag(orie, point);
    //point = tf_atlas2tagA.operator*(point);
    //point = tf_watchtower2tagB.operator*(tf_tagA2watchtower.operator*(tf_atlas2tagA.operator*(point)));
    //point = tf_atlas2tagB.operator*(point);
    //tf_atlas_tag.mult(tf_atlas2tagB, tf_atlas_tag);
    tf_atlas_tag.mult(tf_atlas2tagB, tf_atlas_tag);
    point = tf_atlas_tag.getOrigin();
    if (find (landmarks_id.begin(), landmarks_id.end(), detection.id[0]) == landmarks_id.end()){
      //cout << "new landmark " << detection.id << endl;
      landmarks_id.push_back(detection.id[0]);
      Point3d_Node* new_point_node = new Point3d_Node();
      point_nodes.push_back(new_point_node);
      landmarks_timestamp.push_back(detection.pose.header.stamp.sec);
      slam_ptr->add_node(new_point_node);

      //cout << "apriltag: " << detection.id << " " << -detection.pose.pose.position.x << " " << -detection.pose.pose.position.y << endl;

      //Point2d measurement(-detection.pose.pose.pose.position.x, -detection.pose.pose.pose.position.y);
      Point3d measurement(detection.pose.pose.pose.position.x, detection.pose.pose.pose.position.y, detection.pose.pose.pose.position.z);
      //Point2d measurement(point.getX(), point.getY());
      //cout << "The tag " << detection.id[0] << "at" << point.getX() << point.getY() << point.getZ();
      Pose3d_Point3d_Factor* constraint = new Pose3d_Point3d_Factor(pose_nodes[last_odom_index], new_point_node, measurement, noise2_3D);
      slam_ptr->add_factor(constraint);
      //landmark_factors.push_back(constraint);
    }
    else{
      //cout << "apriltag: " << landmarks_id[index] << " " << -detection.pose.pose.position.x << " " << -detection.pose.pose.position.y << endl;
      Point3d measurement(detection.pose.pose.pose.position.x, detection.pose.pose.pose.position.y, detection.pose.pose.pose.position.z);
      //Point2d measurement(point.getX(), point.getY());
      //cout << "The tag " << detection.id[0] << "at" << point.getX() << point.getY() << point.getZ();
      Pose3d_Point3d_Factor* constraint = new Pose3d_Point3d_Factor(pose_nodes[last_odom_index], point_nodes[index], measurement, noise2_3D);

      slam_ptr->add_factor(constraint);

      landmark_factors.push_back(constraint);
      vector<int> timestamp;
      timestamp.push_back(landmarks_timestamp[index]);
      timestamp.push_back(detection.pose.header.stamp.sec);
      landmarks_factors_timestamp.push_back(timestamp);

    }
  }
}

void watchtowercallback(const apriltags2_ros::AprilTagDetectionArray::ConstPtr& msg, int& wt_id){

  apriltags2_ros::AprilTagDetectionArray tag_detection_array = *msg;
  int size = tag_detection_array.detections.end()-tag_detection_array.detections.begin();
  //cout << "size: " << size << endl;
  if (size > 0){
    for(int it=0; it< tag_detection_array.detections.end()-tag_detection_array.detections.begin(); it++){
      if(tag_detection_array.detections[it].id[0] != 424)
        return; //We only care the tag for the robot

        apriltags2_ros::AprilTagDetection detection = tag_detection_array.detections[it];
        //cout << "apriltag detected id: " << detection.id << endl;
        int index = find (watchtower_id.begin(), watchtower_id.end(), wt_id) - watchtower_id.begin();

        cout << detection.pose.pose.pose.position.x << detection.pose.pose.pose.position.y << detection.pose.pose.pose.position.z << endl;

        // Use the transformation from bot to the watchtower as a measurement
        if (find (watchtower_id.begin(), watchtower_id.end(), wt_id) == watchtower_id.end()){
          cout << "if first look" << endl;
          //cout << "new landmark " << detection.id << endl;
          cout << "watchtower_id.push_back(wt_id)" << endl;
          watchtower_id.push_back(wt_id);
          cout << "Point3d_Node* new_point_node = new Point3d_Node();" << endl;
          Point3d_Node* new_point_node = new Point3d_Node();
          cout << "watchtower_point_nodes.push_back(new_point_node);" << endl;
          watchtower_point_nodes.push_back(new_point_node);
          cout << "watchtower_timestamp.push_back(detection.pose.header.stamp.sec);" << endl;
          watchtower_timestamp.push_back(detection.pose.header.stamp.sec);
          cout << "slam_ptr->add_node(new_point_node);" << endl;
          slam_ptr->add_node(new_point_node);

          //cout << "apriltag: " << detection.id << " " << -detection.pose.pose.position.x << " " << -detection.pose.pose.position.y << endl;
          cout << "Point3d measurement" << endl;
          Point3d measurement(detection.pose.pose.pose.position.x, detection.pose.pose.pose.position.y, detection.pose.pose.pose.position.z);
          cout << "Pose3d_Point3d_Factor* constraint" << endl;
          Pose3d_Point3d_Factor* constraint = new Pose3d_Point3d_Factor(pose_nodes[last_odom_index], new_point_node, measurement, noise2_3D);
          cout << "slam_ptr->add_factor(constraint);" << endl;
          slam_ptr->add_factor(constraint);
          //landmark_factors.push_back(constraint);
        }
        else{
          cout << "if second look" << endl;
          //cout << "apriltag: " << landmarks_id[index] << " " << -detection.pose.pose.position.x << " " << -detection.pose.pose.position.y << endl;
          //Point2d measurement(-detection.pose.pose.pose.position.x, -detection.pose.pose.pose.position.y);
          Point3d measurement(detection.pose.pose.pose.position.x, detection.pose.pose.pose.position.y, detection.pose.pose.pose.position.z);
          Pose3d_Point3d_Factor* constraint = new Pose3d_Point3d_Factor(pose_nodes[last_odom_index], watchtower_point_nodes[index], measurement, noise2_3D);

          slam_ptr->add_factor(constraint);

          watchtower_factors.push_back(constraint);
          vector<int> timestamp;
          timestamp.push_back(watchtower_timestamp[index]);
          timestamp.push_back(detection.pose.header.stamp.sec);
          watchtower_factors_timestamp.push_back(timestamp);
        }
    }
  }
}

void gounrd_truth_callback(const duckietown_msgs::GlobalPoseArray::ConstPtr& msg){

  cout << "in gounrd_truth_callback" << endl;

  // extract pose
  duckietown_msgs::GlobalPoseArray pose_array = *msg;
  duckietown_msgs::GlobalPose pose = pose_array.poses[0]; //only one robot in the town
  double pose_x = pose.pose.x;
  double pose_y = pose.pose.y;
  double pose_yaw = pose.pose.theta;

  if(origin_set != true){
    Pose3d_Node* new_pose_node = new Pose3d_Node();
    slam_ptr->add_node(new_pose_node);
    Pose3d origin(pose_x, pose_y, 0, pose_yaw, 0, 0);
    pose_nodes.push_back(new_pose_node);
    Pose3d_Factor* prior = new Pose3d_Factor(pose_nodes[0], origin, noise3_3D);
    slam_ptr->add_factor(prior);

    origin_set = true;
  }

  Pose2d new_ground_truth(pose_x, pose_y, pose_yaw);
  ground_truth_node.push_back(new_ground_truth);
}

double chi2(int dof){
    if(dof>0)
      return boost::math::quantile(boost::math::chi_squared(dof),0.95)/2500; //0.98/700
    else
    {
      std::cerr<<" chi2 error " << dof <<std::endl;
      return 0;
    }
}

void process(){
cout << "in process" << endl;
  if(opcount >= 30){
    //optimize slam
    ros::Time optimize_start_time = ros::Time::now();
    //slam_ptr->update();
    cout << "batch_optimization" << endl;
    slam_ptr->batch_optimization();
    ros::Time optimize_end_time = ros::Time::now();
    //slam_ptr->save("output.graph");
    //cout << "optimizing duration: " << optimize_end_time-optimize_start_time << endl;
    opcount = 0;
  }
  opcount++;

  ros::Time draw_start_time = ros::Time::now();
  int it=0;
  //marker_ptr->action = 3; //DELETEALL
  //marker_pub_ptr->publish(*marker_ptr);
  for (it=0; it< pose_nodes.end()-pose_nodes.begin(); it+=5){

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
    marker_ptr->scale.x = 0.01;
    marker_ptr->scale.y = 0.01;
    marker_ptr->scale.z = 0.01;

    // Set the color -- be sure to set alpha to something non-zero!
    marker_ptr->color.r = 1.0f;
    marker_ptr->color.g = 0.0f;
    marker_ptr->color.b = 0.0f;
    marker_ptr->color.a = 1.0;

    marker_ptr->lifetime = ros::Duration();
    marker_pub_ptr->publish(*marker_ptr);


    //cout << pose_nodes[it]->value().x() << endl;
  }
  //marker_ptr->action = 3; //DELETEALL
  //marker_pub_ptr->publish(*marker_ptr);
  for (it=0; it< point_nodes.end()-point_nodes.begin(); it++)
  {

    landmark_marker_ptr->id = it;
    landmark_marker_ptr->action = visualization_msgs::Marker::ADD;
    landmark_marker_ptr->pose.position.x = point_nodes[it]->value().x();
    landmark_marker_ptr->pose.position.y = point_nodes[it]->value().y();
    landmark_marker_ptr->pose.position.y = point_nodes[it]->value().z();
    landmark_marker_ptr->pose.position.z = 0;
    landmark_marker_ptr->pose.orientation.x = 0.0;
    landmark_marker_ptr->pose.orientation.y = 0.0;
    landmark_marker_ptr->pose.orientation.z = 0.0;
    landmark_marker_ptr->pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    landmark_marker_ptr->scale.x = 0.05;
    landmark_marker_ptr->scale.y = 0.05;
    landmark_marker_ptr->scale.z = 0.05;

    // Set the color -- be sure to set alpha to something non-zero!
    landmark_marker_ptr->color.r = 0.0f;
    landmark_marker_ptr->color.g = 1.0f;
    landmark_marker_ptr->color.b = 0.0f;
    landmark_marker_ptr->color.a = 1.0;

    landmark_marker_ptr->lifetime = ros::Duration();
    marker_pub_ptr->publish(*landmark_marker_ptr);



    //cout << pose_nodes[it]->value().x() << endl;
    //cout << pose_nodes[it]->value().y() << endl;
  }

  for (it=0; it< watchtower_point_nodes.end()-watchtower_point_nodes.begin(); it++)
  {

    watchtower_marker_ptr->id = it;
    watchtower_marker_ptr->action = visualization_msgs::Marker::ADD;
    watchtower_marker_ptr->pose.position.x = watchtower_point_nodes[it]->value().x();
    watchtower_marker_ptr->pose.position.y = watchtower_point_nodes[it]->value().y();
    watchtower_marker_ptr->pose.position.z = watchtower_point_nodes[it]->value().z();
    watchtower_marker_ptr->pose.orientation.x = 0.0;
    watchtower_marker_ptr->pose.orientation.y = 0.0;
    watchtower_marker_ptr->pose.orientation.z = 0.0;
    watchtower_marker_ptr->pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    watchtower_marker_ptr->scale.x = 0.05;
    watchtower_marker_ptr->scale.y = 0.05;
    watchtower_marker_ptr->scale.z = 0.05;

    // Set the color -- be sure to set alpha to something non-zero!
    watchtower_marker_ptr->color.r = 0.0f;
    watchtower_marker_ptr->color.g = 1.0f;
    watchtower_marker_ptr->color.b = 1.0f;
    watchtower_marker_ptr->color.a = 1.0;

    watchtower_marker_ptr->lifetime = ros::Duration();
    marker_pub_ptr->publish(*watchtower_marker_ptr);

    //cout << pose_nodes[it]->value().x() << endl;
    //cout << pose_nodes[it]->value().y() << endl;
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
    ground_truth_marker_ptr->color.g = 0.0f;
    ground_truth_marker_ptr->color.b = 1.0f;
    ground_truth_marker_ptr->color.a = 1.0;

    ground_truth_marker_ptr->lifetime = ros::Duration();
    marker_pub_ptr->publish(*ground_truth_marker_ptr);

    //cout << pose_nodes[it]->value().x() << endl;
    //cout << pose_nodes[it]->value().y() << endl;
  }

  ros::Time draw_end_time = ros::Time::now();
  //cout << "drawing duration: " << draw_end_time-draw_start_time << endl;
}


void spinOnce(){
   process();
   ros::spinOnce();
}

void spin(){
  ros::Rate rate(30);
  while (ros::ok())
  {
    spinOnce();
    rate.sleep();
   }
}

int main(int argc, char **argv){
  VERBOSE = atoi(argv[2]);
  Slam slam;
  slam_ptr = &slam;

  // add init node
  // We now add the origin pose while first see the robot
  // by watchtower.
  // See watchtowercallback

  // Transformation from bot camera to bot
  tf::Transform tf_tagA2atlas = tf_atlas2tagA.inverse();
  //tf::Transform tf_tagA2watchtower = tf_watchtower2tagA.inverse();
  tf::Transform tf_tagB2watchtower = tf_watchtower2tagB.inverse();
  //tf_atlas2tagB.mult(tf_tagA2atlas, tf_watchtower2tagA);
  //tf_atlas2tagB.mult(tf_atlas2tagB, tf_tagB2watchtower);
  tf_atlas2tagB.mult(tf_tagB2watchtower, tf_watchtower2tagA);
  tf_atlas2tagB.mult(tf_atlas2tagB, tf_tagA2atlas);
  //tf_atlas2tagB = tf_atlas2tagB.inverse();
  cout << tf_atlas2tagB.getOrigin().getX() << " " << tf_atlas2tagB.getOrigin().getY() << " " <<tf_atlas2tagB.getOrigin().getZ();


  //cout << last_odom_index << " " << current_index << " " << int(pose_nodes.end()-pose_nodes.begin()) << endl;

  current_index++;
  ros::init(argc, argv, "pose_fusing");

  ros::NodeHandle n;

  visualization_msgs::Marker marker;
  marker_ptr = &marker;
  marker.header.frame_id = "/map";
  marker.header.stamp = ros::Time::now();
  marker.ns = "odom";
  marker.type = shape;

  visualization_msgs::Marker landmark_marker;
  landmark_marker_ptr = &landmark_marker;
  landmark_marker.header.frame_id = "/map";
  landmark_marker.header.stamp = ros::Time::now();
  landmark_marker.ns = "landmark";
  landmark_marker.type = shape;

  visualization_msgs::Marker watchtower_marker;
  watchtower_marker_ptr = &watchtower_marker;
  watchtower_marker.header.frame_id = "/map";
  watchtower_marker.header.stamp = ros::Time::now();
  watchtower_marker.ns = "watchtower";
  watchtower_marker.type = shape;

  visualization_msgs::Marker ground_truth_marker;
  ground_truth_marker_ptr = &ground_truth_marker;
  ground_truth_marker.header.frame_id = "/map";
  ground_truth_marker.header.stamp = ros::Time::now();
  ground_truth_marker.ns = "ground_truth";
  ground_truth_marker.type = shape;

  ros::Subscriber watchtowerSub = n.subscribe("/erickietop/pose_optimization/bot_global_poses_optimized", 10, gounrd_truth_callback);
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
  //ros::Subscriber watchtower35 = n.subscribe<apriltags2_ros::AprilTagDetectionArray> ("/watchtower35/tag_detections", 10, boost::bind(watchtowercallback, _1, 35));
  //ros::Subscriber watchtower36 = n.subscribe<apriltags2_ros::AprilTagDetectionArray> ("/watchtower36/tag_detections", 10, boost::bind(watchtowercallback, _1, 36));
  //ros::Subscriber watchtower37 = n.subscribe<apriltags2_ros::AprilTagDetectionArray> ("/watchtower37/tag_detections", 10, boost::bind(watchtowercallback, _1, 37));
  //ros::Subscriber watchtower38 = n.subscribe<apriltags2_ros::AprilTagDetectionArray> ("/watchtower38/tag_detections", 10, boost::bind(watchtowercallback, _1, 38));
  //ros::Subscriber watchtower40 = n.subscribe<apriltags2_ros::AprilTagDetectionArray> ("/watchtower40/tag_detections", 10, boost::bind(watchtowercallback, _1, 40));
  ros::Subscriber watchtower42 = n.subscribe<apriltags2_ros::AprilTagDetectionArray> ("/watchtower42/tag_detections", 10, boost::bind(watchtowercallback, _1, 42));
  //ros::Subscriber watchtower43 = n.subscribe<apriltags2_ros::AprilTagDetectionArray> ("/watchtower43/tag_detections", 10, boost::bind(watchtowercallback, _1, 43));
  //ros::Subscriber watchtower47 = n.subscribe<apriltags2_ros::AprilTagDetectionArray> ("/watchtower47/tag_detections", 10, boost::bind(watchtowercallback, _1, 47));

  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_odom_marker", 1);
  marker_pub_ptr = &marker_pub;
  spin();
  return 0;
}
