#include <ros/ros.h>
#include <stdio.h>
#include <string>
#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include <string>     // std::string, std::stod
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

int robot_name = 424;

//double tick2dist = ((3.14159265 * 0.13) / 5940)*0.3/0.635;
double tick2dist = ((3.1415962*0.065)/990);
double length = 0.17;

Noise noise3_3D = Information(100.*eye(6));
Noise odom_noise3_3D = Information(100.*eye(6));
Noise noise2_3D = Information(1. * eye(3));
Noise noise2_rot_3D = Information(1. * eye(6));

std::vector<int> watchtower_id; //the AprilTags ID

std::vector<Pose3d> robot_localization_node; //if we use watchtower to "localize" without isam and odometry.
std::vector<tf::Transform> watchtower_pose_node; //if we use watchtower to "localize" robots, the global pose of watchtowers are save here.
std::vector<Pose2d> ground_truth_node;

uint32_t shape = visualization_msgs::Marker::SPHERE;
visualization_msgs::Marker *marker_ptr;
ros::Publisher *marker_pub_ptr;
visualization_msgs::Marker *landmark_marker_ptr;
ros::Publisher *landmark_marker_pub_ptr;
visualization_msgs::Marker *watchtower_marker_ptr;
ros::Publisher *watchtower_marker_pub_ptr;
visualization_msgs::Marker *ground_truth_marker_ptr;
ros::Publisher *ground_truth_marker_pub_ptr;

void read_watchtower(/* arguments */) {

  string line;
  vector<string> result;
  ifstream thefile("watchtower_pose_1331.txt");

  if (thefile.is_open()){
    while ( getline (thefile, line) ){
      cout << line << '\n';
      boost::split(result, line, boost::is_any_of(","));

      if(result[0][0] == 'w'){
        int id;
        id = ((int)result[0][10] - 48)*10 + ((int)result[0][11] - 48);
        watchtower_id.push_back(id);
        cout << "The id is: " << id << endl;

        const char *r1 = result[1].c_str();
        const char *r2 = result[2].c_str();
        const char *r3 = result[3].c_str();
        const char *r4 = result[4].c_str();
        const char *r5 = result[5].c_str();
        const char *r6 = result[6].c_str();

        cout << "READ watchtower" << endl;
        cout << atof(r1) << endl;
        cout << atof(r2) << endl;
        cout << atof(r3) << endl;
        cout << atof(r4) << endl;
        cout << atof(r5) << endl;
        cout << atof(r6) << endl;

        tf::Vector3 point(atof(r1), atof(r2), atof(r3));
        tf::Quaternion orie(0, 0, 0, 1);
        orie.setRPY(atof(r6), atof(r5), atof(r4));
        //orie.setEulerZYX(atof(r4), atof(r5), atof(r6));
        tf::Transform new_pose(orie, point);

        watchtower_pose_node.push_back(new_pose);
      }
    }
    thefile.close();
    cout << "The number of watchtower: " << watchtower_pose_node.size() << endl;
  }
}

void anchorball_localization_callback(const apriltags2_ros::AprilTagDetectionArray::ConstPtr& msg, int& wt_id){

  apriltags2_ros::AprilTagDetectionArray tag_detection_array = *msg;
  int size = tag_detection_array.detections.end()-tag_detection_array.detections.begin();

  if (size > 0){
    for(int it=0; it< tag_detection_array.detections.end()-tag_detection_array.detections.begin(); it++){
      if(tag_detection_array.detections[it].id[0] != 424)
        continue; //We only care the tag for the robot

        apriltags2_ros::AprilTagDetection detection = tag_detection_array.detections[it];
        //cout << "apriltag detected id: " << detection.id << endl;
        int index = find (watchtower_id.begin(), watchtower_id.end(), wt_id) - watchtower_id.begin();

        tf::Vector3 point(detection.pose.pose.pose.position.x, detection.pose.pose.pose.position.y, detection.pose.pose.pose.position.z);
        tf::Quaternion orie(detection.pose.pose.pose.orientation.x, detection.pose.pose.pose.orientation.y, detection.pose.pose.pose.orientation.z, detection.pose.pose.pose.orientation.w);
        tf::Transform tf_wt_bot(orie, point);

        // Use the transformation from bot to the watchtower as a measurement
        if (find (watchtower_id.begin(), watchtower_id.end(), wt_id) == watchtower_id.end())
          cout << "The localization of the watchtower is unknown." << endl;
        else{
          tf::Transform pose_global_bot(tf_wt_bot);
          pose_global_bot.mult(watchtower_pose_node[index], tf_wt_bot);

          double pose_x = pose_global_bot.getOrigin().getX();
          double pose_y = pose_global_bot.getOrigin().getY();
          double pose_z = pose_global_bot.getOrigin().getZ();
          double roll, pitch, yaw;
          tf::Matrix3x3 m(pose_global_bot.getRotation());
          m.getRPY(roll, pitch, yaw);

          Pose3d new_bot_pose(pose_x, pose_y, pose_z, yaw, pitch, roll);
          robot_localization_node.push_back(new_bot_pose);
        }
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

  Pose2d new_ground_truth(pose_x, pose_y, pose_yaw);
  ground_truth_node.push_back(new_ground_truth);
}

void process(){

  ros::Time draw_start_time = ros::Time::now();
  int it=0;
  //marker_ptr->action = 3; //DELETEALL
  //marker_pub_ptr->publish(*marker_ptr);
  for (it=0; it< robot_localization_node.size(); it+=5){

    marker_ptr->id = it;
    marker_ptr->action = visualization_msgs::Marker::ADD;
    marker_ptr->pose.position.x = robot_localization_node[it].x();
    marker_ptr->pose.position.y = robot_localization_node[it].y();
    marker_ptr->pose.position.z = robot_localization_node[it].z();
    marker_ptr->pose.orientation.x = 0.0;
    marker_ptr->pose.orientation.y = 0.0;
    marker_ptr->pose.orientation.z = 0.0;
    marker_ptr->pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker_ptr->scale.x = 0.03;
    marker_ptr->scale.y = 0.03;
    marker_ptr->scale.z = 0.03;

    // Set the color -- be sure to set alpha to something non-zero!
    marker_ptr->color.r = 1.0f;
    marker_ptr->color.g = 0.0f;
    marker_ptr->color.b = 1.0f;
    marker_ptr->color.a = 1.0;

    marker_ptr->lifetime = ros::Duration();
    marker_pub_ptr->publish(*marker_ptr);

    //cout << pose_nodes[it]->value().x() << endl;
  }

  for (it=0; it< watchtower_pose_node.size(); it++){

    watchtower_marker_ptr->id = it;
    watchtower_marker_ptr->action = visualization_msgs::Marker::ADD;
    watchtower_marker_ptr->pose.position.x = watchtower_pose_node[it].getOrigin().getX();
    watchtower_marker_ptr->pose.position.y = watchtower_pose_node[it].getOrigin().getY();
    watchtower_marker_ptr->pose.position.z = watchtower_pose_node[it].getOrigin().getZ();
    watchtower_marker_ptr->pose.orientation.x = 0.0;
    watchtower_marker_ptr->pose.orientation.y = 0.0;
    watchtower_marker_ptr->pose.orientation.z = 0.0;
    watchtower_marker_ptr->pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    watchtower_marker_ptr->scale.x = 0.05;
    watchtower_marker_ptr->scale.y = 0.05;
    watchtower_marker_ptr->scale.z = 0.05;

    // Set the color -- be sure to set alpha to something non-zero!
    watchtower_marker_ptr->color.r = 1.0f;
    watchtower_marker_ptr->color.g = 1.0f;
    watchtower_marker_ptr->color.b = 0.0f;
    watchtower_marker_ptr->color.a = 1.0;

    watchtower_marker_ptr->lifetime = ros::Duration();
    marker_pub_ptr->publish(*watchtower_marker_ptr);

    //cout << pose_nodes[it]->value().x() << endl;
    //cout << pose_nodes[it]->value().y() << endl;
  }

  for (it=0; it< ground_truth_node.size(); it++){

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
}

void spinOnce(){
   process();
   ros::spinOnce();
}

void spin(){
  ros::Rate rate(15);
  while (ros::ok())
  {
    spinOnce();
    rate.sleep();
   }
}

int main(int argc, char **argv){
  if(atoi(argv[1]) > 0)
    robot_name = atoi(argv[1]);

  // add init node
  // We now add the origin pose while first see the robot
  // by watchtower.
  // See watchtowercallback

  ros::init(argc, argv, "anchorball_localization");

  ros::NodeHandle n;

  read_watchtower();

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

  ros::Subscriber watchtowerSub = n.subscribe("/erickietop/pose_optimization/bot_global_poses_optimized", 10, ground_truth_callback);

  ros::Subscriber watchtower12 = n.subscribe<apriltags2_ros::AprilTagDetectionArray> ("/watchtower12/tag_detections", 10, boost::bind(anchorball_localization_callback, _1, 12));
  ros::Subscriber watchtower13 = n.subscribe<apriltags2_ros::AprilTagDetectionArray> ("/watchtower13/tag_detections", 10, boost::bind(anchorball_localization_callback, _1, 13));
  ros::Subscriber watchtower25 = n.subscribe<apriltags2_ros::AprilTagDetectionArray> ("/watchtower25/tag_detections", 10, boost::bind(anchorball_localization_callback, _1, 25));
  ros::Subscriber watchtower26 = n.subscribe<apriltags2_ros::AprilTagDetectionArray> ("/watchtower26/tag_detections", 10, boost::bind(anchorball_localization_callback, _1, 26));
  ros::Subscriber watchtower29 = n.subscribe<apriltags2_ros::AprilTagDetectionArray> ("/watchtower29/tag_detections", 10, boost::bind(anchorball_localization_callback, _1, 29));
  //ros::Subscriber watchtower32 = n.subscribe<apriltags2_ros::AprilTagDetectionArray> ("/watchtower32/tag_detections", 10, boost::bind(watchtowercallback, _1, 32));
  ros::Subscriber watchtower33 = n.subscribe<apriltags2_ros::AprilTagDetectionArray> ("/watchtower33/tag_detections", 10, boost::bind(anchorball_localization_callback, _1, 33));
  ros::Subscriber watchtower35 = n.subscribe<apriltags2_ros::AprilTagDetectionArray> ("/watchtower35/tag_detections", 10, boost::bind(anchorball_localization_callback, _1, 35));
  ros::Subscriber watchtower36 = n.subscribe<apriltags2_ros::AprilTagDetectionArray> ("/watchtower36/tag_detections", 10, boost::bind(anchorball_localization_callback, _1, 36));
  ros::Subscriber watchtower37 = n.subscribe<apriltags2_ros::AprilTagDetectionArray> ("/watchtower37/tag_detections", 10, boost::bind(anchorball_localization_callback, _1, 37));
  ros::Subscriber watchtower38 = n.subscribe<apriltags2_ros::AprilTagDetectionArray> ("/watchtower38/tag_detections", 10, boost::bind(anchorball_localization_callback, _1, 38));
  ros::Subscriber watchtower40 = n.subscribe<apriltags2_ros::AprilTagDetectionArray> ("/watchtower40/tag_detections", 10, boost::bind(anchorball_localization_callback, _1, 40));
  ros::Subscriber watchtower42 = n.subscribe<apriltags2_ros::AprilTagDetectionArray> ("/watchtower42/tag_detections", 10, boost::bind(anchorball_localization_callback, _1, 42));
  ros::Subscriber watchtower43 = n.subscribe<apriltags2_ros::AprilTagDetectionArray> ("/watchtower43/tag_detections", 10, boost::bind(anchorball_localization_callback, _1, 43));
  ros::Subscriber watchtower47 = n.subscribe<apriltags2_ros::AprilTagDetectionArray> ("/watchtower47/tag_detections", 10, boost::bind(anchorball_localization_callback, _1, 47));

  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_odom_marker", 1);
  marker_pub_ptr = &marker_pub;
  spin();
  return 0;
}
