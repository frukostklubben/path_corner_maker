#include <ros/ros.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include "ros/console.h"
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <eigen_conversions/eigen_msg.h>
#include <tf/tf.h>

visualization_msgs::Marker marker;
float kinectheight=0;
std_msgs::Float64MultiArray navigationcorners;
Eigen::MatrixXd eigennavigationcorners(10,13);

void Path_Recieved(nav_msgs::Path Path) {
	navigationcorners.data.clear();
	int k,x;
	ROS_INFO_STREAM("Path Recieved!");
	ROS_INFO_STREAM("Path length: "<< Path.poses.size());
	if(Path.poses.size()<10){
		x=Path.poses.size();
		eigennavigationcorners.resize(x,13);
	}
	else{
		x=10;	
		eigennavigationcorners.resize(10,13);
 	}			
	for (int i=0; i<x; i++){
		if(Path.poses.size()<10)
			k=i;			
		else
			k=(Path.poses.size()/10*(i+1))-1;
		
		eigennavigationcorners(i,0)=Path.poses[k].pose.position.x-0.1;
		eigennavigationcorners(i,1)=Path.poses[k].pose.position.y-0.1;
		eigennavigationcorners(i,2)=-kinectheight;
		
		eigennavigationcorners(i,3)=Path.poses[k].pose.position.x-0.1;
		eigennavigationcorners(i,4)=Path.poses[k].pose.position.y+0.1;
		eigennavigationcorners(i,5)=-kinectheight;

		eigennavigationcorners(i,6)=Path.poses[k].pose.position.x+0.1;
		eigennavigationcorners(i,7)=Path.poses[k].pose.position.y-0.1;
		eigennavigationcorners(i,8)=-kinectheight;

		eigennavigationcorners(i,9)=Path.poses[k].pose.position.x+0.1;
		eigennavigationcorners(i,10)=Path.poses[k].pose.position.y+0.1;
		eigennavigationcorners(i,11)=-kinectheight;
	}
	marker.header.frame_id= "map";
	marker.header.stamp=ros::Time();
	marker.ns="goal";
	marker.id=1;
	marker.lifetime=ros::Duration(5);
	marker.type=visualization_msgs::Marker::CYLINDER;
	marker.action=visualization_msgs::Marker::ADD;
	marker.pose.position.x=Path.poses[Path.poses.size()-1].pose.position.x;
	marker.pose.position.y=Path.poses[Path.poses.size()-1].pose.position.y;
	marker.pose.position.z=Path.poses[Path.poses.size()-1].pose.position.z;
	marker.pose.orientation.x=0.0;
	marker.pose.orientation.y=0.0;
	marker.pose.orientation.z=0.0;
	marker.pose.orientation.w=1.0;
	marker.scale.x=0.3;
	marker.scale.y=0.3;
	marker.scale.z=0.1;
	marker.color.a=1.0;
	marker.color.r=255.0;
	marker.color.g=0.0;
	marker.color.b=0.0;
	tf::matrixEigenToMsg(eigennavigationcorners, navigationcorners);

			
}
void Poses_Recieved(geometry_msgs::PoseArray PeoplePoses) {
	ROS_INFO_STREAM("Position recieved: x: " << PeoplePoses.poses[0].position.x<< ", y: "<< PeoplePoses.poses[0].position.y<< ", z: "<<PeoplePoses.poses[0].position.z);
			
}

int main(int argc, char** argv){
	ros::init(argc, argv, "topictest");
	ros::NodeHandle nh;
	ros::Publisher marker_pub=nh.advertise<visualization_msgs::Marker>("visualization_marker",1000);
	ros::Subscriber People_Poses_sub=nh.subscribe("ground_based_rgbd_people_detector/PeoplePoses",1000, Poses_Recieved);
	ros::Subscriber Path_sub=nh.subscribe("move_base/DWAPlannerROS/global_plan",1000, Path_Recieved);
	ros::Publisher corners_pub=nh.advertise<std_msgs::Float64MultiArray>("topictest/NavigationCorners",10);
while(ros::ok()){
	ros::spinOnce();
	marker_pub.publish(marker);
	corners_pub.publish(navigationcorners);
}
}

