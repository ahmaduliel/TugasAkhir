#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Int32.h>

void clbk_laser(const sensor_msgs::LaserScan::ConstPtr& scan){
     //ROS_INFO("Scan: [%f]", msg->ranges);
	ROS_INFO("Scan...");
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "my_node");
	ros::NodeHandle n;
	ros::Subscriber sub_laser = n.subscribe("/scan",1,clbk_laser);
	ros::spin();
}
