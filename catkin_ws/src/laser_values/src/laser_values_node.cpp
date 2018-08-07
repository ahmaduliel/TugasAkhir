#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Int32.h>

#include <fstream>
#include <stdio.h>
using namespace std;
char filename[20];

void clbk_laser(const sensor_msgs::LaserScan::ConstPtr& scan){
	//for(int i=0; i<1440; i++){
	//	ROS_INFO("Scan number [%d]: [%f]",i ,scan->ranges[i]);
	//}
	
	//scanf("%s",filename);

	ofstream myfile("coba.txt", ios::out | ios::binary);
	for(int i=0; i<1080; i++){
		myfile << scan->ranges[i] << "\n";
		//ROS_INFO("Data %d : [%f]",i, scan->ranges[i]);
		//myfile << "\n";
	}	
	myfile.close();
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "my_node");
	ros::NodeHandle n;
	ros::Subscriber sub_laser = n.subscribe("/scan",1,clbk_laser);
	printf("Data berhasil disimpan\n");
	ros::spin();
}
