#include "ros/ros.h"
#include "vlp16_lidar/T_Msg_3D16_GRID_TO_FU.h"
#include "vlp16_lidar/T_Msg_3D16_OBS_TO_FU.h"

#include <iostream>
#include <sstream>

using namespace std;
using vlp16_lidar::T_Msg_3D16_OBS_TO_FU;
using vlp16_lidar::T_Msg_3D16_GRID_TO_FU;

int main(int argc, char **argv){
    ros::init(argc, argv, "main_node");
    ros::NodeHandle n;
    ros::Publisher grid_pub = n.advertise<T_Msg_3D16_GRID_TO_FU>("T_FU",1000); // Length ?? overflow?
    ros::Publisher obs_pub = n.advertise<T_Msg_3D16_OBS_TO_FU>("obs_to_fu",1000);

    ros::Rate loop_rate(0.2);
    int count = 0;
    int syntime;
    while (ros::ok()) {
    	T_Msg_3D16_GRID_TO_FU grid_msg;
    	T_Msg_3D16_OBS_TO_FU obs_msg;

    	syntime = ros::Time::now().nsec;
    	grid_msg.frameID = count;
    	grid_msg.syntime = syntime;
    	grid_msg.navID = 0;
    	// grid_msg.gridMsk = NULL;
    	// grid_msg.pObs = NULL;
    	grid_msg.nObs = 0;

    	obs_msg.frameID = count;
    	obs_msg.navID = 0;
    	// obs_msg.pObs = NULL;
    	obs_msg.nObs = 0;

    	grid_pub.publish(grid_msg);
    	obs_pub.publish(obs_msg);

    	ros::spinOnce();
    	loop_rate.sleep();
    	++count;
    }

    return 0;
}
