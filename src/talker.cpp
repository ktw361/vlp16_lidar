#include "ros/ros.h"
#include "vlp16_lidar/T_Msg_3D16_GRID_TO_FU.h"
#include "vlp16_lidar/T_Msg_3D16_OBS_TO_FU.h"
#include "vlp16_lidar/UdpClient.h"
#include "vlp16_lidar/velodyne.h"
#include "vlp16_lidar/MeshGrid.h"

#include <iostream>
#include <sstream>

using namespace std;
using vlp16_lidar::T_Msg_3D16_OBS_TO_FU;
using vlp16_lidar::T_Msg_3D16_GRID_TO_FU;

int main(int argc, char **argv){
    ros::init(argc, argv, "talker_node");
    ros::NodeHandle n;
    ros::Publisher grid_pub = n.advertise<T_Msg_3D16_GRID_TO_FU>("T_3D16",1000); // Length ?? overflow?
    ros::Publisher obs_pub = n.advertise<T_Msg_3D16_OBS_TO_FU>("obs_to_fu",1000);

    ros::Rate loop_rate(10);
    int count = 0;
    int syntime;

    MeshGrid mesh;
    UdpClient udp_client;

    int recvlen;
    char buf[BUFSIZE];
    int state;
    
    while (recvlen = udp_client.receive(buf)) {
        if (!ros::ok()) break;
        if (recvlen != 1206) break; // ??

        state = mesh.consume_udp(buf);
        if (state == 1) {

            T_Msg_3D16_GRID_TO_FU grid_msg;
            T_Msg_3D16_OBS_TO_FU obs_msg;

            syntime = ros::Time::now().nsec;
            grid_msg.frameID = count;
            grid_msg.syntime = syntime;
            grid_msg.navID = 0;
            mesh.set_grid_msg(grid_msg);

            obs_msg.frameID = count;
            obs_msg.navID = 0;
            mesh.set_obs_msg(obs_msg);
            
            mesh.visualize_text();
            mesh.clear();

            grid_pub.publish(grid_msg);
            obs_pub.publish(obs_msg);

            ros::spinOnce();
            loop_rate.sleep();
            ++count;
        }
    }

    cout << "should not come here" << endl;
    return 0;
}
