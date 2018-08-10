#include "vlp16_lidar/UdpClient.h"
#include "vlp16_lidar/velodyne.h"
#include "vlp16_lidar/MeshGrid.h"

#include <iostream>

using std::cout; using std::endl;

int
main(int argc, char **argv)
{
	MeshGrid mesh;
	UdpClient udp_client;

	int recvlen;
	char buf[BUFSIZE];
	int state;

	int count = 0;
	while(recvlen = udp_client.receive(buf))
	{
		if (recvlen != 1206) break;

		state = mesh.consume_udp(buf);
//		if (state){;
//			mesh.
//			mesh.merge_obstacles();
//			// msg = mesh.get_msg();
//			// pub.publish(msg);
//		}
	}
	cout << "should not come here" << endl;
}
