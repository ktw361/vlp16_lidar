#ifndef UDPCLIENT_H
#define UDPCLIENT_H

#include <netdb.h>
#include <sys/socket.h>
#include <arpa/inet.h>

#define BUFSIZE 				2048
#define SERVICE_PORT 			2368


class UdpClient
{
	struct sockaddr_in myaddr;	/* our address */
	struct sockaddr_in remaddr;	/* remote address */
	socklen_t addrlen = sizeof(remaddr);		/* length of addresses */
	int recvlen;			/* # bytes received */
	int fd;				/* our socket */
public:
	UdpClient();
	~UdpClient();
	int receive(char*);
};

#endif