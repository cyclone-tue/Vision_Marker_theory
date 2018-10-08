#ifndef MARKER_VISION_DEBUGSERVER_H
#define MARKER_VISION_DEBUGSERVER_H

#include <thread>
#include <sys/socket.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <opencv/cv.h>
#include <list>
#include "opencv/highgui.h"
using namespace std;
using namespace cv;

class DebugServer{
public:
    DebugServer(const string& addr, int port);

    void stop();

    void setImage(InputArray image);

    struct addrinfo * getAddressInfo();

private:
    void runServer();

    thread server_thread;
    int socket_fd;
    list<int> clients ={};
    struct sockaddr_in address;
    int opt = 1;
    int addrlen = sizeof(address);
    Mat image;
    bool running = true;
    int f_port;
    std::string f_addr;
    struct addrinfo * f_addrinfo;
};
#endif //MARKER_VISION_DEBUGSERVER_H


