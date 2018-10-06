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

private:
    thread server_thread;
    void runServer(int port);
    int socket_fd, new_socket = 0;
    list<int> clients ={};
    struct sockaddr_in address;
    int opt = 1;
    int addrlen = sizeof(address);
    Mat image;
    bool running = true;
};
#endif //MARKER_VISION_DEBUGSERVER_H


