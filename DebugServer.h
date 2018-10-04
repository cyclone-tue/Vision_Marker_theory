#ifndef MARKER_VISION_DEBUGSERVER_H
#define MARKER_VISION_DEBUGSERVER_H

#include <thread>
#include <sys/socket.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <opencv/cv.h>
#include "opencv/highgui.h"
using namespace std;
using namespace cv;

class DebugServer{
public:
    DebugServer(int port);

    void stop();

    void setImage(InputArray image);

private:
    thread server_thread;
    void runServer(int port);
    int socket_fd, new_socket, valread;
    struct sockaddr_in address;
    int opt = 1;
    int addrlen = sizeof(address);
    Mat image;

};
#endif //MARKER_VISION_DEBUGSERVER_H


