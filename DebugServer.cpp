#include <cstring>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include "DebugServer.h"
#include "base64.h"

using namespace std;
DebugServer::DebugServer(int port) {
    DebugServer::server_thread = thread(&DebugServer::runServer, this, port);

}

void DebugServer::setImage(InputArray img) {
    img.copyTo(image);
    Mat frame;
    resize(image, frame, Size(640, 480));
    vector<uchar> buffer;
    imencode(".img", frame, buffer);
    string encoded = base64_encode(buffer.data(), static_cast<unsigned int>(buffer.size()));
    if(new_socket != NULL){
        send(new_socket, encoded.c_str(), sizeof(encoded));
    }


}

void DebugServer::stop() {
    terminate();
}

void DebugServer::runServer(int port) {
    if ((DebugServer::socket_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0){
        perror("Socker creation failed");
        return;
    }
    if (setsockopt(DebugServer::socket_fd, SOL_SOCKET, SO_REUSEPORT | SO_REUSEADDR, &(DebugServer::opt), sizeof(DebugServer::opt))){
        perror("Could not set socket options.");
        return;
    }
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(port);

    if(bind(DebugServer::socket_fd, (struct sockaddr *)&address, sizeof(address)) < 0){
        perror("Could not bind to port");
        return;
    }

    if (listen(socket_fd, 3) < 0){
        perror("Could not listen for sockets.");
        return;
    }
    if((new_socket = accept(socket_fd, (struct sockaddr *)&address, (socklen_t*)&addrlen)) < 0){
        perror("Could not open new connection.");
        return;
    }

    return;
    //send(new_socket, "Hello world" , strlen("Hello world"), 0);
}
