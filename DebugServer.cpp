#include <cstring>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <iostream>
#include <netdb.h>
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
    imencode(".bmp", frame, buffer);
    string encoded = base64_encode(buffer.data(), static_cast<unsigned int>(buffer.size()));
    list<int> removed = {};
    for(int c : clients){
        size_t length = strlen(encoded.c_str());
        cout << length << endl;
        //send(c, &strlen(encoded.c_str()), sizeof(size_t), 0);
        if(send(c, encoded.c_str(), strlen(encoded.c_str()), 0) == -1){
            removed.push_back(c);
            //cout << "Removed client from list" <<endl;
        }
    }
    for (int r: removed){
        clients.remove(r);
    }
    removed.clear();

}

void DebugServer::stop() {
    running = false;
}

void DebugServer::runServer(const string& addr, int port) {
    struct addrinfo hints;
    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_DGRAM;
    hints.ai_protocol = IPPROTO_UDP;
    int r(getaddrinfo())
    if ((DebugServer::socket_fd = socket(AF_INET, SOCK_DGRAM | SOCK_CLOEXEC, IPPROTO_UDP)) == 0){
        perror("Socker creation failed");
        return;
    }
    /*if (setsockopt(DebugServer::socket_fd, SOL_SOCKET, SO_REUSEPORT | SO_REUSEADDR, &(DebugServer::opt), sizeof(DebugServer::opt))){
        perror("Could not set socket options.");
        return;
    }*/
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(port);

    if(bind(DebugServer::socket_fd, (struct sockaddr *)&address, sizeof(address)) < 0){
        perror("Could not bind to port");
        return;
    }
    while(running){
        if (listen(socket_fd, 3) < 0){
            perror("Could not listen for sockets.");
            return;
        }
        if((new_socket = accept(socket_fd, (struct sockaddr *)&address, (socklen_t*)&addrlen)) < 0){
            perror("Could not open new connection.");
            return;
        }
        clients.push_back(new_socket);
    }

    return;
    //send(new_socket, "Hello world" , strlen("Hello world"), 0);
}
