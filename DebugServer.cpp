#include <cstring>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <iostream>
#include <netdb.h>
#include <unistd.h>
#include "DebugServer.h"
#include "base64.h"

using namespace std;
DebugServer::DebugServer(const string& addr, int port)
: f_port(port), f_addr(addr)
{
    runServer();

}

void DebugServer::setImage(InputArray img) {
    img.copyTo(image);
    Mat frame;
    resize(image, frame, Size(640, 480));
    vector<uchar> buffer;
    imencode(".bmp", frame, buffer);
    /*string encoded = base64_encode(buffer.data(), static_cast<unsigned int>(buffer.size()));
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
    removed.clear();*/

    sendto(socket_fd, frame.data, frame.total()*frame.channels(), 0, f_addrinfo->ai_addr, f_addrinfo->ai_addrlen);

}

void DebugServer::stop() {
    running = false;
}

void DebugServer::runServer() {
    char decimal_port[16];
    snprintf(decimal_port, sizeof(decimal_port), "%d", f_port);
    decimal_port[sizeof(decimal_port)/ sizeof(decimal_port[0]) - 1] = '\0';
    struct addrinfo hints;
    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_DGRAM;
    hints.ai_protocol = IPPROTO_UDP;
    int r(getaddrinfo(f_addr.c_str(), decimal_port, &hints, & f_addrinfo));
    if(r != 0 || f_addrinfo == NULL){
        perror("invalid address or port.");
        return;
    }
    socket_fd = socket(f_addrinfo->ai_family, SOCK_DGRAM | SOCK_CLOEXEC, IPPROTO_UDP);
    if(socket_fd == -1){
        freeaddrinfo(f_addrinfo);
        perror(("Could not create UDP socket for: \"" + f_addr + ":" + decimal_port +"\"").c_str());
        return;
    }
    /*if (setsockopt(DebugServer::socket_fd, SOL_SOCKET, SO_REUSEPORT | SO_REUSEADDR, &(DebugServer::opt), sizeof(DebugServer::opt))){
        perror("Could not set socket options.");
        return;
    }*/
    r= bind(socket_fd, f_addrinfo->ai_addr, f_addrinfo->ai_addrlen);
    if (r != 0){
        freeaddrinfo(f_addrinfo);
        close(socket_fd);
        perror(("Could not bind UDP socket with: \"" + f_addr + ":" + decimal_port + "\"").c_str());
        return;
    }
    /*while(running){
        if (listen(socket_fd, 3) < 0){
            perror("Could not listen for sockets.");
            return;
        }
        if((new_socket = accept(socket_fd, (struct sockaddr *)&address, (socklen_t*)&addrlen)) < 0){
            perror("Could not open new connection.");
            return;
        }
        clients.push_back(new_socket);
    }*/
    //send(new_socket, "Hello world" , strlen("Hello world"), 0);
}
