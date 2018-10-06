import binascii
import socket
import argparse
import string
import base64
import numpy as np
from ipaddress import ip_address

import cv2

BUFFER_SIZE = 16*1000000;


def main():
    parser = argparse.ArgumentParser(description='Open debug stream')
    parser.add_argument('-ip', type=ip_address,help='The ip to connect to')
    parser.add_argument('-port', type=int,help='port to connect to')
    args = parser.parse_args()
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    print("Connecting to", str(args.ip), args.port);
    s.connect((str(args.ip), args.port))
    while True:
        data = s.recv(BUFFER_SIZE)

        try:
            frameData = base64.decodebytes(data)
            # print(frameData)
            # print(frameData.size())
            frameData = np.frombuffer(frameData, dtype=np.uint8)
            frame = cv2.imdecode(frameData, -1)
            cv2.imshow("stream", frame)
        except binascii.Error:
            print("Could not decode frame.")
        key = cv2.waitKey(1)
        if key == 27:
            break;

if __name__ == '__main__':
    main()