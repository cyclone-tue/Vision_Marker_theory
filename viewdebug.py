import socket
import argparse
import string
from ipaddress import ip_address

BUFFER_SIZE = 1024;


def main():
    parser = argparse.ArgumentParser(description='Open debug stream')
    parser.add_argument('-ip', type=ip_address,help='The ip to connect to')
    parser.add_argument('-port', type=int,help='port to connect to')
    args = parser.parse_args()
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    print("Connecting to", str(args.ip), args.port);
    s.connect((str(args.ip), args.port))
    while True:
        data = s.recv(BUFFER_SIZE)
        print(data)

if __name__ == '__main__':
    main()