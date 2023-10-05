# A simple script to simulate the wifi client on the marimbabot
# Used mainly for debugging

import socket

UDP_IP = "127.0.0.1"
UDP_PORT = 8888

def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))
    
    pos = 120
    
    while True:
        data, addr = sock.recvfrom(1024)
        print(f"received message: {data}")
        data_str = data.decode()
        if data_str[0] == "p":
            sock.sendto(f"p {pos}\n".encode(), addr)
        elif data_str[0] == "l":
            sock.sendto("l 40 200 255\n".encode(), addr)
            print("limits returned")
        elif data_str[0] == "s":
            servo_pos = data_str[:-1].split(" ")[1]
            pos = int(servo_pos)
            print(f"servo pos: {servo_pos}")
            sock.sendto("ok\n".encode(), addr)

if __name__ == "__main__":
    main()