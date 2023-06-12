# server
import socket
import struct

# https://realpython.com/python-sockets/#socket-api-overview
# https://docs.python.org/3/howto/sockets.html#socket-howto

PORT = 5555

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.bind(('', PORT))
    s.listen()
    # s.setblocking()
    conn, addr = s.accept()
    with conn:
        print(f"Connected by {addr}")
        while True:
            data = conn.recv(1024)
            speed, turn = struct.unpack("dd", data)
            print("data:", (round(speed, 3), round(turn, 3)))
            if not data:
                break
            conn.sendall(data)