# Echo client program
import socket
import time
import cv2
import image_processing

HOST = "192.168.111.22"  # This computer - remember to set in in the robot!!!
PORT = 30002  # The same port as used by the server


print("Starting Program")
count = 0

cv2.namedWindow('Scan', cv2.WINDOW_NORMAL | cv2.WINDOW_GUI_NORMAL)

while (count < 1000):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind((HOST, PORT))  # Bind to the port
    s.listen(5)  # Now wait for client connection.
    c, addr = s.accept()  # Establish connection with client.


    # Process picture: Find location
    location = [-0.625,  0.01,  0.016,  0,  3.14, 0]
    try:
        msg = c.recv(1024)
        print(msg)
        time.sleep(1)
        if msg == b'asking_for_data':
            count = count + 1
            print("Message 1:", msg)
            time.sleep(0.5)
            time.sleep(0.5)
            c.send(str.encode("(%s, %s, %s, %s, %s, %s)" % (location[0], location[1], location[2], location[3], location[4], location[5])));
        if msg == b'take_picture':
            print("Message 2: ", msg)
            frame = cv2.imread('C:/Users/Paul Toft Duizer/Desktop/Mads Lyngsaae Nielsen/Lego robot/Test_Images/4_blocks_9.bmp')
            cv2.imshow('Scan', frame)
            cv2.waitKey(1)
            c.send(b"(1)")
    except socket.error as socketerror:
        print(count)

    #Process picture: find number

    c.close()
    s.close()

print("Program finish")