import numpy as np
import cv2

import socket


def _mouse_event(event, x, y, flags, param):
    global hsv_img
    global lower
    global upper
    global error

    if event == cv2.EVENT_LBUTTONDOWN:
        lower[0] = hsv_img[y, x, 0]
        upper[0] = hsv_img[y, x, 0]

        lower = cv2.subtract(lower, error)
        upper = cv2.add(upper, error)

        print(lower, upper)

TCP_IP = '192.168.0.140'
TCP_PORT = 2000
BUFFER_SIZE = 5
MESSAGE = "Listo"
PASS = "G03"
CONNECTED = False
# s = None
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((TCP_IP, TCP_PORT))

while True:

    data = s.recv(BUFFER_SIZE)
    print("received data:", data)

    if data == "PASS?":
        s.send(PASS.encode())
    elif data == "AOK":
        CONNECTED = True
        break
    else:
        CONNECTED = False
        break


img = 1
hsv_img = 1

lower = np.array([0, 50, 50], np.uint8)
upper = np.array([5, 255, 255], np.uint8)
error = np.array([15, 0, 0], np.uint8)

Title_tracker = "Color Tracker"
Title_original = "Original Image"

cv2.namedWindow(Title_tracker,  cv2.WINDOW_AUTOSIZE)
cv2.namedWindow(Title_original,  cv2.WINDOW_AUTOSIZE)

cv2.setMouseCallback(Title_original, _mouse_event)

capture = cv2.VideoCapture(0)

if capture.isOpened():
    capture.open(0)


while True:
    _, img = capture.read()

    cv2.imshow(Title_original, img)

    img = cv2.blur(img, (10, 10))

    hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    thres = cv2.inRange(hsv_img, lower, upper)

    moments = cv2.moments(thres, 0)

    area = moments['m00']

    if area > 1000:

        x = (np.uint32)(moments['m10']/area)
        y = (np.uint32)(moments['m01']/area)

        cv2.circle(img, (x, y), 2, (255, 255, 255), 10)

        img_aux = cv2.bitwise_and(img,img, mask= thres)

        cv2.imshow(Title_tracker, img_aux)
    k = cv2.waitKey(10)

    if k == 27:
        capture.release()
        break
    maxx = 60
    minn = -60
    menn = 0
    if k == 119 or k == 97 or k == 115 or k == 100:
        if k == 119:
            s.send((str(maxx)+","+str(maxx)+"\n").encode())
        elif k == 97:
            s.send((str(minn)+","+str(maxx)+"\n").encode())
        elif k == 100:
            s.send((str(maxx)+","+str(minn)+"\n").encode())
        elif k == 115:
            s.send((str(minn)+","+str(minn)+"\n").encode())
    if k == 113:
        s.send("0,0\n".encode())

cv2.destroyAllWindows()


