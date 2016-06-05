import numpy as np
import cv2, time

x_co, y_co = 0, 0
cams = {'internal': 0, 'field': 1}

def on_mouse(event, x, y, flag, param):
    global x_co, y_co
    if event == cv2.EVENT_MOUSEMOVE:
        x_co, y_co = x, y

detector_mode = 0
prev_val = ""

if detector_mode == 0:
    img = cv2.imread('test1.jpg', cv2.IMREAD_COLOR)
    o_img = img.copy()
    b_img = cv2.blur(img, (3,3))
    hsv = cv2.cvtColor(b_img, cv2.COLOR_BGR2HSV)
    while True:
        img = o_img.copy()
        cv2.setMouseCallback("Detector", on_mouse)
        value = "H: {0}, S: {1}, V: {2}".format(*[hsv.item(y_co, x_co, i) for i in range(3)])
        if value != prev_val:
            print(value)
        cv2.putText(img, value, (x_co, y_co), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.imshow("Detector", img)
        prev_val = value
        if cv2.waitKey(10) == 27:
            break

elif detector_mode == 1:
    cam = cv2.VideoCapture(cams['internal'])
    ret, img = cam.read()
    b_img = cv2.blur(img, (3,3))
    hsv = cv2.cvtColor(b_img, cv2.COLOR_BGR2HSV)
    while ret:
        cv2.setMouseCallback("Detector", on_mouse)
        value = "H: {0}, S: {1}, V: {2}".format(*[hsv.item(y_co, x_co, i) for i in range(3)])
        if value != prev_val:
            print(value)
        cv2.putText(img, value, (x_co, y_co), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.imshow("Detector", img)
        prev_val = value
        ret, img = cam.read()
        if cv2.waitKey(10) == 27:
            break