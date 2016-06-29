from Vis import Ranges
import cv2
from socket import socket, AF_INET, SOCK_STREAM
from numpy import zeros_like
from math import atan2, sqrt, radians
from time import sleep


# PID vars
Kp_d = 1
Ki_d = 0
Kd_d = 0
Kp_a = 0
Ki_a = 0
Kd_a = 0
error_prev_d = 0
error_prev_a = 0
int_err_d = 0
int_err_a = 0


# pos_robotg is the center of the big pelotitaw, pos_robotc is the center of the pelitaw chiquititaw
def pid_angular(pos_robotg, pos_robotc, pos_f):
    global int_err_a
    global error_prev_a

    # phi is the angle where the robot is looking at
    phi = atan2(pos_robotc[1]-pos_robotg[1], pos_robotc[0]-pos_robotg[0])
    # th is the angle where the robot should be looking at
    th = atan2(pos_f[1]-pos_robotg[1],pos_f[0]-pos_robotg[0])
    # error is the angle between the robot's current orientation and it's desired orientation.
    error = th-phi

    delta_err_q = error - error_prev_a
    int_err_a += error
    p_power = Kp_a * error
    i_power = Ki_a * int_err_a
    d_power = Kd_d * delta_err_q
    motor_power = p_power + i_power + d_power
    error_prev_a = error
    return motor_power


def angle_error(pos_robotg, pos_robotc, pos_f):
    phi = atan2(pos_robotc[1]-pos_robotg[1], pos_robotc[0]-pos_robotg[0])
    th = atan2(pos_f[1]-pos_robotg[1],pos_f[0]-pos_robotg[0])
    error = th-phi
    return error


def distance_error(pos_robot, pos_f):
    error = sqrt((pos_robot[0]-pos_f[0])**2+(pos_robot[1]-pos_f[1])**2)
    return error


def pid_distancia(pos_robot, pos_f):
    global int_err_d
    global error_prev_d

    # error is the distance between the robot and it's desired position.
    error = sqrt((pos_robot[0]-pos_f[0])**2+(pos_robot[1]-pos_f[1])**2)

    delta_err_r = error - error_prev_d
    int_err_d += error
    p_power = Kp_d * error
    i_power = Ki_d * int_err_d
    d_power = Kd_d * delta_err_r
    motor_power = p_power + i_power + d_power
    error_prev_d = error
    return motor_power


# Callback function for selecting HSV ranges
def mouse_event(event, x, y, flags, param):
    global x_co, y_co
    if event == cv2.EVENT_MOUSEMOVE:
        x_co, y_co = x, y
    if event == cv2.EVENT_LBUTTONDOWN:
        print('in')
        key = cv2.waitKey(0)
        print('past', key)
        try:
            ranges.update(key, hsv_img[y, x])
        except ValueError:
            print("Wrong key!")


def motors_to_bytes(left_vel, right_vel):
    # TODO Implemente normalizer
    left_vel = 100 if left_vel > 100 else (-100 if left_vel < -100 else left_vel)
    right_vel = 100 if right_vel > 100 else (-100 if right_vel < -100 else right_vel)
    print("Constrained: ", left_vel, "; ", right_vel)
    print("Dirs: ", 1 if left_vel > 0 else 2, "; ", 1 if right_vel > 0 else 2)
    packet = bytearray()
    # Inverted
    # packet.append(1 if right_vel > 0 else 2)
    # packet.append(1 if left_vel > 0 else 2)
    # packet.append(abs(right_vel))
    # packet.append(abs(left_vel))
    # Normal
    packet.append(abs(left_vel))
    packet.append(abs(right_vel))
    packet.append(1 if left_vel > 0 else 2)
    packet.append(1 if right_vel > 0 else 2)
    return packet

# Cam variables
cam_index = 1
cam_closed = True
ret = True
img = None
hsv_img = None
global_mask = None
x_co, y_co = 0, 0
prev_val = ""

# WiFly Variables
# IP = "192.168.0.140"
IP = "192.168.43.114"
PORT = 2000
BUF_SIZE = 9
PASS = "G03"
no_fi = False
connected = no_fi

# Robot variables
# Epsilon is the maximum allowed angle error in radians(degrees)
eps_ang = radians(5)
rotate_vel = 30
move_vel = 80

# Generate empty ranges object to store HSV values
ranges = Ranges()
ranges.self_big.high = 45
ranges.self_big.low = 45
ranges.self_small.high = 150
ranges.self_small.low = 150
# ranges.ball.high = 30
# ranges.ball.low = 30
ranges.ball.high = 14
ranges.ball.low = 14
# ranges.op_small.high = 20
# ranges.op_small.low = 20
# ranges.op_big.high = 40
# ranges.op_big.low = 40

# Create camera object and WiFly socket
cam = cv2.VideoCapture(cam_index)
sock = socket(AF_INET, SOCK_STREAM)
if not no_fi:
    sock.connect((IP, PORT))

# Ensure camera is connected and obtain first frame and confirmation
while cam_closed:
    if cam.isOpened():
        ret, img = cam.read()
        cam_closed = not ret
        hsv_img = cv2.blur(cv2.cvtColor(img, cv2.COLOR_BGR2HSV), (3, 3))
        global_mask = zeros_like(hsv_img)


# Establish connection with WiFly module by sending passcode
while not connected:
    data = sock.recv(BUF_SIZE)
    print("Received: {}".format(data))
    if data.decode('ascii') == "PASS?":
        sock.send(PASS.encode('ascii'))
    elif data.decode('ascii') == "AOK":
        connected = True
    else:
        connected = False
        raise ConnectionRefusedError("Received no valid keyword.")

# Vision loop
while ret:
    masks = ranges.masking(hsv_img)
    # for mask in masks.values():
    #     global_mask = cv2.bitwise_or(global_mask, mask)
    # Calculate moments given the masks
    moments = {name: cv2.moments(img_mask) for name, img_mask in masks.items()}
    # Calculate centroids of the previous moments
    centroids = {}
    for name, moment in moments.items():
        try:
            centroids[name] = (int(moment['m10'] / moment['m00']), int(moment['m01'] / moment['m00']))
        except ZeroDivisionError:
            centroids[name] = (0, 0)
    # Add robot centroid
    centroids['self_robot'] = tuple(int((front_coord + back_coord) / 2) for front_coord, back_coord in
                                    zip(centroids['self_big'], centroids['self_big']))
    # Draw centroids
    for centroid in centroids.values():
        img = cv2.circle(img, centroid, 3, (0, 0, 0), -1)
    # Show HSV value under mouse
    value = "H: {0}, S: {1}, V: {2}".format(*[hsv_img.item(y_co, x_co, i) for i in range(3)])
    if value != prev_val:
        print(value)
    cv2.putText(img, value, (x_co, y_co), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2, cv2.LINE_AA)
    # Show images
    cv2.imshow('Detected objects', global_mask)
    cv2.imshow('Camera feed', img)
    # Decide whether to rotate or advance
    e_ang = angle_error(centroids['self_big'], centroids['self_big'], centroids['ball'])
    payload = None
    if abs(e_ang) > eps_ang:
        # Rotate robot. Multiplier is -1 or 1 depending or orientation. Invert e_ang > 0 to e_ang < 0 if the robot
        # rotates in the other direction
        multiplier = e_ang > 0
        payload = motors_to_bytes((1 if multiplier else -1) * rotate_vel, (-1 if multiplier else 1) * rotate_vel)
    else:
        # Advance robot. Move by fixed speed. Assume that the robot doesn't go in the reverse because I fail at life
        payload = motors_to_bytes(move_vel, move_vel)
    # Send payload to robot
    sock.send(payload)
    # Obtain next frame, confirmation, blurred HSV image and empty mask image
    ret, img = cam.read()
    hsv_img = cv2.blur(cv2.cvtColor(img, cv2.COLOR_BGR2HSV), (3, 3))
    global_mask = zeros_like(hsv_img)
    prev_val = value

    k = cv2.waitKey(20)
    if k == 27:
        break

cam.release()
cv2.destroyAllWindows()
exit()
