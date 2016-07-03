from Vis import Ranges
import cv2
from socket import socket, AF_INET, SOCK_STREAM
from numpy import zeros_like, array
from math import atan2, sqrt, radians, degrees, ceil


# Calculate angle error given the robot orientation and a goal point
# noinspection PyUnusedLocal
def angle_error(pos_robotg, pos_robotc, pos_f):
    phi = atan2(pos_robotc[1] - pos_robotg[1], pos_robotc[0] - pos_robotg[0])
    th = atan2(pos_f[1] - pos_robotg[1], pos_f[0] - pos_robotg[0])
    error = th - phi
    error1 = error
    error2 = error
    error3 = error
    error4 = error
    if error1 < 0:
        error1 += 180  # asumo que esta en grados, sino cambiar 180 por math.pi
    if error2 < 0:
        error2 += 360  # asumo que esta en grados, sino cambiar 360 por 2*(math.pi)
    if error3 > 90:
        error3 = 450 - error3
    else:
        error3 = 90 - error3
    error4 += ceil(-error4 / 360) * 360
    return error


# Calculate distance error as the euclidean distance between the robot centroid and a goal point
def distance_error(pos_robot, pos_f):
    error = sqrt((pos_robot[0] - pos_f[0]) ** 2 + (pos_robot[1] - pos_f[1]) ** 2)
    return error


# Callback function for selecting HSV ranges
# noinspection PyUnusedLocal
def mouse_event(event, x, y, flags, param):
    global x_co, y_co
    if event == cv2.EVENT_MOUSEMOVE:
        x_co, y_co = x, y
    if event == cv2.EVENT_LBUTTONDOWN:
        if len(goal_points) < 4:
            goal_points.append((x, y))
            print("Goal Point {}: {}".format(len(goal_points), (x, y)))
        else:
            print('HSV MOD')
            print("q: self_big")
            print("w: self_small")
            print("a: op_big")
            print("s: op_small")
            print("z: ball")
            print("x: field")
            color_key = cv2.waitKey(0)
            try:
                ranges.update(chr(color_key), hsv_img[y, x])
            except ValueError:
                print("Wrong key!")


# Generates packet to send to robot over sockets
def motors_to_bytes(left_vel, right_vel):
    # TODO Implemente normalizer
    left_vel = 100 if left_vel > 100 else (-100 if left_vel < -100 else left_vel)
    right_vel = 100 if right_vel > 100 else (-100 if right_vel < -100 else right_vel)
    if motor_debug:
        print("Constrained: ", left_vel, "; ", right_vel)
        print("Dirs: ", 1 if left_vel > 0 else 2, "; ", 1 if right_vel > 0 else 2)
    packet = bytearray()
    packet.append(abs(left_vel))
    packet.append(abs(right_vel))
    packet.append(1 if left_vel > 0 else 2)
    packet.append(1 if right_vel > 0 else 2)
    return packet


# Mapping function
# FIXME Given a neg value in a 0 to pos range, the function returns a neg value. Is that valid?
def cmap(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) // (in_max - in_min) + out_min


# Cam variables
cam_index = 0
cam_closed = True
ret = True
img = None
hsv_img = None
blurred_img = None
global_mask = None
x_co, y_co = 0, 0
prev_val = ""
value = ""
view_masks = False
view_NonE = False
debug_HSV = False
goal_points = []
key = -1
goal = (0, 0)

# WiFly Variables
IP = "192.168.0.140"
PORT = 2000
BUF_SIZE = 9
PASS = "G03"
payload_debug = False
no_fi = False
connected = no_fi

# Robot variables
# Epsilon is the maximum allowed angle error in radians(degrees)
eps_ang = radians(15)
rotate_vel = 48
move_vel = 70
guard_vel = 60
motor_debug = False
angle_debug = False
key_guard = 0

# Lab HSV Values
# lower_amarillo = array([24,157,181])
# upper_amarillo = array([34,255,255])
# lower_rojo = array([170,52,178])
# upper_rojo = array([179,255,255])
# lower_verde = array([46,185,187])
# upper_verde = array([56,255,255])
# lower_morado = array([162,10,137])
# upper_morado = array([172,255,255])
# lower_azul = array([110,120,178])
# upper_azul = array([120,255,255])
# lower_verde_cancha = array([80, 50, 50])
# upper_verde_cancha = array([90, 255, 255])

# Home HSV Values
lower_amarillo = array([12, 200, 160])
upper_amarillo = array([22, 255, 255])
lower_rojo = array([0, 190, 220])
upper_rojo = array([10, 255, 255])
lower_verde = array([23, 200, 100])
upper_verde = array([33, 255, 200])
lower_morado = array([162, 10, 137])
upper_morado = array([172, 255, 255])
lower_azul = array([110, 120, 178])
upper_azul = array([120, 255, 255])
lower_verde_cancha = array([7, 215, 127])
upper_verde_cancha = array([11, 255, 185])

# Generate ranges object, store HSV values and define centroid colors. Assignment of ranges is done
# on different lines to make it easier to swap colors on robots
ranges = Ranges(threshold=7)
ranges.self_big.low = (lower_verde, True)
ranges.self_big.high = (upper_verde, True)
ranges.self_small.low = (lower_rojo, True)
ranges.self_small.high = (upper_rojo, True)
ranges.ball.low = (lower_amarillo, True)
ranges.ball.high = (upper_amarillo, True)
ranges.op_big.low = (lower_azul, True)
ranges.op_big.high = (upper_azul, True)
ranges.op_small.low = (lower_morado, True)
ranges.op_small.high = (upper_morado, True)
ranges.field.low = (lower_verde_cancha, True)
ranges.field.high = (upper_verde_cancha, True)
ranges.def_centroid_colors()

# Create camera object and WiFly socket
# noinspection PyArgumentList
cam = cv2.VideoCapture(cam_index)
print("Init cam")
sock = socket(AF_INET, SOCK_STREAM)
if not no_fi:
    sock.connect((IP, PORT))
print("Opened socket")
# Ensure camera is connected and obtain first frame and confirmation
while cam_closed:
    if cam.isOpened():
        ret, img = cam.read()
        blurred_img = cv2.blur(img, (10, 10))
        hsv_img = cv2.cvtColor(blurred_img, cv2.COLOR_BGR2HSV)
        global_mask = zeros_like(hsv_img)
        cam_closed = not ret
print("Obtained first frame:", ret)
print("Opened cam")
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
print("Connected to bot")
# Create window and set callback.
cv2.namedWindow('Camera feed', cv2.WINDOW_AUTOSIZE)
cv2.setMouseCallback('Camera feed', mouse_event)

# Vision loop
while ret:
    masks = ranges.masking(hsv_img)
    # Calculate moments given the masks
    moments = {name: cv2.moments(img_mask) for name, img_mask in masks.items()}
    # Calculate centroids of the previous moments
    centroids = {}
    for name, moment in moments.items():
        try:
            centroids[name] = (int(moment['m10'] / moment['m00']), int(moment['m01'] / moment['m00']))
        # If the area (m00) of a mask is 0, avoid the error by setting the centroid to the origin
        except ZeroDivisionError:
            centroids[name] = (0, 0)
    # If the goal points have already been fully defined, calculate the centroid between both
    if len(goal_points) >= 4:
        centroids['goal1'] = tuple(int((front_coord + back_coord) / 2) for front_coord, back_coord in
                                   zip(goal_points[0], goal_points[1]))
        centroids['goal2'] = tuple(int((front_coord + back_coord) / 2) for front_coord, back_coord in
                                   zip(goal_points[2], goal_points[3]))
        # If the field centroid cannot be defined by HSV masking, define it by finding the average between both goals
        if centroids['field'] == (0, 0):
            centroids['field'] = tuple(int((front_coord + back_coord) / 2) for front_coord, back_coord in
                                       zip(centroids['goal1'], centroids['goal2']))
    # Draw centroids. The colors are the inverse of the average of the ranges
    for name, centroid in centroids.items():
        if centroid != (0, 0):
            img = cv2.circle(img, centroid, 3, ranges.centroid_colors[name], -1)
    # Add robot centroid
    centroids['self_robot'] = tuple(int((front_coord + back_coord) / 2) for front_coord, back_coord in
                                    zip(centroids['self_big'], centroids['self_small']))
    img = cv2.circle(img, centroids['self_robot'], 3, (255, 0, 0), -1)
    # Show HSV value under mouse
    if debug_HSV:
        value = "H: {0}, S: {1}, V: {2}".format(*[hsv_img.item(y_co, x_co, i) for i in range(3)])
        if value != prev_val:
            print(value)
        cv2.putText(img, value, (x_co, y_co), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2, cv2.LINE_AA)
    # View masks if required
    if view_masks:
        [cv2.imshow(name, elem) for name, elem in masks.items()]
    # Display Non Essential image feeds if required
    if view_NonE:
        cv2.imshow('HSV Image', hsv_img)
        cv2.imshow('Blurred image', blurred_img)
    # Update main feed
    cv2.imshow('Camera feed', img)
    # Path planning code. The main idea of the code consists of updating the final or goal position depending on
    # actions or state machines.
    payload = None
    # If all goal points are defined, initiate path planning. If not, set all motors to 0
    if len(goal_points) >= 4:
        # Approximate the robot radius
        robot_radius = distance_error(centroids['self_robot'], centroids['self_big']) * 2.5
        # For operation 9, reset state machines and goal position
        if key == 9:
            key_guard = 0
            goal = (0, 0)
        # Operation 0 seeks the ball
        elif key == 0:
            goal = centroids['ball']
        # Operation 1 seeks the center point of the field. This may be defined by masks or goals
        elif key == 1:
            goal = centroids["field"]
        # Operation 2 initiates defensive maneuvers by running the GUARD state machine
        elif key == 2:
            # State 0: Move to a goal with the same y_coord as the robot position and a x_coord 30 pixels past
            # the goal towards the center of the field
            if key_guard == 0:
                goal = (centroids['goal1'][0] + 30, centroids['self_robot'][1])
            # State 1: With the robot in the necessary x_coord, set the new goal to same x_coord with a y_coord
            # aligned the the goal centroid. The x_coord continues being the shifted goal position to avoid
            # accumulating error in the x_coord
            elif key_guard == 1:
                goal = (centroids['goal1'][0] + 30, centroids['goal1'][1])
            # State 2: Finally, the robot must guard the goal from incoming shots. This may be done in two ways:
            # Firstly, the enemy robot points can generate a linear to function which can be evaluated at the
            # current robot x_coord and return a projected ball position. However, because the goal is not much larger
            # than 2 robot lengths, we decide to map the ball y_coord to a corresponding point in the goal. This map
            # operation translates between the width of the field to width of the goal minus a robot radius at each end
            elif key_guard == 2:
                goal = cmap(centroids["ball"][1], 0, img.shape[0],
                            goal_points[0][1] + robot_radius, goal_points[1][1] - robot_radius)
        # Once the goal position has been decided, commence path planning. A (0, 0) goal position indicate a null value,
        # and all path planning must cease. Begin by calculating angle error to goal position. Default to no movement
        # if the goal is undefined.
        if goal == (0, 0):
            payload = motors_to_bytes(0, 0)
        else:
            e_ang = angle_error(centroids['self_big'], centroids['self_small'], goal)
            # If the GUARD state machine is currently guarding the goal, override the motors to only advance or reverse
            if key_guard == 2:
                if centroids["ball"][1] < centroids["self_robot"][1]:
                    payload = motors_to_bytes(-guard_vel, -guard_vel)
                else:
                    payload = motors_to_bytes(guard_vel, guard_vel)
            else:
                if abs(e_ang) > eps_ang:
                    # Rotate robot. Multiplier is -1 or 1 depending or orientation. Invert e_ang > 0 to e_ang < 0 if
                    # the robot rotates in the other direction
                    multiplier = e_ang > 0
                    payload = motors_to_bytes((1 if multiplier else -1) * rotate_vel,
                                              (-1 if multiplier else 1) * rotate_vel)
                else:
                    # Advance robot by a fixed speed
                    payload = motors_to_bytes(move_vel, move_vel)
            # End conditions. In an ideal world, the robot would advance perfectly to the specified goal position.
            # However, these conditions allow the robot centroid to be within a certain distance and/or angle error
            # to the given goal position
            dist = distance_error(centroids['self_robot'], goal)
            # For operation 0, set the deadband at the robot radius plus 30 pixels
            if key == 0:
                if dist < robot_radius + 30:
                    payload = motors_to_bytes(0, 0)
            # For operation 1, set the deadband at the robot radius plus 30 pixels
            if key == 1:
                if dist < robot_radius + 30:
                    payload = motors_to_bytes(0, 0)
            # For operation 2, set the deadband at the robot radius plus 30 pixels
            if key == 2:
                if dist < robot_radius + 30:
                    # Once the final condition is reached, change the state variable by one and limit to 2
                    key_guard += 1
                    key_guard = 2 if key_guard > 2 else key_guard
            # print("Goal to: ", goal)
            if angle_debug:
                print(degrees(e_ang))
    # Stop motors if there are undefined goal points.
    else:
        payload = motors_to_bytes(0, 0)
    if payload_debug:
        print(list(payload))
    if debug_HSV:
        prev_val = value
    # Send payload to robot
    if not no_fi:
        sock.send(payload)
    # Obtain next frame, confirmation, blurred HSV image
    ret, img = cam.read()
    blurred_img = cv2.blur(img, (10, 10))
    hsv_img = cv2.cvtColor(blurred_img, cv2.COLOR_BGR2HSV)
    # Place a 20 ms delay in the program and obtain input
    k = cv2.waitKey(20)
    # ESC
    if k == 27:
        break
    # If the pressed key is numeric (0 - 9), change from ASCII to numeric values
    elif 48 <= k <= 58:
        key = int(chr(k))

# Once out of the processing loop, deactivate the motor outputs
if not no_fi:
    sock.send(motors_to_bytes(0, 0))
# Standard exit procedure
cam.release()
cv2.destroyAllWindows()
exit()
