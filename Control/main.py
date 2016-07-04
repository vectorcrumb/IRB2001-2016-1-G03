from Vis import Ranges
import cv2
from datetime import datetime
from socket import socket, AF_INET, SOCK_STREAM
from numpy import array, uint8, zeros
from math import atan2, sqrt, radians, degrees, ceil, pi

packet = bytearray()
prev_packet = bytearray()


# Calculate angle error given the robot orientation and a goal point
# noinspection PyUnusedLocal
def angle_error(pos_robotg, pos_robotc, pos_f):
    pos_robotg = list(map(int, pos_robotg))
    pos_robotc = list(map(int, pos_robotc))
    pos_f = list(map(int, pos_f))
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
            print("p: coordinates")
            color_key = cv2.waitKey(0)
            if chr(color_key) == 'p':
                print("Selected Point: {}".format((x, y)))
            else:
                try:
                    ranges.update(chr(color_key), hsv_img[y, x])
                except ValueError:
                    print("Wrong key!")


# Generates packet to send to robot over sockets
def motors_to_bytes(left_vel, right_vel):
    global packet, prev_packet
    global prev_vel_l, prev_vel_r, speed_counter
    # TODO Implement normalizer
    left_vel = 100 if left_vel > 100 else (-100 if left_vel < -100 else left_vel)
    right_vel = 100 if right_vel > 100 else (-100 if right_vel < -100 else right_vel)
    if motor_debug:
        debug_dict['motor_vals'] = (left_vel, right_vel)
        debug_dict['dirs'] = (1 if left_vel > 0 else 2, 1 if right_vel > 0 else 2)
    prev_packet = list(packet)
    if speed_counter < len(start_speeds):
        if prev_packet and packet:
            if prev_packet[2] != list(packet)[2] and prev_packet[3] != list(packet)[3]:
                left_vel = (abs(left_vel) / left_vel) * start_speeds[speed_counter]
                right_vel = (abs(right_vel) / right_vel) * start_speeds[speed_counter]
            else:
                speed_counter = 0
    prev_vel_l = left_vel
    prev_vel_r = right_vel
    packet = bytearray()
    packet.append(abs(left_vel))
    packet.append(abs(right_vel))
    packet.append(1 if left_vel > 0 else 2)
    packet.append(1 if right_vel > 0 else 2)
    return packet


def write_debug_window(debug_vals):
    image = zeros((len(debug_vals) * 40, 400, 3), uint8)
    for n, item in enumerate(debug_vals.items()):
        cv2.putText(image, "{}: {}".format(*item), (10, 20 + n * 40), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (255, 255, 255), 1, cv2.LINE_AA)
    return image


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
out = None
x_co, y_co = 0, 0
prev_val = ""
value = ""
goal_points = []
key = -1
goal = (0, 0)
view_masks = False
view_NonE = False
debug_HSV = True
video_debug = True

# WiFly Variables
IP = "192.168.0.140"
PORT = 2000
BUF_SIZE = 9
PASS = "G03"
payload_debug = True
no_fi = False
connected = no_fi

# Robot variables
# Epsilon is the maximum allowed angle error in radians(degrees)
eps_ang = radians(15)
eps_guard_ang = radians(5)
rotate_vel = 50
move_vel = 80
guard_vel = 55
strike_vel = 95
dribble_vel = 51
motor_debug = True
angle_debug = True
goal_debug = True
state_G_debug = True
force_center_field = True
start_speeds = [100, 80]
prev_vel_l, prev_vel_r = 0, 0
speed_counter = 0
key_guard = 0
key_attack = 0
debug_dict = {}

# Lab HSV Values
lower_amarillo = array([24, 157, 181])
upper_amarillo = array([34, 255, 255])
lower_rojo = array([0, 200, 200])
upper_rojo = array([9, 255, 255])
lower_verde = array([46, 185, 187])
upper_verde = array([56, 255, 255])
lower_morado = array([162, 10, 137])
upper_morado = array([172, 255, 255])
lower_azul = array([100, 160, 130])
upper_azul = array([110, 200, 190])
lower_verde_cancha = array([80, 50, 50])
upper_verde_cancha = array([90, 255, 255])

# Home HSV Values
# lower_amarillo = array([24, 220, 140])
# upper_amarillo = array([34, 255, 240])
# lower_rojo = array([4, 200, 200])
# upper_rojo = array([14, 255, 255])
# lower_verde = array([40, 210, 125])
# upper_verde = array([50, 250, 135])
# lower_morado = array([15, 215, 170])
# upper_morado = array([25, 250, 190])
# lower_azul = array([95, 70, 95])
# upper_azul = array([110, 125, 160])
# lower_verde_cancha = array([15, 190, 130])
# upper_verde_cancha = array([25, 250, 185])

# Generate ranges object, store HSV values and define centroid colors. Assignment of ranges is done
# on different lines to make it easier to swap colors on robots
ranges = Ranges(threshold=7)
ranges.self_big.low = (lower_verde, True)
ranges.self_big.high = (upper_verde, True)
ranges.self_small.low = (lower_morado, True)
ranges.self_small.high = (upper_morado, True)
ranges.ball.low = (lower_amarillo, True)
ranges.ball.high = (upper_amarillo, True)
ranges.op_big.low = (lower_rojo, True)
ranges.op_big.high = (upper_rojo, True)
ranges.op_small.low = (lower_azul, True)
ranges.op_small.high = (upper_azul, True)
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

if video_debug:
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    name = 'logs/log_{}.avi'.format(datetime.now().strftime('%Y-%m-%d_%H-%M-%S'))
    debug_dict['video_file'] = name
    out = cv2.VideoWriter(name, fourcc, fps=30.0, frameSize=tuple(reversed(img.shape[:-1])))

# Vision loop
while ret:
    global count
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
    # Simulate opponent robot
    # centroids['op_big'] = (583, 167)
    # centroids['op_small'] = (541, 174)
    # If the goal points have already been fully defined, calculate the centroid between both
    if len(goal_points) >= 4:
        centroids['goal1'] = tuple(int((front_coord + back_coord) / 2) for front_coord, back_coord in
                                   zip(goal_points[0], goal_points[1]))
        centroids['goal2'] = tuple(int((front_coord + back_coord) / 2) for front_coord, back_coord in
                                   zip(goal_points[2], goal_points[3]))
        # Force center field centroid by geometry
        if force_center_field:
            centroids['field'] = (0, 0)
        # If the field centroid cannot be defined by HSV masking, define it by finding the average between both goals
        if centroids['field'] == (0, 0):
            centroids['field'] = tuple(int((front_coord + back_coord) / 2) for front_coord, back_coord in
                                       zip(centroids['goal1'], centroids['goal2']))
    # Draw centroids. The colors are the inverse of the average of the ranges\
    for name, centroid in centroids.items():
        if centroid != (0, 0):
            if name not in ['goal1', 'goal2']:
                color = list(map(int, ranges.centroid_colors[name][0][0]))
            else:
                color = ranges.centroid_colors[name]
            img = cv2.circle(img=img, center=centroid, radius=3, color=tuple(color), thickness=-1)
    # Add robot centroid if both robot markers exist. Else, set the centroid to (0, 0)
    if centroids['self_big'] != (0, 0) and centroids['self_small'] != (0, 0):
        centroids['self_robot'] = tuple(int((front_coord + back_coord) / 2) for front_coord, back_coord in
                                        zip(centroids['self_big'], centroids['self_small']))
    else:
        centroids['self_robot'] = (0, 0)
    # Draw robot centroid in blue and goal point in green
    img = cv2.circle(img, centroids['self_robot'], 3, (255, 0, 0), -1)
    img = cv2.circle(img, tuple(map(int, goal)), 3, (0, 255, 0), -1)
    # Show false goal if GUARD state is 2 or 3
    if key == 2 and key_guard == 3:
        goal = tuple(map(int, goal))
        img = cv2.circle(img, (centroids['self_big'][0], goal[1]), 3, (0, 0, 255), -1)
    # Show HSV value under mouse
    if debug_HSV:
        value = "H: {0}, S: {1}, V: {2}".format(*[hsv_img.item(y_co, x_co, i) for i in range(3)])
        if value != prev_val:
            # print(value)
            pass
        debug_dict['cursor_xy'] = (x_co, y_co)
        debug_dict['HSV_xy'] = value
        # cv2.putText(img, value, (x_co, y_co), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2, cv2.LINE_AA)
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
    # If all goal points are defined AND the robot centroid is defined to coordinates other than the origin,
    # initiate path planning. If not, set all motors to 0
    if len(goal_points) >= 4 and centroids['self_robot'] != (0, 0):
        # Approximate the robot radius
        robot_radius = distance_error(centroids['self_robot'], centroids['self_big']) * 2.5
        # Approximate the ball radius
        ball_radius = sqrt(moments['ball']['m00'] / pi) / 14.44
        # For operation 9, reset state machines and goal position
        if key == 9:
            key_guard = 0
            key_attack = 0
            goal = (0, 0)
        # Operation 0 seeks the ball
        elif key == 0:
            key_guard = 0
            key_attack = 0
            goal = centroids['ball']
        # Operation 1 seeks the center point of the field. This may be defined by masks or goals
        elif key == 1:
            key_guard = 0
            key_attack = 0
            goal = centroids["field"]
        # Operation 2 initiates defensive maneuvers by running the GUARD state machine
        elif key == 2:
            key_attack = 0
            # State 0: Move to a goal with the same y_coord as the robot position and a x_coord 30 pixels past
            # the goal towards the center of the field
            if key_guard == 0:
                goal = (centroids['goal1'][0] + 30, centroids['self_robot'][1])
            # State 1: With the robot in the necessary x_coord, set the new goal to same x_coord with a y_coord
            # aligned the the goal centroid. The x_coord continues being the shifted goal position to avoid
            # accumulating error in the x_coord
            elif key_guard == 1:
                goal = (centroids['goal1'][0] + 30, centroids['goal1'][1])
            # State 2: Align robot in an orientation parallel to the side of the field. The goal is set at the same
            # x_coord as the shifted goal and at the top y_coord position.
            elif key_guard == 2:
                goal = (centroids['goal1'][0] + 30, 0)
            # State 3: Finally, the robot must guard the goal from incoming shots. This may be done in two ways:
            # Firstly, the enemy robot points can generate a linear function which can be evaluated at the
            # current robot x_coord and return a projected ball position. However, because the goal is not much larger
            # than 2 robot lengths, we decide to map the ball y_coord to a corresponding point in the goal. This map
            # operation translates between the width of the field to width of the goal minus a robot radius at each end
            elif key_guard == 3:
                goal = (centroids['goal1'][0] + 30,
                        cmap(centroids["ball"][1], 0, img.shape[0],
                             goal_points[0][1] + robot_radius, goal_points[1][1] - robot_radius))
        # Operation 3 makes the robot dribble the ball towards the goal
        # Operation 4 makes the robot hit the ball towards the goal
        # It's the same as operation 3, except for key_attack 3
        # The first 3 states get the ball in the correct position behind the ball
        # may need to switch goal1 and goal2
        elif key == 3 or key == 4:
            key_guard = 0
            # delta_x&y are the horizontal and vertical distances between the ball and the goal
            delta_x = centroids["ball"][0] - centroids["goal2"][0]
            delta_y = centroids["ball"][1] - centroids["goal2"][1]
            # define whether to take upper or lower path
            upperPath = True
            if centroids['ball'][1] < robot_radius * 5:  # may need to change that 5
                upperPath = False
            # State 0 moves the robot vertically towards the upper or lower edge of the map,
            # depending on the path to be taken
            if key_attack == 0:
                if upperPath:
                    goal = (centroids['self_robot'][0], robot_radius * 2)
                else:
                    goal = (centroids['self_robot'][0], img.shape[0] - robot_radius * 2)
            # State 1 moves the robot horizontally, up to the point that right above (or below) the final goal point
            elif key_attack == 1:
                applySat_x = centroids["ball"][0] + delta_x > centroids['goal1'][0] - 30  # may need to change that 30
                if upperPath:
                    goal = (centroids["ball"][0] + delta_x,
                            robot_radius * 2) if not applySat_x else (centroids['goal1'][0] - 30, robot_radius * 2)
                else:
                    goal = (centroids["ball"][0] + delta_x,
                            img.shape[0] - robot_radius * 2) if not applySat_x else (centroids['goal1'][0] - 30,
                                                                                     img.shape[0] - robot_radius * 2)
            # State 2 moves the robot vertically to the final position on the ball's side that's opposite to the goal
            elif key_attack == 2:
                applySat_x = centroids["ball"][0] + delta_x > centroids['goal1'][0] - 30
                applySat_y = centroids["ball"][1] + delta_y > img.shape[0] - robot_radius * 2 or centroids["ball"][
                                                                                                     1] + delta_y < robot_radius * 2
                goalx = centroids["ball"][0] + delta_x if not applySat_x else centroids['goal1'][0] - 30
                if centroids["ball"][1] + delta_y > img.shape[0] - robot_radius * 2:
                    goaly = img.shape[0] - robot_radius * 2
                elif centroids["ball"][1] + delta_y < robot_radius * 2:
                    goaly = robot_radius * 2
                else:
                    goaly = centroids["ball"][1] + delta_y
                goal = (goalx, goaly)
            # State 3 dribbles (if key == 3) or hits (if key == 4) the ball
            elif key_attack == 3:
                goal = centroids['ball']
                # how do I define each operation differently???
                if key == 3:
                    pass
                elif key == 4:
                    pass
        # Once the goal position has been decided, commence path planning. A (0, 0) goal position indicate a null value,
        # and all path planning must cease. Begin by calculating angle error to goal position. Default to no movement
        # if the goal is undefined.
        if goal == (0, 0):
            payload = motors_to_bytes(0, 0)
        else:
            e_ang = angle_error(centroids['self_big'], centroids['self_small'], goal)
            # If the GUARD state machine is currently guarding the goal, override the motors to only advance or reverse.
            # The robot radius is added as a deadband value to prevent oscillations in motor control signals
            if key_guard == 3:
                if goal[1] - robot_radius > centroids["self_robot"][1]:
                    payload = motors_to_bytes(-guard_vel, -guard_vel)
                elif goal[1] + robot_radius < centroids["self_robot"][1]:
                    payload = motors_to_bytes(guard_vel, guard_vel)
                else:
                    payload = motors_to_bytes(0, 0)
            # remember to define dribble_vel and strike_vel in robot variables
            elif key_attack == 3:
                if key == 3:
                    payload = motors_to_bytes(dribble_vel, dribble_vel)
                elif key == 4:
                    payload = motors_to_bytes(strike_vel, strike_vel)
            # MOTOR CONTROL
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
            # END CONDITIONS
            # In an ideal world, the robot would advance perfectly to the specified goal position.
            # However, these conditions allow the robot centroid to be within a certain distance and/or angle error
            # to the given goal position
            dist = distance_error(centroids['self_robot'], goal)
            # For operation 0, set the deadband at the robot radius plus 30 pixels
            if key == 0:
                if dist < robot_radius + ball_radius + 5:
                    payload = motors_to_bytes(0, 0)
            # For operation 1, set the deadband at the robot radius plus 30 pixels
            if key == 1:
                if dist < robot_radius + 20:
                    payload = motors_to_bytes(0, 0)
            if key == 2:
                if key_guard == 0:
                    if dist < 20:
                        key_guard += 1
                elif key_guard == 1:
                    if dist < 20:
                        key_guard += 1
                elif key_guard == 2:
                    if abs(e_ang) < eps_guard_ang:
                        key_guard += 1
                elif key_guard == 3:
                    # Calculate the angle error to a false goal located above the robot at a y_coord of 0. This allows
                    # the robot to realign itself without moving towards the upper goal. If this error exceeds a
                    # certain angle, the GUARD state machine is returned to state 2 to realign itself.
                    if goal[1] == 0:
                        err_guard_ang = angle_error(centroids['self_big'], centroids['self_small'],
                                                    (centroids['self_big'][0], goal[1]))
                        if abs(err_guard_ang) > eps_guard_ang:
                            key_guard -= 1
                    # If the goal is not located at the top of the map, the robot is not trying to align itself.
                    # Instead, it is guarding the goal. In this case, pass.
                    # TODO Observe behaviour when attempting to leave GUARD state machine
                    else:
                        pass
            # VARIABLE LOGGERS
            if goal_debug:
                debug_dict['goal'] = goal
                # print("Goal to: ", goal)
            if angle_debug:
                debug_dict['angle_err'] = degrees(e_ang)
                # print(degrees(e_ang))
            if state_G_debug:
                debug_dict['guard_state'] = key_guard
                # print(key_guard)
    # Stop motors if there are undefined goal points.
    else:
        payload = motors_to_bytes(0, 0)
    if payload_debug:
        debug_dict['payload'] = list(payload)
        # print(list(payload))
    if debug_HSV:
        prev_val = value
    # Send payload to robot
    if not no_fi:
        sock.send(payload)
    # Write video log
    if video_debug:
        out.write(img)
    # Show debug window
    cv2.imshow("Debug Information", write_debug_window(debug_dict))
    cv2.imshow("Centroid Information", write_debug_window(centroids))
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
        debug_dict['operation'] = key

# Once out of the processing loop, deactivate the motor outputs
if not no_fi:
    sock.send(motors_to_bytes(0, 0))
# Standard exit procedure. Release cam, video log, destroy windows and exit program
cam.release()
out.release()
cv2.destroyAllWindows()
exit()
