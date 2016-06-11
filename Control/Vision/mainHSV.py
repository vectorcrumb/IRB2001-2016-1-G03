import numpy as np
import cv2
import math

# Define hsv ranges
hsv_ball = [np.array([20, 50, 50], np.uint8), np.array([30, 255, 255], np.uint8)]
hsv_field = [np.array([65, 50, 50], np.uint8), np.array([75, 255, 255], np.uint8)]
hsv_f_s_robot = [np.array([145, 50, 50], np.uint8), np.array([150, 255, 255], np.uint8)]
hsv_b_s_robot = [np.array([175, 50, 50], np.uint8), np.array([180, 255, 255], np.uint8)]
hsv_f_o_robot = [np.array([90, 50, 50], np.uint8), np.array([110, 250, 250], np.uint8)]
hsv_b_o_robot = [np.array([120, 50, 50], np.uint8), np.array([140, 250, 250], np.uint8)]

# Obtain image
img = cv2.imread('Test5.jpg', cv2.IMREAD_COLOR)
# Obtain copy of original image
o_img = img.copy()
# Convert to HSV
hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

# Obtain distinct binary masks
ball = cv2.inRange(hsv_img, *hsv_ball)
field = cv2.inRange(hsv_img, *hsv_field)
f_s_robot = cv2.inRange(hsv_img, *hsv_f_s_robot)
b_s_robot = cv2.inRange(hsv_img, *hsv_b_s_robot)

# Define the moments of the binary images
mom_ball = cv2.moments(ball)
mom_field = cv2.moments(field)
mom_f_s_robot = cv2.moments(f_s_robot)
mom_b_s_robot = cv2.moments(b_s_robot)

# Calculate (x,y) coordinates of detected objects given the moments of mask
pos_ball = (int(mom_ball['m10'] / mom_ball['m00']), int(mom_ball['m01'] / mom_ball['m00']))
pos_field = (int(mom_field['m10'] / mom_field['m00']), int(mom_field['m01'] / mom_field['m00']))
pos_f_s_robot = (int(mom_f_s_robot['m10'] / mom_f_s_robot['m00']), int(mom_f_s_robot['m01'] / mom_f_s_robot['m00']))
pos_b_s_robot = (int(mom_b_s_robot['m10'] / mom_b_s_robot['m00']), int(mom_b_s_robot['m01'] / mom_b_s_robot['m00']))

# Average the marker positions of self bot to find its centroid
centroid_bot = tuple(int((front_coord + back_coord) / 2) for front_coord, back_coord in zip(pos_f_s_robot, pos_b_s_robot))

# Mark centroids of each discovered figure
img = cv2.circle(img, pos_ball, 3, (255, 255, 0), -1)
img = cv2.circle(img, pos_field, 3, (255, 255, 0), -1)
img = cv2.circle(img, pos_f_s_robot, 3, (255, 255, 0), -1)
img = cv2.circle(img, pos_b_s_robot, 3, (255, 255, 0), -1)
img = cv2.circle(img, centroid_bot, 3, (255, 255, 0), -1)

# Calculate theta to ball from robot
dif_s_robot_ball = (pos_ball[0] - centroid_bot[0], centroid_bot[1] - pos_ball[1])
theta_s_bot_ball = math.atan2(*reversed(dif_s_robot_ball))
img = cv2.line(img, centroid_bot, pos_ball, (0, 0, 255), 2)
print("THETA - Radians:", theta_s_bot_ball, ", Factor of Pi:", theta_s_bot_ball / math.pi,
      ", Degrees:", math.degrees(theta_s_bot_ball))

# Calculate phi of robot centrpid to reference frame
dif_s_robot_int = (pos_f_s_robot[0] - pos_b_s_robot[0], pos_b_s_robot[1] - pos_f_s_robot[1])
phi_s_bot_frame = math.atan2(*reversed(dif_s_robot_int))
print("PHI - Radians:", phi_s_bot_frame, ", Factor of Pi:", phi_s_bot_frame / math.pi,
      ", Degrees:", math.degrees(phi_s_bot_frame))


# Visualization code. DO NOT UNCOMMENT!
# # Draw arc for angle
# img = cv2.ellipse(img, center=centroid_bot, axes=tuple(int(dif_s_robot_ball[0]/3) for _ in range(2)),
#                   angle=0, startAngle=0, endAngle=-math.degrees(theta_s_bot_ball), color=(255, 0, 0,), thickness=-1)
# # Extend dashed line from robot centroid for visualization purposes
# # for dx in range(0, dif_s_robot_ball[0]//2, np.sign(dif_s_robot_ball[0])*10):
# for dx in range(0, np.sign(dif_s_robot_ball[0])*dif_s_robot_ball[0]//2, 10):
#     img = cv2.line(img, (centroid_bot[0]+2*dx, centroid_bot[1]),
#                         (centroid_bot[0]+2*dx+10, centroid_bot[1]), (0, 255, 0), 2)
# # Display angle within arc
# img = cv2.putText(img, text=str(int(math.degrees(theta_s_bot_ball))),
#                   org=(centroid_bot[0]+10, centroid_bot[1]-5),
#                   fontFace=cv2.FONT_HERSHEY_SIMPLEX, color=(0, 255, 255), fontScale=0.5)


# Display masks and image
cv2.imshow("Image", img)
# cv2.imshow("HSV Image", hsv_img)
# cv2.imshow("Ball", ball)
# cv2.imshow("Field", field)
# cv2.imshow("Front of Self Robot", f_s_robot)
# cv2.imshow("Back of Self Robot", b_s_robot)

cv2.waitKey(0)
cv2.destroyAllWindows()