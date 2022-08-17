#!/usr/bin/env python3

import cv_bridge
from geometry_msgs import msg
from geometry_msgs.msg import Twist, Pose
import rclpy
from std_msgs.msg import UInt32, Float64MultiArray, String
from sensor_msgs.msg import Image
from std_srvs.srv import Empty
from tello_msgs.srv import TelloAction
import cv2 
from cv_bridge import CvBridge
import imutils
from std_msgs.msg import Float64
import math
import numpy as np
from apriltag_msgs.msg import AprilTagDetectionArray
import tf_transformations 

rclpy.init()
node_handle = rclpy.create_node('img_proc')
pub_hsv = node_handle.create_publisher(Image, '/processImg', 1)
pub_pnp = node_handle.create_publisher(Pose, '/pnpPose', 1)

global gate_pos
corners = []


async def calc_angle(origin, p1, p2):
  a = np.array([origin[0],origin[1]])
  b = np.array([p1[0],p1[1]])
  c = np.array([p2[0],p2[1]])
  vec1 = np.subtract(a,b)
  vec2 = np.subtract (a,c) 
  abs_a = np.linalg.norm(vec1)
  abs_b = np.linalg.norm(vec2)
  dotprod = np.dot(vec1,vec2)
  phi = np.rad2deg(np.arccos(dotprod/(abs_a*abs_b)))
  return phi

async def direction(p1, p2):
      diff_x = abs(p1[0]-p2[0])
      diff_y = abs(p1[1]-p2[1])
      return [diff_x,diff_y]


async def calc_center(extLeft, extRight, extBot, extTop ,dist_LB,dist_LR,dist_RB, gate_rot, frame):
  if gate_rot == "Center":
    center = (extLeft[0]+extRight[0])/2, (extLeft[1]+extRight[1])/2
  elif gate_rot == "Right":
    center = (extBot[0]+extRight[0])/2, (extBot[1]+extRight[1])/2
    #print("Using these points:",extBot, extRight)
  elif gate_rot == "Left":
    center = (extBot[0]+extLeft[0])/2, (extBot[1]+extLeft[1])/2
  cv2.circle(frame, (int(center[0]),int(center[1])), 8, (0, 255, 0), -1)
  #print("Calucated Center: ", center)
  return center


async def gate_position(d_LR,d_TL,d_TR,d_LB,d_RB, extLeft, extRight, extBot, extTop, frame):

  if d_LR < 200 or d_TL < 150 or d_TR < 100 or calc_angle(extLeft, extTop, extRight)<36 or calc_angle(extRight, extTop, extLeft)<36 or calc_angle(extRight, extTop, extLeft)>50 or calc_angle(extLeft, extTop, extRight)>50:
    #print("Left y: ", extLeft[1])
    #print("Right y: ", extRight[1])
    #print("angle: ", calc_angle(extLeft, extTop, extRight))
    #print("angle: ", calc_angle(extRight, extTop, extLeft))
    gate_detected = False
    cv2.line(frame, extLeft, extRight, (0,0,255), 2)
    cv2.line(frame, extLeft, extTop, (0,0,255), 2)
    cv2.line(frame, extRight, extTop, (0,0,255), 2)
    #print("No Gate Detected")
  else:
    gate_detected = True

  if gate_detected == True:
    cv2.line(frame, extLeft, extRight, (255,255,255), 2)
    cv2.line(frame, extLeft, extTop, (255,255,255), 2)
    cv2.line(frame, extRight, extTop, (255,255,255), 2)
    #print("Gate Detected")
    #print("angle: ", calc_angle(extLeft, extTop, extRight))
    #print("angle: ", calc_angle(extRight, extTop, extLeft))
    if d_LB<d_RB:
      gate_pos = "Right"
      cv2.line(frame, extBot, extRight, (174,2,244), 2)
      cv2.line(frame, extBot, extLeft, (174,2,244), 2)
      cv2.line(frame, extRight, extLeft, (174,2,244), 2)
    elif d_RB<d_LB:
      gate_pos = "Left"
      cv2.line(frame, extBot, extRight, (2,122,242), 2)
      cv2.line(frame, extBot, extLeft, (2,122,242), 2)
      cv2.line(frame, extRight, extLeft, (2,122,242), 2)
    elif math.isclose(d_RB, 0,abs_tol=5) or math.isclose(d_LB,0,abs_tol=5):
      gate_pos = "Center"
      cv2.line(frame, extLeft, extRight, (0,255,0), 2)
      cv2.line(frame, extLeft, extTop, (0,255,0), 2)
      cv2.line(frame, extRight, extTop, (0,255,0), 2)
  else:
    gate_pos = "No Gate"
  #print(gate_pos)
  gate_status = [gate_pos, gate_detected]
  return gate_status


async def face_top_pose_est(extLeft, extRight, extBot, extTop, center, proc_frame, gate_rot, dist_LB, dist_RB, frame):

  # Setting up Image Points depending on Rotation of Gate to Camera

    if gate_rot == "Center" and (dist_RB<10 or dist_LB<10):
        #print("Setting Center Image Points")
        image_points = np.array([
                              [extLeft[0],extLeft[1]],     # Left Extreme of Gate
                              [extRight[0],extRight[1]],    # Right Extreme of Gate
                              [center[0],center[1]],                   # Middle between left and right
                              [extTop[0],extTop[1]],       # Top Extreme of  Gate
                          ], dtype="double")

    elif gate_rot == "Left":
        image_points = np.array([
                        [extLeft[0],extLeft[1]],     # Left Extreme of Gate
                        [extBot[0],extBot[1]],     # Bottom Extreme of Gate (Right Corner)
                        #[extRight[0],extRight[1]],    # Right Extreme of Gate
                        [extTop[0],extTop[1]],       # Top Extreme of  Gate
                        [center[0],center[1]],
                    ], dtype="double")

    elif gate_rot == "Right": 
      image_points = np.array([
                      #[extLeft[0],extLeft[1]],     # Left Extreme of Gate
                      [extBot[0],extBot[1]],     # Bottom Extreme of Gate (Left Corner)
                      [extRight[0],extRight[1]],    # Right Extreme of Gate
                      [extTop[0],extTop[1]],       # Top Extreme of  Gate
                      [center[0],center[1]],
                  ], dtype="double")
    #print("Image Points: ", image_points)


  #Setting up Model Points depending on Rotation of Gate to Camera

    if gate_rot == "Center" and (dist_RB<10 or dist_LB<10):
        #print("Setting Center Model Points")
        model_points = np.array([    #x,y,z
                                [-0.735, -0.65, 0],     # Front Left Corner
                                [0.735, -0.65, 0],      # Front Right Corner 
                                [0, -0.65, 0],      # Front Center
                                [0, 0, 0],    # Front Top
                            ], dtype="double")

    elif gate_rot == "Left":
        model_points = np.array([    #x,y,z
                                [-0.735, -0.65, 0],     # Front Left Corner
                                [0.735, -0.65, 0],      # Front Right Corner 
                                [0, 0, 0],    # Front Top
                                [0, -0.65, 0],      # Front Center
                            ], dtype="double")

    elif gate_rot == "Right":
        model_points = np.array([    #x,y,z
                                [-0.735, -0.65, 0],     # Front Left Corner
                                [0.735, -0.65, 0],      # Front Right Corner 
                                [0, 0, 0],    # Front Top
                                [0, -0.65, 0],      # Front Center
                            ], dtype="double")


  # Camera internals

    #Ben 
    # camera_matrix = np.array(
    #                       [[921.170702, 0.000000, 459.904354], [0.000000, 919.018377, 351.238301], [0.000000, 0.000000, 1.000000]]
    #                       )
    #Tom
    camera_matrix = np.array(
                            [[917.301627, 0.000000, 485.224763 ],[0.000000, 916.345539, 357.199463],[ 0.000000, 0.000000, 1.000000]]
    )
    #print ("Camera Matrix :\n {0}".format(camera_matrix))

    #dist_coeffs = np.array([-0.033458, 0.105152, 0.001256, -0.006647, 0.000000])
    #dist_coeffs = np.array([-0.019450, 0.060040, -0.001809, 0.004933, 0.000000])  cam Ben
    dist_coeffs =np.array([-0.049578, 0.072457, 0.001663, -0.003238, 0.000000])
    
    

    (success, rotation_vector, translation_vector) = cv2.solvePnP(model_points, image_points, camera_matrix, dist_coeffs, flags=cv2.SOLVEPNP_P3P)
    #(success, rotation_vector, translation_vector) = cv2.solvePnP(model_points, image_points, camera_matrix, dist_coeffs, flags=cv2.SOLVEPNP_EPNP)
    #(success, rotation_vector, translation_vector) = cv2.solvePnP(model_points, image_points, camera_matrix, dist_coeffs, flags=cv2.SOLVEPNP_ITERATIVE)
    #(success, rotation_vector, translation_vector) = cv2.solvePnPRansac(model_points,image_points, camera_matrix, dist_coeffs, flags=cv2.SOLVEPNP_P3P)
    #(success, rotation_vector, translation_vector) = cv2.solvePnP(rns_model_points, rns_image_points, camera_matrix, dist_coeffs, flags=cv2.SOLVEPNP_P3P)
    #(success, rotation_vector, translation_vector) = cv2.solvePnP(model_points, image_points, camera_matrix, dist_coeffs)

    #print ("Rotation Vector:\n {0}".format(rotation_vector))
    #print ("Translation Vector:\n {0}".format(translation_vector))
    #print(rotation_vector)
    #Sprint(translation_vector)

    # Project a 3D point (0, 0, 1000.0) onto the image plane.
    # We use this to draw a line sticking out of the nose

    if (rotation_vector is not None) or (translation_vector is not None):

      cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rotation_vector, translation_vector, 1,4)
      pnp_data = Pose()
      pnp_data.position.x = translation_vector[0][0]
      pnp_data.position.y = translation_vector[1][0]
      pnp_data.position.z = translation_vector[2][0]

      q = tf_transformations.quaternion_from_euler(rotation_vector[0][0],rotation_vector[1][0],rotation_vector[2][0])

      pnp_data.orientation.x = q[1]
      pnp_data.orientation.y = q[2]
      pnp_data.orientation.z =  q[3]
      pnp_data.orientation.w = q[0]

      #print('this is pnp')
      #print(pnp_data)
      pub_pnp.publish(pnp_data)
    
    else: 
      pub_pnp.publish(Pose())



def aprilTag_corr(msg):
  global corners
  try:
    data = msg.detections[0].corners
    corners = [int(data[0].x), int(data[0].y), int(data[1].x), int(data[1].y), int(data[2].x), int(data[2].y), int(data[3].x), int(data[3].y)]
    return corners
  except:
    corners = [0,0,0, 0,0,0,0,0]
    return

      
def main():

    sub_Tag_det = node_handle.create_subscription(AprilTagDetectionArray, '/apriltag/detections', aprilTag_corr , 10)
    node_handle.create_subscription(Image, 'image_raw', mysubcallback, 10)

    try:
        rclpy.spin(node_handle)
    except KeyboardInterrupt:
        pass
    node_handle.destroy_node()
    rclpy.shutdown()

def mysubcallback(msg):
    global corners
    #low1_H = 77
    low1_H = 160
    low1_S = 180
    low1_V = 100
    high1_H = 180
    high1_S = 255
    high1_V = 255

    low2_H = 0
    low2_S = 254
    low2_V = 134
    high2_H = 20
    high2_S = 255
    high2_V = 255
    cv_bridge = CvBridge()
    frame = cv_bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

    def get_perceive_brightness(img):
      float_img = np.float64(img)  # unit8 will make overflow
      b, g, r = cv2.split(float_img)
      float_brightness = np.sqrt(
          (0.241 * (r ** 2)) + (0.691 * (g ** 2)) + (0.068 * (b ** 2)))
      brightness_channel = np.uint8(np.absolute(float_brightness))
      return brightness_channel

    perceived_brightness_channel = get_perceive_brightness(frame)

    clahe = cv2.createCLAHE(clipLimit=0.5, tileGridSize=(4,4))
    clahe_applied_perceived_channel = clahe.apply(perceived_brightness_channel) 

    def hsv_equalizer(img, new_channel):
      hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
      h,s,v =  cv2.split(hsv)
      merged_hsv = cv2.merge((h, s, new_channel))
      bgr_img = cv2.cvtColor(merged_hsv, cv2.COLOR_HSV2BGR)
      return bgr_img

    def lab_equalizer(img, new_channel):
      lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
      l,a,b =  cv2.split(lab)
      merged_lab = cv2.merge((new_channel,a,b))
      bgr_img = cv2.cvtColor(merged_lab, cv2.COLOR_LAB2BGR)
      return bgr_img

    hsv_equalized_img = hsv_equalizer(frame,clahe_applied_perceived_channel)
    #lab_equalized_img = lab_equalizer(frame,clahe_applied_perceived_channel)


    blurred = cv2.GaussianBlur(frame, (7, 7), 0)
    #blurred = cv2.GaussianBlur(lab_equalized_img, (7, 7), 0)
    frame_HSV = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    mask1 = cv2.inRange(frame_HSV, (low1_H, low1_S, low1_V), (high1_H, high1_S, high1_V))
    mask2 = cv2.inRange(frame_HSV, (low2_H, low2_S, low2_V), (high2_H, high2_S, high2_V))
    comb_mask = mask1 + mask2


    #APRIL TAG CORRECTION HERE
    #print(corners)
    if  corners:
      #print("dis shit")
      #print(corners)
      cv2.rectangle(comb_mask, (corners[0],corners[1]),(corners[4],corners[5]), (255,255,255), -1)



    # Value is okay but maybe a bit too big
    se3 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (51,51))
    se4 = cv2.getStructuringElement(cv2.MORPH_RECT, (40,40))
    se1 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (41,41))
    se2 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (39,39))


    #open first
    #open = cv2.morphologyEx(mask1, cv2.MORPH_OPEN, se1)
    #open_first = cv2.morphologyEx(open, cv2.MORPH_CLOSE, se2)

    #close first
    #close = cv2.morphologyEx(mask1, cv2.MORPH_CLOSE, se3)
    #close_first = cv2.morphologyEx(close, cv2.MORPH_OPEN, se4)

    #erode
    #ero = cv2.morphologyEx(mask1, cv2.MORPH_OPEN, se1)

    #dialate
    dia = cv2.morphologyEx(comb_mask, cv2.MORPH_CLOSE, se3)
    #TEST
    
    #edged = cv2.Canny(open_first, 50, 150)




    cnts = cv2.findContours(dia, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    if len(cnts) != 0:
      c = max(cnts, key=cv2.contourArea)
      extLeft = tuple(c[c[:, :, 0].argmin()][0])
      extRight = tuple(c[c[:, :, 0].argmax()][0])
      extTop = tuple(c[c[:, :, 1].argmin()][0])
      extBot = tuple(c[c[:, :, 1].argmax()][0])
      cv2.drawContours(frame, [c], -1, (0, 255, 255), 2)
    # POINTS OF GATE ARE FROM POV OF DRONE
      cv2.circle(frame, extLeft, 8, (0, 0, 255), -1)
      cv2.circle(frame, extRight, 8, (0, 255, 0), -1)
      cv2.circle(frame, extTop, 8, (255, 0, 0), -1)
      cv2.circle(frame, extBot, 8, (255, 255, 0), -1)



      dist_LR = np.linalg.norm([abs(extLeft[0]-extRight[0]),abs(extLeft[1]-extRight[1])])
      dist_TL = np.linalg.norm([abs(extTop[0]-extLeft[0]),abs(extTop[1]-extLeft[1])])
      dist_TR = np.linalg.norm([abs(extTop[0]-extRight[0]),abs(extTop[1]-extRight[1])])
      dist_LB = np.linalg.norm([abs(extBot[0]-extLeft[0]),abs(extBot[1]-extLeft[1])])
      dist_RB = np.linalg.norm([abs(extBot[0]-extRight[0]),abs(extBot[1]-extRight[1])])

      

      gate_status = gate_position(dist_LR, dist_TL, dist_TR,dist_LB, dist_RB ,extLeft, extRight, extBot, extTop, frame)

      if gate_status[1] == True:
        center = calc_center(extLeft, extRight, extBot, extTop,dist_LB,dist_LR,dist_RB, gate_status[0], frame)
        #pose_est(extLeft,extRight,extBot,extTop,center,dia,ergebnisliste[0], dist_LB, dist_RB)
        face_top_pose_est(extLeft,extRight,extBot,extTop,center,dia,gate_status[0], dist_LB, dist_RB, frame)
        #face_bottom_pose_est(extLeft,extRight,extBot,extTop,center,dia,gate_status[0], dist_LB, dist_RB, frame)
        #gate_pose(extLeft,extRight,extBot,extTop,center,dia,ergebnisliste[0], dist_LB, dist_RB)
      else: 
        pub_pnp.publish(Pose())





    # When contours overlap the distances change rapidly, maybe way to tell when overlapping?

    # Make some type of publisher for control values here



    #hsv_theshold = cv.bitwise_and(blur, blur, mask=hsv_mask)
    #ros_frame = cv_bridge.cv2_to_imgmsg(dia)
    ros_frame = cv_bridge.cv2_to_imgmsg(frame)
    ros_frame.header.frame_id = msg.header.frame_id
    
    pub_hsv.publish(ros_frame)

    




if __name__ == '__main__':
    main()  
































  




  