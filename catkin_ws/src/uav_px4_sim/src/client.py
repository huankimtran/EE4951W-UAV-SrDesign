#!/usr/bin/env python
"""
    Author: Huan Tran
    Contact: huankimtran@gmail.com
    ----
    This script needs python contrib for python 2.7
    run "pip2 install opencv-contrib-python" if you encouter error about the QRCode module
    The QR code used in this example has the text "You found it!"
    This example currently randomly moves the viewport until it luckly spots the QR code in its view port
    Uncomment line 59 to make the process slower
"""
import rospy
from std_msgs.msg import String
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import random
from px4_sim_config import PX4SimConfig
import numpy as np
import time
import sys

cfg = PX4SimConfig()
qrCodeDetector = cv2.QRCodeDetector()
viewport_img = None
px4_cmd_pub = None

def on_new_img_arrive(data):
    global viewport_img
    bridge = CvBridge()
    try:
        # Save the newly published img so that the while loop in clien_node can display it
        # Also convert from the ROS Image type to OpenCV image type
        viewport_img = bridge.imgmsg_to_cv2(data)
        # Read and process image to get a vector to move
        move_vector = inspect_view(viewport_img)
        # Send command to move the viewport accordingly
        if move_vector is not None:
            px4_cmd_pub.publish(String('move {} {}'.format(move_vector[0], move_vector[1])))
    except CvBridgeError as e:
        print(e)

def inspect_view(view):
    """
        Function takes in an image describing the viewport
        and return a move vector to move the viewport to find the qr code
        the return vector has the form (move_x, move_y)
    """
    if is_qr(view):
        # Found Qr code then stop moving
        print('QR found!')
        return None
    else:
        # Calculate move vector
        # TODO: Design a better way to control the viewport
        #  Right now, don't know how to do, so just delay 1s and new random move_vector
        #  Uncomment this when you see the search process is too fast

        # time.sleep(1)
                
        return (random.randint(-1000, 1000), random.randint(-1000, 1000))

def is_qr(img):
    try:
        decodedText, points, _ = qrCodeDetector.detectAndDecode(img)
        if points is not None:
            # The QR code is the representation of the text "You found it!"
            # So let check if we found it
            if decodedText == 'You found it!':
                nrOfPoints = len(points)
                for i in range(nrOfPoints):
                    nextPointIndex = (i+1) % nrOfPoints
                    cv2.line(img, tuple(points[i][0]), tuple(points[nextPointIndex][0]), (255,0,0), 5)
                return True
            else:
                return False
    except Exception as e:
        return False

def client_node():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('client', anonymous=False)

    # A subscriber that capture new images from the viewport everytime the viewport change its position
    viewport_scene_sub = rospy.Subscriber(
        name=cfg.TOPIC_CURRENT_VIEW_PORT_NAME,
        data_class=Image,
        callback=on_new_img_arrive)
    
    # Publisher that can be used to send command to the PX4
    global px4_cmd_pub
    px4_cmd_pub = rospy.Publisher(
        name=cfg.TOPIC_CMD_NAME,
        data_class=String,
        queue_size=10
    )

    time.sleep(1)
    # Prime the subscrber handler
    px4_cmd_pub.publish(String('move 0 0'))

    while not rospy.is_shutdown():
        # Keep showing the new image if there is any changes
        if viewport_img is not None:
            cv2.imshow('Viewport scene', viewport_img)
            cv2.waitKey(1)

if __name__ == '__main__':
    client_node()

    