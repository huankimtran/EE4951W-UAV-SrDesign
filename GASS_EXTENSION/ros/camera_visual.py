"""
    Author: Huan Tran
    Contact: huankimtran@gmail.com
    ----
"""
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import rospy
import cv2

cvbridge = CvBridge()


def image_callback(image):
    global cvbridge
    cv_image = None

    try:
        cv_image = cvbridge.imgmsg_to_cv2(image, "bgr8")

    except CvBridgeError as e:
        print(e)
    # Show all images
    cv2.imshow('left_view', cv_image)
    cv2.waitKey(1)


if __name__ == '__main__':
    rospy.init_node('CameraVisual')
    image_sub = rospy.Subscriber("/gi/simulation/left/image_raw", Image, image_callback)
    rospy.spin()
