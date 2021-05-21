"""
    Author: Huan Tran
    Contact: huankimtran@gmail.com
    ----
"""
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
from mavros_msgs.srv import CommandBool
from commander import Commander
from geometry_msgs.msg import PoseStamped
from qr_code import *
import cv2
import time
import math

LANDING_THRESHOLD = 0.3
cvbridge = CvBridge()
com = None
qr = QRdetect(None)
arm_serv = None
last_goal = None
land_seq = None
landed = False
current_pose = None

def compute_distance(a,b):
    return math.sqrt(math.pow(a[0]-b[0],2) + math.pow(a[1]-b[1],2))

def get_ts_ms():
    return round(time.time() * 1000)

class LandingSequnce:
    def __init__(self):
        """
            3 stages, different radius
            different wait time
        """
        self.BLIND_HEIGHT = 1.5
        self.LANDING_TOLORANCE = 0.5
        self.RESEND_DURATION = 15000
        self.MAX_STAGE = 2
        self.stage = 0
        self.radius = [200, 300, 400]
        self.wait = [10000, 15000, 9000]
        self.last_time = None
        self.acc_time = 0
    
    def get_radius(self):
        return self.radius[self.stage]
    
    def tick(self, dist):
        global last_goal
        global current_pose
        if dist <= self.radius[self.stage] or (current_pose[2] <= self.BLIND_HEIGHT and compute_distance(current_pose[:-1], last_goal[:-1]) <= self.LANDING_TOLORANCE):
            if self.last_time:
                last = self.last_time
                self.last_time = get_ts_ms()
                self.acc_time += self.last_time - last
            else:
                self.last_time = get_ts_ms()
        else:
            if self.last_time != None:
                last = self.last_time
                self.last_time = get_ts_ms()
                self.acc_time = max(-self.RESEND_DURATION, (self.acc_time - (self.last_time - last)) if self.acc_time != 0 else -1)
        # print(self.acc_time)
        if self.acc_time >= self.wait[self.stage]:
            # Pass the stage, next stage
            self.stage = min(self.MAX_STAGE, self.stage + 1)
            # Reset
            self.acc_time = 0
            self.last_time = None
            # Send new goal to land
            if not last_goal:
                # If no goal sent, but target detected, then use current pos as goal
                if current_pose:
                    last_goal = current_pose
                else:
                    return
            last_goal[2] /= 2
            last_goal[2] = 0 if last_goal[2] <= 0.5 else last_goal[2]
            com.move(last_goal[0], last_goal[1], last_goal[2], False)
            print('Next landing stage, to ' + str(last_goal))
        elif self.acc_time <= -self.RESEND_DURATION:
            # Reset
            self.acc_time = 0
            # And resend
            if not last_goal:
                # If no goal sent, but target detected, then use current pos as goal
                if current_pose:
                    last_goal = current_pose
                else:
                    return
            com.move(last_goal[0], last_goal[1], last_goal[2], False)
            print('Resending target ' + str(last_goal))

def image_callback(image):
    global cvbridge
    global qr
    global land_seq
    global landed
    cv_image = None

    try:
        cv_image = cvbridge.imgmsg_to_cv2(image, "bgr8")
        # process
        train_qrcode_position = qr.QRcode_Position(cv_image)
    except CvBridgeError as e:
        print(e)
        return

    if train_qrcode_position is None:
        display = cv_image
        if landed:
            land_seq = None
        elif land_seq:
            # Tick say that this is out of range
            land_seq.tick(1000000000)
    else:
        marked = qr.drawPoints(cv_image.copy(), train_qrcode_position)
        # Mark qr center
        qr_center = tuple((np.sum(train_qrcode_position, axis=0) / 4).astype(int))
        cv2.circle(marked, qr_center, 11, (0,0,255), -1)
        # Mark center viewport
        vp_center = (int(marked.shape[1]/2), int(marked.shape[0]/2))
        # Create landing sequence if not yet created
        if landed:
            land_seq = None
        else:
            if not land_seq:
                land_seq = LandingSequnce()
                print('Landing sequence initiated!')
            # Show trigger radius
            cv2.circle(marked, vp_center, land_seq.get_radius(), (255, 0 , 0), thickness=1, lineType=8, shift=0)
            # Show distance from center of the qr code to center of viewport 
            cv2.line(marked, vp_center, qr_center, (0,255,0), 3)
            # Find distance from centor of the qr code to center of viewport
            dist = compute_distance(vp_center, qr_center)
            # Update landing stage
            land_seq.tick(dist)
        # Decide
        if marked is not None:
            display = marked
        else:
            display = cv_image
    # Display
    cv2.imshow('Camera view', display)
    cv2.waitKey(1)

def goal_handler(data):
    global last_goal
    if com:
        print('Receive goal at : ' + data.data)
        s = data.data.split(',')
        x, y, z = float(s[0]) , float(s[1]), float(s[2])
        last_goal = [float(x), float(y), float(z)]
        com.move(x, y, z, False)
        
def land_detector_handler(msg):
    global current_pose
    global landed
    if not current_pose:
        current_pose = [0,0,0]
    # Check z position, if too load, than disarm
    current_pose[0] = -msg.pose.position.x
    current_pose[1] = msg.pose.position.y
    current_pose[2] = z = msg.pose.position.z
    # Threshold
    if land_seq and z <= LANDING_THRESHOLD:
        # Disarm the drone
        arm_serv(False)
        print("Landed!")
        landed = True

if __name__ == '__main__':
    com = Commander()
    rospy.wait_for_service('/mavros/cmd/arming')
    arm_serv = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
    land_detector = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, land_detector_handler)
    image_sub = rospy.Subscriber("/gi/simulation/left/image_raw", Image, image_callback)
    goal_sub = rospy.Subscriber("/uav/goal", String, goal_handler)
    print('Ready!')
    rospy.spin()
