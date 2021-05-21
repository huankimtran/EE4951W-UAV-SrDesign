#!/usr/bin/env python
"""
    Author: Huan Tran
    Contact: huankimtran@gmail.com
    ----
"""
import rospy
from std_msgs.msg import String
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import random
from px4_sim_config import PX4SimConfig

# cfg = None
camera = None
config = None

def overlay_image_alpha(img, img_overlay, x, y):
    """Overlay `img_overlay` onto `img` at (x, y) and blend using `alpha_mask`.

    `alpha_mask` must have same HxW as `img_overlay` and values in range [0, 1].
    """
    img = cv2.cvtColor(img, cv2.COLOR_RGBA2RGB)
    alpha_mask = img_overlay[:, :, 3] / 255.0
    img_overlay = cv2.cvtColor(img_overlay, cv2.COLOR_RGBA2RGB)

    # Image ranges
    y1, y2 = max(0, y), min(img.shape[0], y + img_overlay.shape[0])
    x1, x2 = max(0, x), min(img.shape[1], x + img_overlay.shape[1])

    # Overlay ranges
    y1o, y2o = max(0, -y), min(img_overlay.shape[0], img.shape[0] - y)
    x1o, x2o = max(0, -x), min(img_overlay.shape[1], img.shape[1] - x)

    # Exit if nothing to do
    if y1 >= y2 or x1 >= x2 or y1o >= y2o or x1o >= x2o:
        return

    # Blend overlay within the determined ranges
    img_crop = img[y1:y2, x1:x2]
    img_overlay_crop = img_overlay[y1o:y2o, x1o:x2o]
    alpha = alpha_mask[y1o:y2o, x1o:x2o, np.newaxis]
    alpha_inv = 1.0 - alpha

    img_crop[:] = alpha * img_overlay_crop + alpha_inv * img_crop

    return img

"""
    Coordinate system will look like
    O-------> X
    |
    |         Backward
    |            ^
    |            |
    |    Left <-Drone->Right
    |            |
    |            v
    |         Forward
    v Y
"""
class CameraViewport:
    def __init__(self, viewport_width, viewport_height, initial_tl_corner_x, initial_tl_corner_y, parent_width, parent_height):
        self.shape = (viewport_height, viewport_width)
        self.width = viewport_width
        self.height = viewport_height
        self.parent_width = parent_width
        self.parent_height = parent_height
        self.x = initial_tl_corner_x        # This is not the x of the center, this is the top left x
        self.y = initial_tl_corner_y        # This is not the y of the center, this is the top left y
    
    def backward(self, step):
        step = int(step)
        if self.y - step >= 0:
            self.y -= step
        else:
            self.y = 0
    
    def forward(self, step):
        step = int(step)
        if self.y + step < self.parent_height - self.height:
            self.y += step
        else:
            self.y = self.parent_height -self.height - 1

    def left(self, step):
        step = int(step)
        if self.x - step >= 0:
            self.x -= step
        else:
            self.x = 0

    def right(self, step):
        step = int(step)
        if self.x + step < self.parent_width - self.width:
            self.x += step
        else:
            self.x = self.parent_width - self.width - 1
    
    def move(self, direction):
        """
            direction is a tuple or list [x_move, y_move]
            x_move > 0 then go right
            y_move > 0 then move forward
        """
        x_move = direction[0]
        y_move = direction[1]
        
        if x_move > 0:
            self.right(x_move)
        else:
            self.left(-x_move)

        if y_move > 0:
            self.forward(y_move)
        else:
            self.backward(-y_move)

def cmd_topic_listener(data):
    """
        5 commands:
        forward amount
        backward amount
        left amount
        right amount
        move x_amount y_amount
    """
    global camera
    cmd = str(data.data)
    print('Received command: {}'.format(cmd))
    # Separate cmd name and argument
    cmd_args = cmd.split()
    # print(cmd_args)
    cmd = cmd_args[0]
    cmd_args = [int(x) for x in cmd_args[1:]]
    cmd_ex = True

    #  Execute the correct command
    if camera is not None:
        if cmd == 'forward':
            camera.viewport.forward(cmd_args[0])
        elif cmd == 'backdward':
            camera.viewport.backward(cmd_args[0])
        elif cmd == 'left':
            camera.viewport.left(cmd_args[0])
        elif cmd == 'right':
            camera.viewport.right(cmd_args[0])
        elif cmd == 'move':
            camera.viewport.move((cmd_args[0], cmd_args[1]))
        else:
            #  Could not understand what command was sent
            cmd_ex = False
    else:
        cmd_ex = False
    # Decide if we should publish new view
    if cmd_ex:
        camera.publish_viewport()

class UAVCamera:
    """
        Representing the UAV camera
        Coordinate system will look like

        O-------> X
        |
        |         Forward
        |            ^
        |            |
        |    Left <-Drone->Right
        |            |
        |            v
        |         Backward
        v Y
    """
    def __init__(self, cfg):

        # Publisher for publishing viewport image
        self.viewport_pub = rospy.Publisher(
            name=cfg.TOPIC_CURRENT_VIEW_PORT_NAME,
            data_class=Image,
            queue_size=10
        )

        self.cv2ros_img_bridge = CvBridge()

        # Subcriver for getting command from the topic
        self.cmd_sub = rospy.Subscriber(
            name=cfg.TOPIC_CMD_NAME,
            data_class=String,
            callback=cmd_topic_listener
        )

        # Loading target and ground image
        self.ground_img_raw = cv2.imread(cfg.GROUND_IMG_LOCATION, cv2.IMREAD_UNCHANGED)
        self.target_img_raw = cv2.imread(cfg.TARGET_IMG_LOCATION, cv2.IMREAD_UNCHANGED)
        # Ground size
        self.ground_width = self.ground_img_raw.shape[1]
        self.ground_height = self.ground_img_raw.shape[0]
        # Target size
        self.target_width = self.target_img_raw.shape[1]
        self.target_height = self.target_img_raw.shape[0]
        
        # Pick a random location for the target
        self.target_x = random.randint(self.target_width-1, self.ground_width - self.target_width - 1)
        self.target_y = random.randint(self.target_height-1, self.ground_height - self.target_height - 1)

        # Overlay the target onto the ground img to get the final map
        self.map_img = overlay_image_alpha(
            img=self.ground_img_raw,
            img_overlay=self.target_img_raw,
            x=self.target_x,
            y=self.target_y,
        )

        # Internal viewport camera setting
        # Pick the original location randomly
        self.viewport = CameraViewport(
            viewport_width=cfg.CAMERA_VIEWPORT_SIZE[1],
            viewport_height=cfg.CAMERA_VIEWPORT_SIZE[0],
            parent_width=self.ground_width,
            parent_height=self.ground_height,
            initial_tl_corner_x=random.randint(cfg.CAMERA_VIEWPORT_SIZE[1]-1, self.ground_width - cfg.CAMERA_VIEWPORT_SIZE[1] - 1),
            initial_tl_corner_y=random.randint(cfg.CAMERA_VIEWPORT_SIZE[0]-1, self.ground_height - cfg.CAMERA_VIEWPORT_SIZE[0] - 1),
        )
        
        # Publish first view
        self.publish_viewport()

    def get_viewport_sight(self):
        return self.map_img[self.viewport.y : self.viewport.y + self.viewport.height,
                            self.viewport.x : self.viewport.x + self.viewport.width]
    
    def publish_viewport(self):
        # print('About to publish')
        ros_img = self.cv2ros_img_bridge.cv2_to_imgmsg(
            cvim=self.get_viewport_sight(),
            encoding='passthrough'
        )
        self.viewport_pub.publish(ros_img)
        # print('published')

def px4_ros_node():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    global camera
    rospy.init_node('ps4_terminal', anonymous=False)
    camera = UAVCamera(PX4SimConfig())
    # Keep the thread running
    rospy.spin()

if __name__ == '__main__':
    px4_ros_node()

    