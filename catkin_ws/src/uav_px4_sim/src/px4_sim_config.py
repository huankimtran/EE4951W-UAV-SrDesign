"""
    Author: Huan Tran
    Contact: huankimtran@gmail.com
    ----
"""
class PX4SimConfig:
    """
        Class holding configuration value
    """
    
    def __init__(self):
        self.TOPIC_CMD_NAME = 'px4_cmd'
        self.TOPIC_CURRENT_VIEW_PORT_NAME = 'px4_viewport'
        self.CAMERA_VIEWPORT_SIZE = (1000, 1000)      # (height, width)
        self.GROUND_IMG_LOCATION = '/home/ubuntu18/Desktop/Working/EE4951W-UAV/catkin_ws/src/uav_px4_sim/img/map12x.png'
        self.TARGET_IMG_LOCATION = '/home/ubuntu18/Desktop/Working/EE4951W-UAV/catkin_ws/src/uav_px4_sim/img/qr.png'
