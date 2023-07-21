from time import time, sleep
import rospy
from typing import List, Tuple
from collections import deque
from flexbe_core import EventState, Logger
from sonia_common.msg import FilterchainTarget, AddPose, MpcInfo, CenterShapeBoundingBox, BoundingBox2D

class vision_alignemnt_check(EventState):
    """
    Get info from filterchains.
        -- filterchain_obj_topic    str     The topic from the filterchain to subscribe to get the object.
        -- filterchain_box_topic    str     The topic from the filterchain to subscribe to get the bounding box.
        -- nb_imgs                  int     Number of images needed to calculate average point.
        -- timeout_sec              int     Time to wait for images from filterchain.
        -- max_adjusts              int     Maximum amount of times to adjust to align target.
        -- tolerance                float   Percentage tolerance for the size of the object.

        ># x_func                           Function parameters when moving in x.

        <= timeout                          If timeout is reached
        <= success                          If target is found and aligned
        <= failed                           If max adjusts reached without success
    """


    def __init__(self, filterchain_obj_topic, filterchain_box_topic, blob_size, nb_imgs = 10, timeout_sec = 5, max_adjusts = 10, tolerance = 0.05):
        super(vision_alignemnt_check, self).__init__(outcomes = ['timeout', 'success', 'failed'],
                                                     input_keys = ['x_func', 'yz_function'])
        self.__obj_topic = filterchain_obj_topic
        self.__box_topic = filterchain_box_topic
        self.__max_adjusts = max_adjusts
        self.__blob_size = blob_size
        self.__tolerence_thresh = tolerance
        self.__adjusts = 0
        self.__timeout = timeout_sec
        self.__filterchain_obj_sub = None
        self.__move_pub = None
        self.__get_controller_info_sub = None
        self.__img_buffer = deque(maxlen=nb_imgs)
        self.__wait_for_target = False
        self.__target_reached = False
        self.__trajectory_done = False
        self.__bounding_box = None
        self.__ax = 0
        self.__bx = 0
        self.__cx = 0
        self.__ayz = 0
        self.__byz = 0
        self.__cyz = 0

    def on_enter(self, userdata):
        self.__filterchain_obj_sub = rospy.Subscriber(self.__obj_topic, FilterchainTarget, self.__filterchain_obj_cb)
        self.__filterchain_box_sub = rospy.Subscriber(self.__box_topic, BoundingBox2D, self.__filterchain_box_cb)
        self.__get_controller_info_sub = rospy.Subscriber('/proc_control/controller_info', MpcInfo, self.__get_controller_info_cb)
        self.__move_pub = rospy.Publisher("/proc_control/add_pose", AddPose, queue_size=10)
        self.__img_buffer.clear()

        self.__ax = userdata.x_func[0]
        self.__bx = userdata.x_func[1]
        self.__cx = userdata.x_func[2]
        self.__ayz = userdata.yz_function[0]
        self.__byz = userdata.yz_function[1]
        self.__cyz = userdata.yz_function[2]
    
    def execute(self, userdata):
        if self.__adjusts > self.__max_adjusts:
            return 'failed'
        if self.__wait_for_target:
            if self.__target_reached and self.__trajectory_done:
                self.__wait_for_target = False
        else:
            self.__adjusts += 1
            if not self.__wait_topic():
                return 'timeout'
        
            avg_x, avg_y = self.__calc_avg_center()
            delta_x = avg_x - self.__bounding_box.center.x
            delta_y =  avg_y - self.__bounding_box.center.y

            if self.__adjust_sub_move(delta_x, delta_y):
                self.__wait_for_target = True
                return
            
            avg_zoom = self.__calc_avg_size()
            delta_zoom = self.__blob_size - avg_zoom
            if self.__adjust_sub_zoom():
                self.__wait_for_target = True
                return

            return 'success'
            
    def on_exit(self, userdata):
        self.__filterchain_obj_sub.unregister()
        self.__get_controller_info_sub.unregister()
        self.__move_pub.unregister()

    def __filterchain_obj_cb(self, data: FilterchainTarget):
        self.__img_buffer.appendleft(data)

    def __filterchain_box_cb(self, data: BoundingBox2D):
        self.__bounding_box = data
        self.__filterchain_box_sub.unregister()

    def __wait_topic(self):
        self.__img_buffer.clear()
        timeout_counter = 0
        while len(self.__img_buffer) < self.__img_buffer.maxlen:
            sleep(0.01)
            timeout_counter += 1
            if timeout_counter > 100 * self.__timeout:
                return False
        return True

    def __calc_avg_center(self) -> Tuple[float, float]:
        avg_x = 0.0
        avg_y = 0.0
        for img in self.__img_buffer:
            avg_x += img.obj_ctr.x
            avg_y += img.obj_ctr.y

        avg_x /= self.__img_buffer.maxlen
        avg_y /= self.__img_buffer.maxlen

        return (avg_x, avg_y)

    def __adjust_sub_move(self, delta_x, delta_y):
        # dif in x is move in y
        # dif in y is move in z
        tol = self.__bounding_box.size_x / 2
        if abs(delta_x) <= tol and abs(delta_y) <= tol:
            return False
        pose = AddPose()
        if delta_x > tol:
            pose.position.y = self.__yz_function(delta_x)
        if delta_y > tol:
            pose.position.z = self.__yz_function(delta_y)
        self.__move_pub.publish(pose)
        return True

    def __get_controller_info_cb(self, data):
        self.__target_reached = data.target_reached
        self.__trajectory_done = data.is_trajectory_done

    def __adjust_sub_zoom(self, delta_zoom):
        tol = self.__blob_size * self.__tolerence_thresh
        if abs(delta_zoom) <= tol:
            return False
        pose = AddPose()
        pose.position.x = self.__x_function(delta_zoom)
        self.__move_pub.publish(pose)
        return True

    def __calc_avg_size(self):
        avg = 0
        for img in self.__img_buffer:
            avg += img.obj_size

        avg /= self.__img_buffer.maxlen
        return avg

    def __x_function(self, delta_pixels):
        return round((self.__ax * delta_pixels * delta_pixels) + (self.__bx * delta_pixels) + self.__cx, 3)

    def __yz_function(self, delta_pixels):
        return round((self.__ayz * delta_pixels * delta_pixels) + (self.__byz * delta_pixels) + self.__cyz, 3)