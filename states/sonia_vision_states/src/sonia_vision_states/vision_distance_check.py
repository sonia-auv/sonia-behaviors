from time import time, sleep
import math
import rospy
from typing import List, Tuple
from collections import deque
from flexbe_core import EventState, Logger
from sonia_common.msg import FilterchainTarget, AddPose, MpcInfo, CenterShapeBoundingBox, BoundingBox2D, MultiAddPose

class vision_distance_check(EventState):
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


    def __init__(self, dist_to_target, nb_imgs = 10, timeout_sec = 5, max_adjusts = 10, tolerance = 5):
        super(vision_distance_check, self).__init__(outcomes = ['timeout', 'success', 'failed'],
                                                     input_keys = ['calc_block'])
        self.__max_adjusts = max_adjusts
        self.__dist_to_target = dist_to_target
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


    def on_enter(self, userdata):
        self.__filterchain_obj_sub = rospy.Subscriber(self.__obj_topic, FilterchainTarget, self.__filterchain_obj_cb)
        self.__filterchain_box_sub = rospy.Subscriber(self.__box_topic, BoundingBox2D, self.__filterchain_box_cb)
        self.__get_controller_info_sub = rospy.Subscriber('/proc_control/controller_info', MpcInfo, self.__get_controller_info_cb)
        self.__move_pub = rospy.Publisher('/proc_planner/send_multi_addpose', MultiAddPose, queue_size=1)
        self.__img_buffer.clear()

    def execute(self, userdata):
        if self.__adjusts > self.__max_adjusts:
            return 'failed'
        
        if self.__wait_for_target and not self.__target_received:
            if not self.__target_reached and not self.__trajectory_done:
                self.__target_received = True
        elif self.__wait_for_target:
            Logger.loghint("Target Reached")
            self.__wait_for_target = False
        else:
            self.__get_controller_info_sub.unregister()
            self.__adjusts += 1
            if not self.__wait_topic():
                return 'timeout'
            Logger.loghint(f"Adjust number {self.__adjusts}")

            target_angle = userdata.calc_block[0]
            ref_angle = userdata.calc_block[1]
            distance = 1
            z_rotation = target_angle 
            if target_angle < ref_angle:
                z_rotation *= -1
            y = distance * math.sin(target_angle)
            x = distance - (distance * math.cos(target_angle))
            
            if self.__adjust_sub_rotate(z_rotation, x, y):
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

    def __adjust_sub_rotate(self, z, x, y):
        # dif in x is move in y
        # dif in y is move in z
        pose = AddPose()
        pose.frame = 1
        pose.orientation.z = z
        pose.position.x = x
        pose.position.y = y
        poses = MultiAddPose()
        poses.pose.append(pose)
        Logger.loghint("Pose is { rotation_z: {}, position_x: {}, position_y: {} }".format(pose.orientation.z, pose.position.x, pose.position.y))
        self.__move_pub.publish(poses)
        # sleep(1.5)
        return True

    def __get_controller_info_cb(self, data):
        self.__target_reached = data.target_reached
        self.__trajectory_done = data.is_trajectory_done

    def __adjust_sub_zoom(self, delta_zoom):
        tol = self.__blob_size * self.__tolerence_thresh
        if abs(delta_zoom) <= tol:
            return False
        Logger.loghint("Moving sub in x")
        pose = AddPose()
        pose.frame = 1
        pose.position.x = self.__x_function(delta_zoom)
        Logger.loghint(f"{delta_zoom}px is {pose.position.x} in m for x")
        poses = MultiAddPose()
        poses.pose.append(pose)
        self.__move_pub.publish(poses)
        # sleep(1.5)
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