from flexbe_core import EventState, Logger
import rospy
from queue import Queue
from time import sleep
import math
from sonia_common.msg import DetectionArray, Detection

class ImportantFunction(EventState):

    def __init__(self, topic, class_, needed_imgs=10, timeout=10, screen_width=600, screen_height=400):
        super().__init__(outcomes=["success", "timeout"],
                         output_keys=["delta_x", "delta_y", "delta_z"])
        self.__topic = topic
        self.__class = class_
        self.__screen_width = screen_width
        self.__screen_height = screen_height
        self.__sub = None
        self.__queue = Queue()
        self.__needed_imgs = needed_imgs
        self.__timeout = timeout

    def on_enter(self, userdata):
        Logger.loghint(f"Entered {self.__class__}")
        self.__sub = rospy.Subscriber(self.__topic, DetectionArray, self.__sub_cb)

    def execute(self, userdata):
        timeout_counter = 0
        while True:
            if len(self.__queue) >= self.__needed_imgs:
                break
            if timeout_counter > self.__timeout:
                Logger.logerr(f"Took too long to find enough images. Found {len(self.__queue)} out of {self.__needed_imgs}")
                return "timeout"
            sleep(1)
            timeout_counter += 1

        self.__sub.unregister()

        avg_top = sum([x.top for x in self.__queue]) / len(self.__queue)
        avg_left = sum([x.left for x in self.__queue]) / len(self.__queue)
        avg_bottom = sum([x.bottom for x in self.__queue]) / len(self.__queue)
        avg_right = sum([x.right for x in self.__queue]) / len(self.__queue)

        deltas = self.get_information(avg_left, avg_top, avg_right, avg_bottom, self.__class)
        userdata.delta_x = deltas[0]
        userdata.delta_y = deltas[1]
        userdata.delta_z = deltas[2]

    def __sub_cb(self, data: DetectionArray):
        for detection in data.detected_object:
            if detection.class_name == self.__class:
                self.__queue.put(detection)
                break

    def get_information(self, left, top, right, bottom, class_name):
        focal_length = 0.0018
        real_size_dict = {"Gate_Earth": [0.155, 0.212], "Gate_Abydos": [0.155, 0.212], "Glyph_Earth_1": [0.58, 0.45],
                        "Glyph_Earth_2": [0.63, 0.49], "Glyph_Abydos_1": [0.57, 0.52], "Glyph_Abydos_2": [0.53, 0.43]}
        angle_left = (left - 300) * focal_length
        angle_right = (right - 300) * focal_length
        angle_top = (top - 200) * focal_length
        angle_bottom = (bottom - 200) * focal_length
        delta_x_1 = real_size_dict[class_name][0] / (math.tan(angle_right) - math.tan(angle_left))
        delta_x_2 = real_size_dict[class_name][1] / (math.tan(angle_bottom) - math.tan(angle_top))
        delta_x_3 = real_size_dict[class_name][1] / (math.tan(angle_right) - math.tan(angle_left))
        delta_x_4 = real_size_dict[class_name][0] / (math.tan(angle_bottom) - math.tan(angle_top))
        prop_not_rotated = max(delta_x_1, delta_x_2) / min(delta_x_1, delta_x_2)
        prop_rotated = max(delta_x_3, delta_x_4) / min(delta_x_3, delta_x_4)
        if min(prop_rotated, prop_not_rotated) > 1.5:
            return None
        if prop_rotated < prop_not_rotated:
            rotated = True
            delta_x = (delta_x_3 + delta_x_4) / 2
        else:
            rotated = False
            delta_x = (delta_x_1 + delta_x_2) / 2
        if not rotated:
            delta_y = delta_x * math.tan(angle_left) + 0.5 * real_size_dict[class_name][0]
            delta_z = delta_x * math.tan(angle_top) + 0.5 * real_size_dict[class_name][1]
        else:
            delta_y = delta_x * math.tan(angle_left) + 0.5 * real_size_dict[class_name][1]
            delta_z = delta_x * math.tan(angle_top) + 0.5 * real_size_dict[class_name][0]
        return [delta_x, delta_y, delta_z]