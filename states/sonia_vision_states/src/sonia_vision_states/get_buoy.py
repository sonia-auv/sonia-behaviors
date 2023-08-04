from flexbe_core import EventState, Logger

import rospy

from sonia_common.msg import FilterchainTarget
from queue import Queue
from time import sleep

class get_buoy(EventState):

    def __init__(self):
        super().__init__(outcomes=["success", "failed"],
                         output_keys=["average_size", "average_ctr"])

        self.__sub = None
        self.__num_imgs = 10
        self.__nb_attempts = 0
        self.__queue = Queue()

    def on_enter(self, userdata):
        self.__sub = rospy.Subscriber("/proc_image_processing/buoy_target", FilterchainTarget, self.__callback)

    def execute(self, userdata):
        while True:
            if self.__nb_attempts > 10:
                Logger.logerr("Too many attempts to get images")
                return 'failed'
            sleep(1)
            if len(self.__queue) >= self.__max_img_nb:
                break
            self.__nb_attempts += 1

        avg_size = sum([data.obj_size for data in self.__queue]) / len(self.__queue)
        userdata.average_size = avg_size
        Logger.loghint(f"Average size found: {avg_size}")
        avg_x = sum([data.obj_ctr.x for data in self.__queue]) / len(self.__queue)
        avg_y = sum([data.obj_ctr.y for data in self.__queue]) / len(self.__queue)
        userdata.average_ctr = (avg_x, avg_y)
        return "success"

    def on_exit(self, userdata):
        self.__sub.unregister()

    def __callback(self, data):
        self.__queue.put(data)