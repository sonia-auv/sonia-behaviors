from flexbe_core import EventState, Logger
import rospy
from sonia_common.msg import FilterchainTargetAngle
from time import sleep
from queue import Queue


class get_pipe_angle_pos(EventState):
    _TOPIC ="/proc_image_processing/pipe_target"

    def __init__(self, num_imgs=10):
        super().__init__(outcomes=["Success", "Failed"],
                         output_keys=["angle_average", "average_ctr", "average_size"])
        self.__sub = None
        self.__num_imgs = 10
        self.__nb_attempts = 0
        self.__queue = Queue()

    def on_enter(self, userdata):
        self.__sub = rospy.Subscriber(self._TOPIC, FilterchainTargetAngle, self.__angle_cb)

    def execute(self, userdata):
        while True:
            if self.__nb_attempts > 10:
                Logger.logerr("Too many attempts to get images")
                return 'failed'
            sleep(1)
            if len(self.__queue) >= self.__max_img_nb:
                break
            self.__nb_attempts += 1

        angle_avg = sum([x.obj_angle for x in self.__queue]) / len(self.__queue)
        userdata.angle_average = angle_avg
        Logger.loghint(f"Pipe angle aquired {angle_avg}")
        avg_x = sum([data.obj_ctr.x for data in self.__queue]) / len(self.__queue)
        avg_y = sum([data.obj_ctr.y for data in self.__queue]) / len(self.__queue)
        userdata.average_ctr = (avg_x, avg_y)
        Logger.loghint(f"Pipe ctr aquired {avg_x}, {avg_y}")

        avg_size = sum([x.obj_size for x in self.__queue])/ len(self.__queue)
        Logger.loghint(f"Pipe size aquired {avg_size}")
        return "success"

    def on_exit(self, userdata):
        self.__sub.unregister()

    def __angle_cb(self, data: FilterchainTargetAngle):
        self.__queue.put(data)
            
