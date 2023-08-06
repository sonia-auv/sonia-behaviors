from flexbe_core import EventState, Logger
import rospy
from queue import Queue
from sonia_common.msg import DetectionArray, Detection
from time import sleep
from statistics import median


class GetPointAreaAI(EventState):
    """
    Get the center X and Y values as well as the area. All values are averaged over the `needed_imgs`.
    -- topic            str     Topic to subscribe to. Needs to be of type `DetectionArray`.
    -- class_           str     Class to check for from the AI.
    -- needed_imgs      int     Number of needed images. Defaults to 10.
    -- timeout          int     Seconds before timed out.
    -- screen_width     int     Width of the screen in pixels.
    -- screen_height    int     Height of the screen in pixels.

    #> delta_x      float   Delta x average over number of specified images.
    #> delta_y      float   Delta y average over number of specified images.
    #> area         float   Area average over number of specified images.

    <= success              If enough images where aquired and average was returned.
    <= timeout              If took too long to get images.
    """
    def __init__(self, topic, class_, needed_imgs=10, timeout=10, screen_width=600, screen_height=400):
        super().__init__(outcomes=["success", "timeout"],
                         output_keys=["delta_x", "delta_y", "area"])
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

        userdata.delta_x = 0
        userdata.delta_y = 0
        userdata.area = 0
        delta_x_array = []
        delta_y_array = []
        area_array = []
        for data in self.__queue:
            data: Detection
            delta_x_array.append(((data.left + data.right)/2) * self.__screen_width)
            delta_y_array.append(((data.top + data.bottom)/2) * self.__screen_height)
            area_array.append((data.right - data.left) * (data.bottom - data.top) * self.__screen_height * self.__screen_width)

        userdata.delta_x = median(delta_x_array)
        userdata.delta_y = median(delta_y_array)
        userdata.area = median(area_array)
        return "success"

    def on_exit(self, userdata):
        self.__sub.unregister()

    def __sub_cb(self, data: DetectionArray):
        for detection in data:
            if detection.class_name == self.__class:
                self.__queue.put(detection)
                break
