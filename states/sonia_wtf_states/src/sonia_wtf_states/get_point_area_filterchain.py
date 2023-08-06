from flexbe_core import EventState, Logger
import rospy
from queue import Queue
from sonia_common.msg import FilterchainTarget
from time import sleep


class GetPointAreaFilterChain(EventState):
    """
    Get the center X and Y values as well as the area. All values are averaged over the `needed_imgs`.
    -- topic        str     Topic to subscribe to. Needs to be of type `FilterchainTarget`.
    -- needed_imgs  int     Number of needed images. Defaults to 10.
    -- timeout      int     Seconds before timed out.

    #> delta_x      float   Delta x average over number of specified images.
    #> delta_y      float   Delta y average over number of specified images.
    #> area         float   Area average over number of specified images.

    <= success              If enough images where aquired and average was returned.
    <= timeout              If took too long to get images.
    """
    def __init__(self, topic, needed_imgs=10, timeout=10):
        super().__init__(outcomes=["success", "timeout"],
                         output_keys=["delta_x", "delta_y", "area"])
        self.__topic = topic
        self.__sub = None
        self.__queue = Queue()
        self.__needed_imgs = needed_imgs
        self.__timeout = timeout

    def on_enter(self, userdata):
        Logger.loghint(f"Entered {self.__class__}")
        self.__sub = rospy.Subscriber(self.__topic, FilterchainTarget, self.__sub_cb)

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
        for data in self.__queue:
            data: FilterchainTarget
            userdata.delta_x += data.obj_ctr.x
            userdata.delta_y += data.obj_ctr.y
            userdata.area += data.obj_size

        userdata.delta_x /= len(self.__queue)
        userdata.delta_y /= len(self.__queue)
        userdata.area /= len(self.__queue)
        return "success"

    def on_exit(self, userdata):
        self.__sub.unregister()

    def __sub_cb(self, data: FilterchainTarget):
        self.__queue.put(data)
