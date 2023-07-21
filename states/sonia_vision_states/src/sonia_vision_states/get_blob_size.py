import rospy
from flexbe_core import EventState, Logger
from time import sleep
from sonia_common.msg import FilterchainTarget

class get_blob_size(EventState):
    """
    Get Blob size for calculation
        -- filterchain_obj_topic    str     The topic from the filterchain to subscribe to get the object.
        -- dist_from_origin         int     Move distance in meters. Defaults to 0.1.
        -- nb_img                   int     Number of images needed calculate average. Defualts to 10.

        ># calc_block               dict    block of all calculations done
        
        #> calc_block               dict    block of all calculations done includeing this step

        <= success                          If able to collect blob size
        <= failed                           If failed to collect blob size
    """
    def __init__(self, filterchain_obj_topic, dist_from_origin=0.1, nb_img=10):
        super(get_blob_size, self).__init__(outcomes = ['success', 'failed'],
                                            input_keys = ['calc_block'],
                                            output_keys = ['calc_block'])
        self.__obj_topic = filterchain_obj_topic
        self.__obj_sub = None
        self.__queue = []
        self.__max_img_nb = nb_img
        self.__nb_attempts = 0
        self.__dist_from_origin = dist_from_origin

    def on_enter(self, userdata):
        Logger.log("Enter get blob size", Logger.REPORT_HINT)
        self.__obj_sub = rospy.Subscriber(self.__obj_topic, FilterchainTarget, self.__filterchain_obj_sub_cb)

    def on_exit(self, userdata):
        self.__obj_sub.unregister()

    def execute(self, userdata):
        
        while True:
            if self.__nb_attempts > 10:
                Logger.logerr("Too many attempts to get images")
                return 'failed'
            sleep(1)
            if len(self.__queue) >= self.__max_img_nb:
                break
            self.__nb_attempts += 1
        
        # Calculate average
        size_avg = sum(self.__queue) / len(self.__queue)
        userdata.calc_block.append((size_avg, self.__dist_from_origin))
        Logger.log(f"blob size aquired {size_avg}", Logger.REPORT_HINT)
        return 'success'


    def __filterchain_obj_sub_cb(self, msg: FilterchainTarget):
        if len(self.__queue) < self.__max_img_nb:
            self.__queue.append(msg.obj_size)
