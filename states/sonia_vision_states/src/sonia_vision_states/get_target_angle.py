import rospy
import math
from flexbe_core import EventState, Logger
from time import sleep
from sonia_common.msg import FilterchainTarget

class get_target_angle(EventState):
    """
    Get target angle for calculation
        -- filterchain_obj_topic    str     The topic from the filterchain to subscribe to get the object.
        -- obj_ratio                float   The width/height ratio of the objerct
        -- nb_img                   int     Number of images needed calculate average. Defualts to 10.

        ># calc_block               dict    block of all calculations done
        
        #> calc_block               dict    block of all calculations done includeing this step

        <= success                          If able to collect blob size
        <= failed                           If failed to collect blob size
    """
    def __init__(self, filterchain_obj_topic, obj_ratio, nb_img=10):
        super(get_target_angle, self).__init__(outcomes = ['success', 'failed'],
                                            input_keys = ['calc_block'],
                                            output_keys = ['calc_block'])
        self.__obj_topic = filterchain_obj_topic
        self.__obj_sub = None
        self.__queue = []
        self.__max_img_nb = nb_img
        self.__nb_attempts = 0
        self.__obj_ratio = obj_ratio

    def on_enter(self, userdata):
        Logger.log("Enter get target angle", Logger.REPORT_HINT)
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
        angle_avg= sum(self.__queue) / len(self.__queue)
        calc_angle = math.acos((angle_avg/self.__obj_ratio) if (angle_avg/self.__obj_ratio) < 1 else 1)
        
        userdata.calc_block.append(math.degrees(calc_angle))
        Logger.loghint(f"Target angle aquired {math.degrees(calc_angle)}")
            
            
        return 'success'


    def __filterchain_obj_sub_cb(self, msg: FilterchainTarget):
        if len(self.__queue) < self.__max_img_nb:
            self.__queue.append(msg.obj_size)