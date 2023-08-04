from flexbe_core import EventState

import rospy

from sonia_common.msg import FilterchainTarget

class touch_buoy(EventState):

    def __init__(self, buoy_target_1=0, buoy_target_2=1):
        super().__init__(outcomes=["success", "failed"],
                         output_keys=["target_1", "target_2"])
        self.__target_1 = buoy_target_1
        self.__target_2 = buoy_target_2
        self.__sub_tl = None
        self.__sub_tr = None
        self.__sub_bl = None
        self.__sub_br = None

    def on_entry(self, userdata):
        self.__sub_tl = rospy.Subscriber("proc_image_processing/tl", FilterchainTarget, self.__tl_cb)
        self.__sub_tr = rospy.Subscriber("proc_image_processing/tr", FilterchainTarget, self.__tr_cb)
        self.__sub_bl = rospy.Subscriber("proc_image_processing/bl", FilterchainTarget, self.__bl_cb)
        self.__sub_br = rospy.Subscriber("proc_image_processing/br", FilterchainTarget, self.__br_cb)

    def execute(self, userdata):
        ...

    def on_exit(self, userdata):
        self.__sub_tl.unregister()
        self.__sub_tr.unregister()
        self.__sub_bl.unregister()
        self.__sub_br.unregister()

    def __tl_cb(self, data):
        pass

    def __tr_cb(self, data):
        pass

    def __bl_cb(self, data):
        pass

    def __br_cb(self, data):
        pass
