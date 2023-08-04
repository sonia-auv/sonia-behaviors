import rospy
from flexbe_core import EventState, Logger
from time import sleep
from sonia_common.msg import DetectionArray

class get_ai(EventState):
    """
    Get AI position on screen
        -- class_name               str     The name of the class of the image we are looking for
        -- nb_img                   int     Number of images needed calculate average. Defualts to 10.
        
        #> ai_pos                   list    Position on screen (x, y) of the object

        <= success                          If able to collect ai position
        <= failed                           If failed to collect ai position
    """
    def __init__(self, class_name, nb_img=10):
        super(get_ai, self).__init__(outcomes = ['success', 'failed'],
                                            output_keys = ['ai_pos'])
        self.__class_name = class_name
        self.__nb_img = nb_img
        self.__queue = []

    def on_enter(self, userdata):
        Logger.log("Enter get ai position", Logger.REPORT_HINT)
        self.__obj_sub = rospy.Subscriber("/proc_detection/bounding_box", DetectionArray, self.__filterchain_obj_sub_cb)

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
        somme = 0
        
        for i in self.__queue :
            somme += i.top
        somme = somme/len(self.__queue)
        userdata.ai_pos[0] = somme

        for i in self.__queue :
            somme += i.left
        somme = somme/len(self.__queue)
        userdata.ai_pos[1] = somme

        for i in self.__queue :
            somme += i.bottom
        somme = somme/len(self.__queue)
        userdata.ai_pos[2] = somme

        for i in self.__queue :
            somme += i.right
        somme = somme/len(self.__queue)
        userdata.ai_pos[3] = somme        
        
        Logger.log(f"AI target aquired {size_avg}", Logger.REPORT_HINT)
            
        return 'success'


    def __filterchain_obj_sub_cb(self, msg: DetectionArray):
        if len(self.__queue) < self.__nb_img:
            for i in msg :
                if i.class_name == self.__class_name :
                    self.__queue.append(i)
