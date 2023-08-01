
from flexbe_core import EventState, Logger


class init_count_loop(EventState):

    def __init__(self):
        super(init_count_loop, self).__init__(outcomes=['success'],
                         output_keys=['count_loop'])

    def execute(self, userdata):
        userdata.count_loop = []
        userdata.count_loop.append(0)
        Logger.log("Calc_block created", Logger.REPORT_HINT)
        return 'success'