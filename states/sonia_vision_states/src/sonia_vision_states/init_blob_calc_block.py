
from flexbe_core import EventState, Logger


class init_blob_calc_block(EventState):

    def __init__(self):
        super(init_blob_calc_block, self).__init__(outcomes=['success'],
                         output_keys=['calc_block'])

    def execute(self, userdata):
        userdata.calc_block = []
        Logger.log("Calc_block created", Logger.REPORT_HINT)
        return 'success'