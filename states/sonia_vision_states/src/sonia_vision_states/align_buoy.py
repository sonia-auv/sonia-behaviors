from flexbe_core import EventState

class align_buoy(EventState):

    def __init__(self):
        super().__init__(outcomes=["success", "failed"])