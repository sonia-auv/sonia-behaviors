from flexbe_core import EventState


class calculate_ctr_ai(EventState):
    def __init__(self):
        super().__init__(outcomes=["success"],
                         input_keys=["ai_pos"],
                         output_keys=["obj_ctr"])

    def execute(self, userdata):
        top = userdata.ai_pos[0]
        left = userdata.ai_pos[1]
        right = userdata.ai_pos[2]
        bottom = userdata.ai_pos[3]

        avg_x = (left + right) / 2
        avg_y = (top + bottom) / 2
        userdata.obj_ctr.x = avg_x
        userdata.obj_ctr.y = avg_y
        return "success"
