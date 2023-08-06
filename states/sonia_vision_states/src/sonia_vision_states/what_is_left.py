from flexbe_core import EventState


class what_is_left(EventState):

    def __init__(self):
        super().__init__(outcomes=["Earth", "Abydos", "blind"],
                         input_keys=["ctr_earth", "ctr_abydos"])

    def execute(self, userdata):
        try:
            y_earch = userdata.ctr_earth[0]
            y_abydos = userdata.ctr_abydos[0]

            if y_earch > y_abydos:
                return "Earth"
            return "Abydos"
        except Exception:
            return "blind"
