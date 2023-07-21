from flexbe_core import EventState, Logger


class calc_pixel_meter_ratio(EventState):
    def __init__(self):
        super().__init__(
            outcomes=["success", "failed"],
            input_keys=["calc_block"],
            output_keys=["pixel_meter_function"],
        )

    def execute(self, userdata):
        calc_block: list = userdata.calc_block
        if (len(calc_block) < 2):
            return "failed"

        x_t = round(userdata.calc_block[0][0],3)
        Logger.loghint(f"x_t: {x_t}")
        # original position at x = 0, z = 0
        x1 = round(x_t - userdata.calc_block[0][0],3)
        y1 = round(userdata.calc_block[0][1], 3)
        Logger.loghint(f"x1: {x1}, y1: {y1}")

        x2 = round(x_t - userdata.calc_block[1][0], 3)
        y2 = round(userdata.calc_block[1][1], 3)
        Logger.loghint(f"x2: {x2}, y2: {y2}")

        x3 = round(userdata.calc_block[1][0] - x_t, 3)
        y3 = round(userdata.calc_block[1][1] * -1, 3)
        Logger.loghint(f"x3: {x3}, y3: {y3}")

        a = round((x1 * (y3 - y2) + x2 * (y1 - y3) + x3 * (y2 - y1)) / ((x1 - x2) * (x1 - x3) * (x2 - x3)), 3)
        b = round(((y2 - y1) / (x2 - x1)) - a * (x1 + x2), 3)
        c = round(y1 - a * (x1 * x1) - b * x1, 3)

        # return y = ax^2 + bx + c

        userdata.pixel_meter_function = []
        userdata.pixel_meter_function.append(a)
        userdata.pixel_meter_function.append(b)
        userdata.pixel_meter_function.append(c)
        Logger.loghint(f"{a}x^2 + {b}x + {c} is the function for this situation")
        Logger.loghint(f"{userdata.pixel_meter_function} is the function for this situation")
        return "success"
