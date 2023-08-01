from flexbe_core import EventState, Logger


class calculate_function_constants(EventState):
    """
        -- order        int     1=first order function, 2=second order function.
        -- precision    int     number of decimal places to round to.

        ># calc_block           List of Tuples containing the points and the diff.
        #> func_block           Dictionary with the constants for a, b, and c.

        <= success              If the calculation had enough points to generate the constants.
        <= failed               If there are not enough points to generate the constants.
    """


    def __init__(self, order=1, precision=3):
        super().__init__(
            outcomes=["success", "failed"],
            input_keys=["calc_block"],
            output_keys=["func_block"]
        )
        self.__order = order
        self.__precision = precision

    def execute(self, userdata):
        calc_block: list = userdata.calc_block
        if (len(calc_block) < self.__order + 1):
            Logger.loghint(f"Not enough points for a function of order {self.__order}")
            return "failed"

        x_t = round(userdata.calc_block[0][0], self.__precision)
        Logger.loghint(f"x_t: {x_t}")
        # original position at x = 0, z = 0
        x1 = round(x_t - userdata.calc_block[0][0], self.__precision)
        y1 = round(userdata.calc_block[0][1], self.__precision)
        Logger.loghint(f"x1: {x1}, y1: {y1}")

        x2 = round(x_t - userdata.calc_block[1][0], self.__precision)
        y2 = round(userdata.calc_block[1][1], 3)
        Logger.loghint(f"x2: {x2}, y2: {y2}")

        if self.__order == 1:
            x3 = round(userdata.calc_block[1][0] - x_t, self.__precision)
            y3 = round(userdata.calc_block[1][1] * -1, self.__precision)
            Logger.loghint(f"x3: {x3}, y3: {y3}")
        elif self.__order == 2:
            x3 = round(x_t - userdata.calc_block[2][0], self.__precision)
            y3 = round(userdata.calc_block[2][1], self.__precision)
            Logger.loghint(f"x3: {x3}, y3: {y3}")

        try:
            a = round((x1 * (y3 - y2) + x2 * (y1 - y3) + x3 * (y2 - y1)) / ((x1 - x2) * (x1 - x3) * (x2 - x3)), self.__precision)
        except ZeroDivisionError:
            a = 0
        b = round(((y2 - y1) / (x2 - x1)) - a * (x1 + x2), self.__precision)
        c = round(y1 - a * (x1 * x1) - b * x1, self.__precision)

        # return y = ax^2 + bx + c
        userdata.func_block = {}
        userdata.func_block['a'] = a
        userdata.func_block['b'] = b
        userdata.func_block['c'] = c
        Logger.loghint(f"{a}x^2 + {b}x + {c} is the function for this situation")
        return "success"
