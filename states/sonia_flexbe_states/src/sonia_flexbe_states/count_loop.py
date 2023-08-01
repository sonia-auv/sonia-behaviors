#!/usr/bin/env python 
#-*- coding: utf-8 -*-

from flexbe_core import EventState, Logger

class count_loop(EventState):
    
    """
        -- nb tries             int     The number of tries before it fail

        ># count_loop           The number of tries
        #> count_loop           The number of tries

        <= success              If the number of tries is below the max value
        <= failed               If the number of tries is over the max value
    """


    def __init__(self, tries=10):
        super().__init__(
            outcomes=["success", "failed"],
            input_keys=["count_loop"],
            output_keys=["count_loop"]
        )
        self.__tries = tries

    def execute(self, userdata):
        if (userdata.count_loop[0] >= self.__tries):
            Logger.loghint(f"The number of tries exceeded {self.__tries}")
            return "failed"

        userdata.count_loop[0] += 1
        return "success"
