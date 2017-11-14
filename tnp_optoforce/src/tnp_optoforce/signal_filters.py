from __future__ import division

from collections import deque

class RunningAverageFilter(object):
    def __init__(self, **kwargs):
        self._window = deque(maxlen=kwargs["window_size"])
    
    def filter(self, data_point):
        self._window.append(data_point)

    def calculate_response(self):
        return sum(self._window) / max(len(self._window), 1)

class ExponentialFilter(object):
    def __init__(self, **kwargs):
        self._update_weight = kwargs["update_weight"]
        self._prev_response_weight = 1 - self._update_weight
        self._response = 0

    def filter(self, data_point):
        self._response = (self._update_weight * data_point
                          + self._prev_response_weight * self._response)

    def calculate_response(self):
        return self._response

def filter_factory(filter_id, **kwargs):
    if filter_id == "running_avg":
        return RunningAverageFilter(**kwargs)
    elif filter_id == "exponential":
        return ExponentialFilter(**kwargs)
    else:
        raise ValueError("Unknown filter: " + filter_id)
