"""M11-only one-shot FAKE blackout with monotonic schedule evidence.

Uses M10's unchanged synthetic depth/odom publishers and trigger service.
The ROS Trigger response adds cross-process CLOCK_MONOTONIC timestamps
for the scheduled blackout; no new ROS publisher or motor interface.
"""

from .m10_triggered_fake_fault import (
    TriggeredDepth, TriggeredOdom, BLACKOUT_S, _run,
)
from .m11_blackout_metrics import encode_window


class TimedDepth(TriggeredDepth):
    def _trigger(self, request, response):
        response = super()._trigger(request, response)
        if response.success:
            response.message = encode_window(
                start=self._fault_until - BLACKOUT_S,
                end=self._fault_until)
        return response


class TimedOdom(TriggeredOdom):
    def _trigger(self, request, response):
        response = super()._trigger(request, response)
        if response.success:
            response.message = encode_window(
                start=self._fault_until - BLACKOUT_S,
                end=self._fault_until)
        return response


def main_depth(args=None):
    _run(TimedDepth, args=args)


def main_odom(args=None):
    _run(TimedOdom, args=args)
