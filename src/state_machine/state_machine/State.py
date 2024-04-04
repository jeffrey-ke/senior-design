from rclpy.action import ActionClient

class State(ActionClient):

    next_states_ = {}
    TIMEOUT_WAIT_FOR_SERVER_SECONDS = 60
    def __init__(self, node, action_type, name) -> None:
        super().__init__(node, action_type, name)

    def AddTransition(self, state, label):
        self.next_states_[label] = state

    
from collections import deque
from profiler_msgs.msg import Waypoint
from profiler_msgs.action import DoWaypoint

class Waypoint(State):
    itinerary_ = deque()
    def __init__(self, node, action_type, name) -> None:
        super().__init__(node, action_type, name)

    def Execute(self):
        # Send waypoint that's first in the queue
        while self.WaypointsRemaining():
            next_wp = DoWaypoint.Goal(self.GetFirstWaypoint())
            self.wait_for_server(self.TIMEOUT_WAIT_FOR_SERVER_SECONDS)
            goal_handle = self.send_goal_async(next_wp)
            goal_handle.add_done_callback(self._on_goal_acknowledge)
        pass

    def AddWaypoint(self, wp: Waypoint):
        self.itinerary_.append(wp)

    def GetFirstWaypoint(self):
        return self.itinerary_.popleft()
    
    def WaypointsRemaining(self):
        return len(self.itinerary_) > 0
    
    def _on_goal_acknowledge(self):
        pass