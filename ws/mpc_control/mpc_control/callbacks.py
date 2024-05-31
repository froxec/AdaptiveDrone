from mavros_msgs.msg import State
from mpc_control.flight_mode import FlightMode

class CallbackHandler:
    def __init__(self) -> None:
        self.last_message = None
    
    def callback(self, msg) -> None:
        self.last_message = msg


class ArdupilotStateCallbackHandler(CallbackHandler):
    def __init__(self) -> None:
        CallbackHandler.__init__(self)
        self.connected : bool = False
        self.armed : bool = False
        self.guided : bool = False
        self.manual_input : bool = False
        self.mode : FlightMode = None
        self.last_message : State = None

    def callback(self, msg : State) -> None:
        self.last_message = msg
        self.connected = State.connected
        self.armed = State.armed
        self.guided = State.guided
        self.manual_input = State.manual_input
        self.mode = FlightMode(State.mode)
        