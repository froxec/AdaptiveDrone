from enum import Enum

class FlightMode(Enum):
    """
    Enum for ArduCopter flight modes.
    """
    GUIDED = "GUIDED"
    LAND = "LAND"
    STABILIZE = "STABILIZE"