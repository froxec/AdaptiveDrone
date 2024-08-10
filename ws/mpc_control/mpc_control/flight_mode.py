from enum import Enum


class FlightMode(Enum):
    """
    Enum for ArduCopter flight modes.
    """

    STABILIZE = "STABILIZE"
    ACRO = "ACRO"
    ALT_HOLD = "ALT_HOLD"
    AUTO = "AUTO"
    GUIDED = "GUIDED"
    LOITER = "LOITER"
    RTL = "RTL"
    CIRCLE = "CIRCLE"
    POSITION = "POSITION"
    LAND = "LAND"
    OF_LOITER = "OF_LOITER"
    DRIFT = "DRIFT"
    SPORT = "SPORT"
    FLIP = "FLIP"
    AUTOTUNE = "AUTOTUNE"
    POSHOLD = "POSHOLD"
    BRAKE = "BRAKE"
    THROW = "THROW"
    AVOID_ADSB = "AVOID_ADSB"
    GUIDED_NOGPS = "GUIDED_NOGPS"
