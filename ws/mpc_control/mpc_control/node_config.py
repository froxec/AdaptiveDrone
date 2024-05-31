from mavros_msgs.srv import SetMode, CommandBool, CommandTOL
from mavros_msgs.msg import State
from mpc_control.callbacks import ArdupilotStateCallbackHandler
# You can add services, publishers, subscribers for your rclpy.Node below

""" Node config template
- SERVICE {"type" -> Message type, "name" -> custom service client name, 
           "topic" -> service topic name}
- PUBLISHER {"type" -> Message type, "name" -> custom service client name,
             "topic" -> topic name, "qos_profile" -> QoS profile (int)}
- SUBSCRIBER {"type" -> Message type, "name" -> custom service client name,
              "callback_handler"-> CallbackHandler class (not an object), "topic" -> topic name, "qos_profile" -> QoS profile (int)} 

Note: Callback Handlers are defined in callback.py. You can define a custom handler, derived from CallbackHandler.
"""
node_config = {
    "services": [
        {"type": SetMode, "name": "set_mode", "topic": "/mavros/set_mode"},
        {"type": CommandBool, "name": "arming", "topic": "/mavros/cmd/arming"},
        {"type": CommandTOL, "name": "takeoff", "topic": "/mavros/cmd/takeoff_local"} 
    ],
    "publishers": [],
    "subscribers": [
        {"type": State, "name": "ardupilot_state", "callback_handler_class": ArdupilotStateCallbackHandler, "topic": "/mavros/state", "qos_profile": 10}
    ]
} 