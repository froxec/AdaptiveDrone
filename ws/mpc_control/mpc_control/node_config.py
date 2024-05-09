from mavros_msgs.srv import SetMode, CommandBool, CommandTOL

# You can add services, publishers, subscribers for your rclpy.Node below

""" Node config template
- SERVICE {"type" -> Message type, "name" -> custom service client name, 
           "topic" -> service topic name}
- PUBLISHER {"type" -> Message type, "name" -> custom service client name,
             "topic" -> topic name, "qos_profile" -> QoS profile (int)}
- SUBSCRIBER {"type" -> Message type, "name" -> custom service client name,
              "callback"-> callback function defined in callback.py, "topic" -> topic name, "qos_profile" -> QoS profile (int)} 

"""
node_config = {
    "services": [
        {"type": SetMode, "name": "set_mode", "topic": "/mavros/set_mode"},
        {"type": CommandBool, "name": "arming", "topic": "/mavros/cmd/arming"},
        {"type": CommandTOL, "name": "takeoff", "topic": "/mavros/cmd/takeoff_local"} 
    ],
    "publishers": [],
    "subscribers": []
} 