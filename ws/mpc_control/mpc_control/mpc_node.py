import rclpy
import rclpy.client
from rclpy.node import Node
from rclpy.logging import LoggingSeverity
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL
from mavros_msgs.msg import State
from typing import TypedDict, Dict, Type
from typing import Any
import time
import rclpy.publisher
import rclpy.service
import rclpy.subscription

from mpc_control.flight_mode import FlightMode
from mpc_control.callbacks import CallbackHandler


class ServiceDict(TypedDict):
    client: rclpy.client.Client
    msg_type: Any


class PublisherDict(TypedDict):
    pub: rclpy.publisher.Publisher
    msg_type: Any


class SubscriberDict(TypedDict):
    sub: rclpy.subscription.Subscription
    callback_handler: CallbackHandler
    msg_type: Any


class CustomNode(Node):
    """ CustomNode class represents ROS nodes Python proxy. This class is used to encapsulate publishers, \
    subscribes and services which define a specific node. Node configuration is passed as a dict. For the dict template look at \
    node_config.py.
    Args:
        node_name (string) -> node name
        node_config_dict (dict) -> configuration dict
    """

    def __init__(self, node_name, node_config_dict):
        Node.__init__(self, node_name)
        self.service_clients: Dict[str, ServiceDict] = {}
        self.pubs: Dict[str, PublisherDict] = {}
        self.subs: Dict[str, SubscriberDict] = {}
        self.config_dict = node_config_dict
        self._initialize_node(self.config_dict)

    def _create_service_client(self, name: str, type, topic: str):
        client = self.create_client(type, topic)
        serv_dict = ServiceDict(client=client, msg_type=type)
        self.service_clients[name] = serv_dict

    def get_service(self, name: str) -> ServiceDict:
        return self.service_clients[name]

    def _create_pub(self, name: str, type, topic: str, qos_profile: int = 10):
        pub = self.create_publisher(type, topic, qos_profile)
        pub_dict = PublisherDict(pub=pub, msg_type=type)
        self.pubs[name] = pub_dict

    def get_pub(self, name: str) -> PublisherDict:
        return self.pubs[name]

    def _create_sub(
        self,
        name: str,
        callback_handler_class: Type[CallbackHandler],
        type,
        topic: str,
        qos_profile: int = 10,
    ):
        # instantiate callback handler
        callback_handler = callback_handler_class()
        sub = self.create_subscription(
            type, topic, callback_handler.callback, qos_profile
        )
        sub_dict = SubscriberDict(
            sub=sub, msg_type=type, callback_handler=callback_handler
        )
        self.subs[name] = sub_dict

    def get_sub(self, name: str) -> SubscriberDict:
        return self.sub[name]

    def _initialize_node(self, node_config_dict: dict):
        services = node_config_dict["services"]
        publishers = node_config_dict["publishers"]
        subscribers = node_config_dict["subscribers"]
        self._initialize_services(services)
        self._initialize_publishers(publishers)
        self._initialize_subscribers(subscribers)

    def _initialize_services(self, services: list):
        for service in services:
            self._create_service_client(
                name=service["name"], type=service["type"], topic=service["topic"]
            )

    def _initialize_publishers(self, publishers: list):
        for publisher in publishers:
            self._create_pub(
                name=publisher["name"],
                type=publisher["type"],
                topic=publisher["topic"],
                qos_profile=publisher["qos_profile"],
            )

    def _initialize_subscribers(self, subscribers: list):
        for subscriber in subscribers:
            self._create_sub(
                name=subscriber["name"],
                callback_handler_class=subscriber["callback_handler_class"],
                type=subscriber["type"],
                topic=subscriber["topic"],
                qos_profile=subscriber["qos_profile"],
            )


class ArduCopterManager:
    """
    ArduCopterManager class is used to manage ArduCopter.
    """

    def __init__(self, mavros_node: CustomNode) -> None:
        assert isinstance(mavros_node, CustomNode)
        self._mavros_node = mavros_node
        # state variables
        self._armed: bool = False
        self._connected: bool = False
        self._is_guided: bool = False
        self._is_manual: bool = False
        self._mode: FlightMode = FlightMode.STABILIZE
        self._ardupilot_state: State = None
        # self.initialize_properties()

    def update_properties(self) -> bool:
        if (
            self._mavros_node.subs["ardupilot_state"]["callback_handler"].last_message
            is not None
        ):
            self._armed = self._mavros_node.subs["ardupilot_state"]["callback_handler"].armed
            self._connected = self._mavros_node.subs["ardupilot_state"]["callback_handler"].connected
            self._is_guided = self._mavros_node.subs["ardupilot_state"]["callback_handler"].guided
            self._is_manual = self._mavros_node.subs["ardupilot_state"]["callback_handler"].manual_input
            self._mode = self._mavros_node.subs["ardupilot_state"]["callback_handler"].mode
            return True
        else:
            print("HALO")
            # self._mavros_node.get_logger().log("[Copter Manager]: State not initialized yet.")
            return False

    def initialize_properties(self) -> None:
        while True:
            if self.update_properties():
                # self._mavros_node.get_logger().log("[Copter Manager]: State request successful.")
                break
            time.sleep(1.0)

    @property
    def armed(self) -> bool:
        return self._armed

    @property
    def mode(self) -> FlightMode:
        return self._mode

    @armed.setter
    def armed(self, value: bool) -> None:
        assert isinstance(value, bool)
        # arming mode: True -> arming; False -> disarming
        if value:
            mode = "Arming"
        else:
            mode = "Disarming"
        # get service client
        serv_dict = self._mavros_node.get_service("arming")
        # prepare message
        msg = CommandBool.Request()
        assert serv_dict["msg_type"] is CommandBool
        msg.value = value
        # check if service is ready for requests
        if not self.check_service_readiness(serv_dict["client"], mode):
            return
        # issue request
        future_response = self.issue_request(
            service_client=serv_dict["client"], msg=msg, operation_name=mode
        )
        future_response.add_done_callback(
            lambda future: self.service_response_callback(
                future, call_name="ARM_DISARM_SERVICE"
            )
        )

    @mode.setter
    def mode(self, flight_mode: FlightMode) -> None:
        assert isinstance(flight_mode, FlightMode)
        # get service client
        serv_dict = self._mavros_node.get_service("set_mode")
        # prepare message
        assert serv_dict["msg_type"] is SetMode
        msg = SetMode.Request()
        msg.custom_mode = flight_mode.value
        if not self.check_service_readiness(serv_dict["client"], flight_mode):
            return
        # issue request
        future_response = self.issue_request(
            service_client=serv_dict["client"],
            msg=msg,
            operation_name=f"FLIGHT MODE request: {flight_mode}",
        )
        future_response.add_done_callback(
            lambda future: self.service_response_callback(
                future, call_name="FLIGHT MODE request: {flight_mode}"
            )
        )

    def service_response_callback(self, future, call_name):
        self._mavros_node.get_logger().log(
            f"[Service Response]: Call {call_name} finished.",
            severity=LoggingSeverity.INFO,
        )


    def check_service_readiness(
        self, service_client: rclpy.client.Client, opertion_name: str
    ) -> bool:
        if not service_client.service_is_ready():
            self._mavros_node.get_logger().log(
                f"[Copter Manager]: {opertion_name} failed: Service not ready",
                severity=LoggingSeverity.WARN,
            )
            return False
        else:
            return True

    def issue_request(
        self,
        service_client: rclpy.client.Client,
        msg: rclpy.service.SrvTypeRequest,
        operation_name: str,
    ) -> rclpy.service.SrvTypeResponse:
        future_response = service_client.call_async(msg)
        self._mavros_node.get_logger().log(
            f"[Copter Manager]: {operation_name} request issued.",
            severity=LoggingSeverity.INFO,
        )
        return future_response


class Controller(CustomNode):
    def __init__(self, node_config):
        super().__init__("controller", node_config)
        self.arducopter_manager = ArduCopterManager(self)
        timer_period = 1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        # if not self.arducopter_manager.armed:
        #     self.arducopter_manager.armed = True
        self.arducopter_manager.mode = FlightMode.GUIDED


def main(args=None):
    from mpc_control.node_config import node_config
    rclpy.init(args=args)
    print(FlightMode("GUIDED"))
    controller_node = Controller(node_config)

    rclpy.spin(controller_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    controller_node.destroy_node()
    rclpy.shutdown()


def set_copter_mode(flight_mode: FlightMode, node: Controller) -> None:
    msg = SetMode.Request()
    msg.custom_mode = flight_mode
    node.get_logger().log(f"FLIGHT MODE REQUEST: {flight_mode}", LoggingSeverity.INFO)
    node.set_mode_client_.call(msg)


def takeoff(altitude: float, node: Controller) -> None:
    msg = CommandTOL.Request()
    msg.altitude = altitude
    print(f"Taking off [ALTITUDE {altitude}]")
    node.takeoff_client.call(msg)


if __name__ == "__main__":
    main()
