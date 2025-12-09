from dataclasses import dataclass, field
import copy
import logging

import zenoh
import msgpack

logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger(__name__)

cmd_vel = {
    "linear": {
        "x": 0.0,
        "y": 0.0,
        "z": 0.0,
    },
    "angular": {
        "x": 0.0,
        "y": 0.0,
        "z": 0.0,
    }
}

pose = {
    "position": {
        "x": 0.0,
        "y": 0.0,
        "z": 0.0
    },
    "orientation": {
        "x": 0.0,
        "y": 0.0,
        "z": 0.0,
        "w": 0.0
    },
}

@dataclass
class Robot:
    name: str
    cmd_vel_publisher: callable
    pose: dict = field(default_factory=lambda: copy.deepcopy(pose))
    cmd_vel: dict = field(default_factory=lambda: copy.deepcopy(cmd_vel))

    def set_cmd_vel(self, translation: float, rotation: float):
        self.cmd_vel["linear"]["x"] = translation
        self.cmd_vel["angular"]["z"] = rotation
        msg_packed = msgpack.packb(self.cmd_vel, use_bin_type=True)
        self.cmd_vel_publisher.put(msg_packed)


class Fleet:
    def __init__(self):
        self.robots = {}
        self.zenoh_session = zenoh.open(zenoh.Config())
        self.optitrack_subscription = self.zenoh_session.declare_subscriber(f"*/pose", self.optitrack_subscription_handler)

    def add_robot(self, robot_name: str) -> Robot:
        robot = Robot(
            name=robot_name,
            cmd_vel_publisher=self.zenoh_session.declare_publisher(f"{robot_name}/cmd_vel")
        )
        self.robots[robot_name] = robot
        return robot
    
    def optitrack_subscription_handler(self, sample):
        robot = str(sample.key_expr).split("/")[0]
        logger.debug(f"Updating \"{robot}\"")
        zenoh_data = msgpack.unpackb(sample.payload.to_bytes(), raw=False)
        
        pose = self.robots[robot].pose
        position = pose["position"]
        orientation = pose["orientation"]

        position["x"] = zenoh_data["pose"]["position"]["x"]
        position["y"] = zenoh_data["pose"]["position"]["y"]
        position["z"] = zenoh_data["pose"]["position"]["z"]
        orientation["x"] = zenoh_data["pose"]["orientation"]["x"]
        orientation["y"] = zenoh_data["pose"]["orientation"]["y"]
        orientation["z"] = zenoh_data["pose"]["orientation"]["z"]
        orientation["w"] = zenoh_data["pose"]["orientation"]["w"]


