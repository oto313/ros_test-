from dataclasses import dataclass


@dataclass
class AivaRosBridgeOptions:
   robot_ip: str
   external_robots_uri: str