import asyncio
import logging
from typing import AsyncIterable

import rclpy
from dishka import Provider, Scope, make_async_container, provide

from aiva_ros_bridge.aiva_ros_bridge_node import AivaRosBridgeNode
from aiva_ros_bridge.aiva_ros_bridge_options import AivaRosBridgeOptions
from aiva_ros_bridge.config import settings
from aiva_ros_bridge.external_robots_client import ExternalRobotsClient
from aiva_ros_bridge.external_robots_handler import ExternalRobotsHandler
from aiva_ros_bridge.robot_definition import RobotDefinition

logger = logging.getLogger(__name__)


class AivaRosBridgeProvider(Provider):
    external_robots_handler = provide(ExternalRobotsHandler, scope=Scope.APP)

    def __init__(self, scope: Scope):
        super().__init__(scope=scope)  # do not forget `super`
        self.config = AivaRosBridgeOptions(
            robot_ip=settings.AivaRosBridgeOptions.RobotIp,
            external_robots_uri=settings.AivaRosBridgeOptions.Uri,
        )

    @provide(scope=Scope.RUNTIME)
    def get_config(self) -> AivaRosBridgeOptions:
        return self.config

    @provide(scope=Scope.APP)
    def get_robot_definition(self) -> RobotDefinition:
        return RobotDefinition(self.config.robot_ip)

    @provide(scope=Scope.APP)
    async def get_client(self) -> AsyncIterable[ExternalRobotsClient]:
        async with ExternalRobotsClient(self.config) as client:
            yield client


async def async_main():
    container = make_async_container(AivaRosBridgeProvider(scope=Scope.APP))
    external_robots_handler = await container.get(ExternalRobotsHandler)
    await external_robots_handler.process()


def main():
    rclpy.init()
    logging.basicConfig()

    node = AivaRosBridgeNode()

    asyncio.run(async_main())
    node.shutdown()


if __name__ == "__main__":
    main()
