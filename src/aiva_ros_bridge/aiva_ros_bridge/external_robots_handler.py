import asyncio
import logging

from aiva_ros_bridge.external_robots_client import ExternalRobotsClient
from aiva_ros_bridge.robot_definition import RobotDefinition

logger = logging.getLogger(__name__)


class ExternalRobotsHandler:
    def __init__(self, client: ExternalRobotsClient, robot_definition: RobotDefinition):
        self.client = client
        self.robot_definition = robot_definition

    async def process(self):
        async with asyncio.TaskGroup() as tg:
            tg.create_task(self.send_robots(self.client, self.robot_definition))
            tg.create_task(self.process_requests(self.client))

    async def process_requests(self, client: ExternalRobotsClient):
        while True:
            try:
                await client.get_requests()
            except Exception as e:
                logger.exception(e)
                await asyncio.sleep(1)

    async def send_robots(self, client: ExternalRobotsClient, robot: RobotDefinition):
        while True:
            try:
                await client.send_robots(robot)
            except Exception as e:
                logger.exception(e)

            await asyncio.sleep(5)
