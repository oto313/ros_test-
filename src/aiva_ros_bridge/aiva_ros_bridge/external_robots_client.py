import asyncio
import logging
from dataclasses import dataclass

import grpc
from proto.aiva_ros_bridge_pb2 import (
    RobotDefinitionMessage,
    RobotDefinitionsMessage,
    RobotResponse,
)
from proto.aiva_ros_bridge_pb2_grpc import AivaRosBridgeStub

from aiva_ros_bridge.aiva_ros_bridge_options import AivaRosBridgeOptions

logger = logging.getLogger(__name__)


@dataclass
class RobotDefinition:
    ip_address: str


class ExternalRobotsClient:
    def __init__(self, options: AivaRosBridgeOptions):
        self.external_robots_uri = options.external_robots_uri
        self.channel = None
        self.client = None

    async def request_generator(self, request_stream: asyncio.Queue):
        try:
            while True:
                request = await request_stream.get()
                if request is None:
                    break
                yield request
        except Exception as e:
            logger.exception(e)
            await asyncio.sleep(1)

    async def __aenter__(self):
        self.channel = grpc.aio.insecure_channel(self.external_robots_uri)
        await self.channel.__aenter__()
        self.client = AivaRosBridgeStub(self.channel)
        return self

    async def __aexit__(self, exc_type, exc_val, exc_tb):
        await self.channel.__aexit__(exc_type, exc_val, exc_tb)

    async def send_robots(self, robot: RobotDefinition):
        await self.client.SendRobots(
            RobotDefinitionsMessage(
                definitions=[RobotDefinitionMessage(ip_address=robot.ip_address)]
            )
        )

    async def get_requests(self):
        request_stream = asyncio.Queue()
        try:
            async for request in self.client.GetRequests(
                request_iterator=self.request_generator(request_stream),
                wait_for_ready=True,
            ):
                print(request.plan_request.plan_name)
                await request_stream.put(RobotResponse(success=True))
        finally:
            await request_stream.put(None)
