from google.protobuf.internal import containers as _containers
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from typing import ClassVar as _ClassVar, Iterable as _Iterable, Mapping as _Mapping, Optional as _Optional, Union as _Union

DESCRIPTOR: _descriptor.FileDescriptor

class Empty(_message.Message):
    __slots__ = ()
    def __init__(self) -> None: ...

class RobotDefinitionMessage(_message.Message):
    __slots__ = ("ip_address",)
    IP_ADDRESS_FIELD_NUMBER: _ClassVar[int]
    ip_address: str
    def __init__(self, ip_address: _Optional[str] = ...) -> None: ...

class RobotDefinitionsMessage(_message.Message):
    __slots__ = ("definitions",)
    DEFINITIONS_FIELD_NUMBER: _ClassVar[int]
    definitions: _containers.RepeatedCompositeFieldContainer[RobotDefinitionMessage]
    def __init__(self, definitions: _Optional[_Iterable[_Union[RobotDefinitionMessage, _Mapping]]] = ...) -> None: ...

class RobotRequest(_message.Message):
    __slots__ = ("plan_request", "execute_request")
    PLAN_REQUEST_FIELD_NUMBER: _ClassVar[int]
    EXECUTE_REQUEST_FIELD_NUMBER: _ClassVar[int]
    plan_request: RobotPlanRequest
    execute_request: RobotExecuteRequest
    def __init__(self, plan_request: _Optional[_Union[RobotPlanRequest, _Mapping]] = ..., execute_request: _Optional[_Union[RobotExecuteRequest, _Mapping]] = ...) -> None: ...

class RobotResponse(_message.Message):
    __slots__ = ("success",)
    SUCCESS_FIELD_NUMBER: _ClassVar[int]
    success: bool
    def __init__(self, success: bool = ...) -> None: ...

class RobotPlanRequest(_message.Message):
    __slots__ = ("plan_name",)
    PLAN_NAME_FIELD_NUMBER: _ClassVar[int]
    plan_name: str
    def __init__(self, plan_name: _Optional[str] = ...) -> None: ...

class RobotExecuteRequest(_message.Message):
    __slots__ = ("plan_name",)
    PLAN_NAME_FIELD_NUMBER: _ClassVar[int]
    plan_name: str
    def __init__(self, plan_name: _Optional[str] = ...) -> None: ...
