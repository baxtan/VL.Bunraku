from typing import List
from bosdyn.api.spot.choreography_sequence_pb2 import ChoreographySequence, MoveParams
from bosdyn.api.spot.choreography_params_pb2 import JumpParams
from bosdyn.api.geometry_pb2 import Vec2Value
from google.protobuf.wrappers_pb2 import BoolValue, DoubleValue

class MoveParamsWrapperInterface:
    def to_MoveParams(self) -> MoveParams:
        return MoveParams()


class Vector2d:
    def __init__(self, x:float=None, y:float=None) -> None:
        self._x = x if x else 0.0
        self._y = y if y else 0.0

    def to_Vec2Value(self) -> Vec2Value:
        return Vec2Value(
            x=DoubleValue(value=self._x), 
            y=DoubleValue(value=self._y)
        )


class JumpMove(MoveParamsWrapperInterface):
    def __init__(self, 
            yaw:float=None,
            flight_slices:float=None,
            stance_width:float=None,
            stance_length:float=None,
            absolute:bool=None,
            translation:Vector2d=None,
            split_fraction:float=None) -> None:
        self._yaw = yaw if yaw else 0.0
        self._flight_slices = flight_slices if flight_slices else 3.0
        self._stance_width = stance_width if stance_width else 0.4
        self._stance_length = stance_length if stance_length else 0.6
        self._absolute = absolute if absolute else False
        self._translation = translation if translation else Vector2d()
        self._split_fraction = split_fraction if split_fraction else 0.0

    def to_MoveParams(self) -> MoveParams:
        return MoveParams(
            type="jump", 
            requested_slices = 5,
            jump_params=JumpParams(
                yaw=DoubleValue(value=self._yaw),
                flight_slices=DoubleValue(value=self._flight_slices),
                stance_width=DoubleValue(value=self._stance_width),
                stance_length=DoubleValue(value=self._stance_length),
                absolute=BoolValue(value=self._absolute),
                translation=self._translation.to_Vec2Value(),
                split_fraction=DoubleValue(value=self._split_fraction),
                lead_leg_pair="LEAD_AUTO"
            )
        )


class Choreography:
    def __init__(
            self, name:str, 
            slices_per_minute=520, 
            moves:List[MoveParamsWrapperInterface]=[]) -> None:
        self._name = name
        self._slices_per_minute = slices_per_minute
        self._moves = moves

    def to_ChoreographySequence(self) -> ChoreographySequence:
        return ChoreographySequence(
            name=self._name,
            slices_per_minute = self._slices_per_minute,
            moves = self._convert_moves()
        )
    
    def _convert_moves(self) -> List[MoveParams]:
        return [m.to_MoveParams() for m in self._moves]
