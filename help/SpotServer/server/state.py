from bosdyn.client.robot_command import RobotCommandBuilder
import bosdyn.api.spot.robot_command_pb2 as spot_command_pb2

MOVEMENT_MODE_DEFAULT:str = "DEFAULT"
MOVEMENT_MODE_STAIR:str = "STAIR"

class State:
    def __init__(
            self, 
            log_http_requests:bool = False) -> None:
        self.log_http_requests = log_http_requests
        self.last_command_id = None
        self.movement = SpotMovementState()
    
    def is_in_stair_mode(self) -> bool:
        return self.movement.mode == MOVEMENT_MODE_STAIR
    
class SpotMovementState:
    def __init__(self) -> None:
        self.mode = MOVEMENT_MODE_DEFAULT

    def set_to_stairs_mode(self):
        self.mode = MOVEMENT_MODE_STAIR

    def set_to_default_mode(self):
        self.mode = MOVEMENT_MODE_DEFAULT