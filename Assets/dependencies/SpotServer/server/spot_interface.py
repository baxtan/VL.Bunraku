from state import State
from bosdyn.api import geometry_pb2
from bosdyn.api.robot_command_pb2 import RobotCommand
from choregraphy import Choreography, JumpMove
import os
from bosdyn import geometry
import bosdyn.api.power_pb2 as PowerServiceProto
import bosdyn.api.robot_state_pb2 as robot_state_proto
import bosdyn.api.basic_command_pb2 as basic_command_pb2
import bosdyn.api.spot.robot_command_pb2 as spot_command_pb2
from bosdyn.client import ResponseError, RpcError
from bosdyn.client.async_tasks import AsyncPeriodicQuery
from bosdyn.client.estop import EstopClient, EstopEndpoint, EstopKeepAlive
from bosdyn.client.lease import LeaseClient, LeaseKeepAlive
from bosdyn.client.lease import Error as LeaseBaseError
from bosdyn.client.power import PowerClient
from bosdyn.client.robot_command import RobotCommandClient, RobotCommandBuilder
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.client.time_sync import TimeSyncError
from bosdyn.client.frame_helpers import ODOM_FRAME_NAME
from bosdyn.util import duration_str, secs_to_hms
from bosdyn.geometry import EulerZXY
from bosdyn.choreography.client.choreography import ChoreographyClient
from bosdyn.api.spot import choreography_sequence_pb2
from bosdyn.choreography.client.choreography import ChoreographyClient, load_choreography_sequence_from_txt_file
from bosdyn.client.robot import Robot
from logger import LOGGER
import time

VELOCITY_BASE_SPEED = 0.5  # m/s
VELOCITY_BASE_ANGULAR = 0.8  # rad/sec
VELOCITY_CMD_DURATION = 0.6  # seconds


def _grpc_or_log(desc, thunk):
    try:
        return thunk()
    except (ResponseError, RpcError) as err:
        LOGGER.error("Failed %s: %s" % (desc, err))


class AsyncRobotState(AsyncPeriodicQuery):
    """Grab robot state."""

    def __init__(self, robot_state_client):
        super(AsyncRobotState, self).__init__("robot_state", robot_state_client, LOGGER,
                                              period_sec=0.2)

    def _start_query(self):
        return self._client.get_robot_state_async()


class SpotInterface(object):

    def __init__(self, robot:Robot, state:State):
        self._robot = robot
        self._state = state
        # Create clients -- do not use the for communication yet.
        self._lease_client = robot.ensure_client(LeaseClient.default_service_name)
        try:
            self._estop_client = self._robot.ensure_client(EstopClient.default_service_name)
            self._estop_endpoint = EstopEndpoint(self._estop_client, 'GNClient', 9.0)
        except:
            # Not the estop.
            print("Estop init failed")
            self._estop_client = None
            self._estop_endpoint = None
        self._power_client = robot.ensure_client(PowerClient.default_service_name)
        self._robot_state_client:RobotStateClient = robot.ensure_client(RobotStateClient.default_service_name)
        self._robot_command_client:RobotCommandClient = robot.ensure_client(RobotCommandClient.default_service_name)
        self._estop_keepalive = None
        self._exit_check = None
        self._choreo_client:ChoreographyClient = self._robot.ensure_client(ChoreographyClient.default_service_name)

        # Stuff that is set in start()
        self._robot_id = None
        self._lease = None
        self._lease_keepalive = None

    def start(self):
        """Begin communication with the robot."""
        self._lease = self._lease_client.acquire()
        # Construct our lease keep-alive object, which begins RetainLease calls in a thread.
        self._lease_keepalive = LeaseKeepAlive(self._lease_client)

        self._robot_id = self._robot.get_id()
        if self._estop_endpoint is not None:
            self._estop_endpoint.force_simple_setup(
            )  # Set this endpoint as the robot's sole estop.

    def shutdown(self):
        """Release control of robot as gracefully as posssible."""
        LOGGER.info("Shutting down Spot Interface.")
        if self._estop_keepalive:
            # This stops the check-in thread but does not stop the robot.
            self._estop_keepalive.shutdown()
        if self._lease:
            _grpc_or_log("returning lease", lambda: self._lease_client.return_lease(self._lease))
            self._lease = None

    def add_message(self, msg_text):
        """Display the given message string to the user in the curses interface."""
        print(msg_text)

    @property
    def robot_state(self):
        """Get latest robot state proto."""
        return self._robot_state_client.get_robot_state()
    
    def _try_grpc(self, desc, thunk):
        try:
            return thunk()
        except (ResponseError, RpcError, LeaseBaseError) as err:
            self.add_message("Failed {}: {}".format(desc, err))
            return None

    def _try_grpc_async(self, desc, thunk):
        def on_future_done(fut):
            try:
                fut.result()
            except (ResponseError, RpcError, LeaseBaseError) as err:
                self.add_message("Failed {}: {}".format(desc, err))
                return None
        future = thunk()
        future.add_done_callback(on_future_done)

    def _quit_program(self):
        self.sit()
        if self._exit_check is not None:
            self._exit_check.request_exit()

    def toggle_time_sync(self):
        if self._robot.time_sync.stopped:
            self._robot.start_time_sync()
        else:
            self._robot.time_sync.stop()

    def toggle_estop(self):
        """toggle estop on/off. Initial state is ON"""
        if self._estop_client is not None and self._estop_endpoint is not None:
            if not self._estop_keepalive:
                self._estop_keepalive = EstopKeepAlive(self._estop_endpoint)
            else:
                self._try_grpc("stopping estop", self._estop_keepalive.stop)
                self._estop_keepalive.shutdown()
                self._estop_keepalive = None
    
    def estop(self):
        if self._estop_endpoint is not None:
            self._estop_endpoint.stop()

    def _toggle_lease(self):
        """toggle lease acquisition. Initial state is acquired"""
        if self._lease_client is not None:
            if self._lease_keepalive is None:
                self._lease = self._lease_client.acquire()
                self._lease_keepalive = LeaseKeepAlive(self._lease_client)
            else:
                self._lease_client.return_lease(self._lease)
                self._lease_keepalive.shutdown()
                self._lease_keepalive = None

    def _start_robot_command(self, desc, command_proto, end_time_secs=None):
        def _start_command():
            self._state.last_command_id = self._robot_command_client.robot_command(
                lease=None, command=command_proto, end_time_secs=end_time_secs)
        self._try_grpc(desc, _start_command)

    def self_right(self):
        if self._state.is_in_stair_mode():
            return
        self._start_robot_command('self_right', RobotCommandBuilder.selfright_command())

    def _battery_change_pose(self):
        if self._state.is_in_stair_mode():
            return
        # Default HINT_RIGHT, maybe add option to choose direction?
        self._start_robot_command(
            'battery_change_pose',
            RobotCommandBuilder.battery_change_pose_command(dir_hint=
            basic_command_pb2.BatteryChangePoseCommand.Request.HINT_RIGHT))

    def sit(self):
        if self._state.is_in_stair_mode():
            return
        self._start_robot_command('sit', RobotCommandBuilder.synchro_sit_command())

    def stand(self, body_height: float=None, body_orientation: EulerZXY=None):
        if self._state.is_in_stair_mode():
            return
        if body_height is None:
            body_height = 0.0
        if body_orientation is None:
            body_orientation = geometry.EulerZXY()
        self._start_robot_command('stand', RobotCommandBuilder.synchro_stand_command(
            body_height=body_height,
            footprint_R_body=body_orientation
        ))

    def move_forward(self, velocity_speed: float = None, body_height: float = None):
        if velocity_speed is None:
            velocity_speed = VELOCITY_BASE_SPEED
        if self._state.is_in_stair_mode():
            if velocity_speed > 0.5:
                velocity_speed = VELOCITY_BASE_SPEED
            body_height = 0.0
        else:
            if body_height is None:
                body_height = 0.0
        self._velocity_cmd_helper('move_forward', v_x=velocity_speed, body_height=body_height)

    def move_backward(self, velocity_speed: float = None, body_height: float = None):
        if velocity_speed is None:
            velocity_speed = VELOCITY_BASE_SPEED
        if self._state.is_in_stair_mode():
            if velocity_speed > 0.5:
                velocity_speed = VELOCITY_BASE_SPEED
            body_height = 0.0
        else:
            if body_height is None:
                body_height = 0.0
        self._velocity_cmd_helper('move_backward', v_x=-velocity_speed, body_height=body_height)

    def move_left(self, velocity_speed: float = None, body_height: float = None):
        if self._state.is_in_stair_mode():
            return
        if velocity_speed is None:
            velocity_speed = VELOCITY_BASE_SPEED
        if body_height is None:
            body_height = 0.0
        self._velocity_cmd_helper('strafe_left', v_y=velocity_speed, body_height=body_height)

    def move_right(self, velocity_speed: float = None, body_height: float = None):
        if self._state.is_in_stair_mode():
            return
        if velocity_speed is None:
            velocity_speed = VELOCITY_BASE_SPEED
        if body_height is None:
            body_height = 0.0
        self._velocity_cmd_helper('strafe_right', v_y=-velocity_speed, body_height=body_height)

    def move(self, v_x=0.0, v_y=0.0, v_rot=0.0, body_height=0.0):
        if v_x is None:
            v_x = 0.0
        if v_y is None:
            v_y = 0.0
        if v_rot is None:
            v_rot = 0.0
        if body_height is None:
            body_height = 0.0
        self._velocity_cmd_helper('move', v_x=v_x, v_y=v_y, v_rot=v_rot, body_height=body_height)
  
    def turn_left(self, velocity_angular: float = None, body_height: float = None):
        if self._state.is_in_stair_mode():
            return
        if velocity_angular is None:
            velocity_angular = VELOCITY_BASE_ANGULAR
        if body_height is None:
            body_height = 0.0
        self._velocity_cmd_helper('turn_left', v_rot=velocity_angular, body_height=body_height)

    def turn_right(self, velocity_angular: float = None, body_height: float = None):
        if self._state.is_in_stair_mode():
            return
        if velocity_angular is None:
            velocity_angular = VELOCITY_BASE_ANGULAR
        if body_height is None:
            body_height = 0.0
        self._velocity_cmd_helper('turn_right', v_rot=-velocity_angular, body_height=body_height)

    def _velocity_cmd_helper(self, desc='', v_x=0.0, v_y=0.0, v_rot=0.0, body_height=0.0):
        if self._state.is_in_stair_mode():
            self._start_robot_command(desc,
                RobotCommandBuilder.synchro_velocity_command(
                    v_x=v_x, v_y=0.0, v_rot=0.0,
                    params=RobotCommandBuilder.mobility_params(
                        locomotion_hint=spot_command_pb2.HINT_TROT,
                        stair_hint=True)),
                end_time_secs=time.time() + VELOCITY_CMD_DURATION
            )
        else:
            self._start_robot_command(desc,
                RobotCommandBuilder.synchro_velocity_command(
                    v_x=v_x, v_y=v_y, v_rot=v_rot,
                    params=RobotCommandBuilder.mobility_params(
                        locomotion_hint=spot_command_pb2.HINT_AUTO,
                        stair_hint=False,
                        body_height=body_height)),
                end_time_secs=time.time() + VELOCITY_CMD_DURATION
            )

    def return_to_origin(self):
        if self._state.is_in_stair_mode():
            return
        self._start_robot_command('fwd_and_rotate',
                                  RobotCommandBuilder.synchro_se2_trajectory_point_command(
                                      goal_x=0.0, goal_y=0.0, goal_heading=0.0,
                                      frame_name=ODOM_FRAME_NAME, params=None, body_height=0.0,
                                      locomotion_hint=spot_command_pb2.HINT_SPEED_SELECT_TROT),
                                  end_time_secs=time.time() + 20)

    def toggle_power(self):
        power_state = self._power_state()
        if power_state is None:
            self.add_message('Could not toggle power because power state is unknown')
            return

        if power_state == robot_state_proto.PowerState.STATE_OFF:
            self._try_grpc_async("powering-on", self._request_power_on)
        else:
            self._try_grpc("powering-off", self._safe_power_off)

    def _request_power_on(self):
        request = PowerServiceProto.PowerCommandRequest.REQUEST_ON
        return self._power_client.power_command_async(request)

    def _safe_power_off(self):
        self._start_robot_command('safe_power_off', RobotCommandBuilder.safe_power_off_command())

    def _power_state(self):
        state = self.robot_state
        if not state:
            return None
        return state.power_state.motor_power_state

    def _lease_str(self, lease_keep_alive):
        alive = '??'
        lease = '??'
        if lease_keep_alive is None:
            alive = 'STOPPED'
            lease = 'RETURNED'
        else:
            if self._lease:
                lease = '{}:{}'.format(self._lease.lease_proto.resource,
                                       self._lease.lease_proto.sequence)
            else:
                lease = '...'
            if lease_keep_alive.is_alive():
                alive = 'RUNNING'
            else:
                alive = 'STOPPED'
        return 'Lease {} THREAD:{}'.format(lease, alive)

    def _power_state_str(self):
        power_state = self._power_state()
        if power_state is None:
            return ''
        state_str = robot_state_proto.PowerState.MotorPowerState.Name(power_state)
        return 'Power: {}'.format(state_str[6:])  # get rid of STATE_ prefix

    def _estop_str(self):
        if not self._estop_client:
            thread_status = 'NOT ESTOP'
        else:
            thread_status = 'RUNNING' if self._estop_keepalive else 'STOPPED'
        estop_status = '??'
        state = self.robot_state
        if state:
            for estop_state in state.estop_states:
                if estop_state.type == estop_state.TYPE_SOFTWARE:
                    estop_status = estop_state.State.Name(estop_state.state)[6:]  # s/STATE_//
                    break
        return 'Estop {} (thread: {})'.format(estop_status, thread_status)

    def _time_sync_str(self):
        if not self._robot.time_sync:
            return 'Time sync: (none)'
        if self._robot.time_sync.stopped:
            status = 'STOPPED'
            exception = self._robot.time_sync.thread_exception
            if exception:
                status = '{} Exception: {}'.format(status, exception)
        else:
            status = 'RUNNING'
        try:
            skew = self._robot.time_sync.get_robot_clock_skew()
            if skew:
                skew_str = 'offset={}'.format(duration_str(skew))
            else:
                skew_str = "(Skew undetermined)"
        except (TimeSyncError, RpcError) as err:
            skew_str = '({})'.format(err)
        return 'Time sync: {} {}'.format(status, skew_str)

    def _battery_str(self):
        if not self.robot_state:
            return ''
        battery_state = self.robot_state.battery_states[0]
        status = battery_state.Status.Name(battery_state.status)
        status = status[7:]  # get rid of STATUS_ prefix
        if battery_state.charge_percentage.value:
            bar_len = int(battery_state.charge_percentage.value) // 10
            bat_bar = '|{}{}|'.format('=' * bar_len, ' ' * (10 - bar_len))
        else:
            bat_bar = ''
        time_left = ''
        if battery_state.estimated_runtime:
            time_left = ' ({})'.format(secs_to_hms(battery_state.estimated_runtime.seconds))
        return 'Battery: {}{}{}'.format(status, bat_bar, time_left)

    def jump(self, move:JumpMove):
        if self._state.is_in_stair_mode():
            return
        choreography = Choreography("jump", moves=[move]).to_ChoreographySequence()
        self._choreo_client.upload_choreography(choreography, non_strict_parsing=True)

        # Start immediately
        client_start_time = time.time()
        # Specify the starting slice of the choreography. We will set this to slice=0 so that the routine begins at
        # the very beginning.
        start_slice = 0
        # Issue the command to the robot's choreography service.
        self._choreo_client.execute_choreography(
            choreography_name=choreography.name,
            client_start_time=client_start_time,
            choreography_starting_slice=start_slice)

        # Estimate how long the choreographed sequence will take, and sleep that long.
        total_choreography_slices = 0
        for move in choreography.moves:
            total_choreography_slices += move.requested_slices
        estimated_time_seconds = total_choreography_slices / choreography.slices_per_minute * 60.0
        time.sleep(estimated_time_seconds)

    def get_robot_metrics(self):
        return self._robot_state_client.get_robot_metrics()

    def get_last_command_feedback(self):
        if self._state is not None and self._state.last_command_id is not None:
            return self._robot_command_client.robot_command_feedback(self._state.last_command_id)