import asyncio
import websockets
import json
import socket
import time
import math

from bosdyn.client import create_standard_sdk
from bosdyn.client.power import PowerClient
from bosdyn.client.robot_command import RobotCommandBuilder
from bosdyn.client.lease import LeaseKeepAlive, add_lease_wallet_processors
from bosdyn.client.frame_helpers import ODOM_FRAME_NAME
from bosdyn.geometry import EulerZXY
from google.protobuf.json_format import MessageToDict

# Track any running UDP-stream tasks per robot alias
udp_stream_tasks = {}
robots = {}

class SpotUnit:
    def __init__(self, ip: str, username: str, password: str):
        # Initialise SDK, authenticate & time-sync
        sdk = create_standard_sdk("BunrakuWSClient")
        self.robot = sdk.create_robot(ip)
        self.robot.authenticate(username, password)
        self.robot.time_sync.wait_for_sync()

        # Power client (manual control)
        self.power_client: PowerClient = self.robot.ensure_client('power')

        # Acquire a lease and keep it alive
        lease_client = self.robot.ensure_client('lease')
        try:
            self.lease = lease_client.acquire()
        except Exception:
            self.lease = lease_client.take()
        self.lease_keep_alive = LeaseKeepAlive(lease_client)

        # Robot-command client with automatic lease handling
        self.cmd_client = self.robot.ensure_client('robot-command')
        add_lease_wallet_processors(self.cmd_client, lease_client.lease_wallet)

        # State client
        self.state_client = self.robot.ensure_client('robot-state')

    def power_on(self, timeout: float = 20.0) -> dict:
        self.robot.power_on(timeout_sec=timeout)
        if not self.robot.is_powered_on():
            return {"status": "error", "msg": f"Power-on failed within {timeout}s"}
        return {"status": "ok", "msg": f"Powered on (timeout={timeout}s)"}

    def power_off(self, timeout: float = 10.0) -> dict:
        self.robot.power_off(timeout_sec=timeout)
        return {"status": "ok", "msg": f"Powered off (timeout={timeout}s)"}

    def self_right(self) -> dict:
        cmd_id = self.cmd_client.robot_command(
            RobotCommandBuilder.selfright_command()
        )
        return {"status": "ok", "msg": f"Self-righting (cmd_id={cmd_id})"}

    def stand(self,
              body_height: float = 0.0,
              yaw: float = 0.0,
              roll: float = 0.0,
              pitch: float = 0.0) -> dict:
        # Build an EulerZXY orientation for the body frame
        footprint_R_body = EulerZXY(yaw=yaw, roll=roll, pitch=pitch)
        cmd = RobotCommandBuilder.synchro_stand_command(
            body_height=body_height,
            footprint_R_body=footprint_R_body
        )
        cmd_id = self.cmd_client.robot_command(cmd)
        return {
            "status": "ok",
            "msg":    (f"Standing at {body_height:.2f} m "
                       f"(yaw={yaw:.2f}, roll={roll:.2f}, pitch={pitch:.2f}; "
                       f"cmd_id={cmd_id})")
        }

    def sit(self) -> dict:
        cmd_id = self.cmd_client.robot_command(
            RobotCommandBuilder.synchro_sit_command()
        )
        return {"status": "ok", "msg": f"Sitting down (cmd_id={cmd_id})"}

    def move(self,
             v_x: float, v_y: float, v_rot: float,
             body_height: float = 0.0,
             duration: float = 2.0) -> dict:
        params = RobotCommandBuilder.mobility_params(body_height=body_height)
        cmd = RobotCommandBuilder.synchro_velocity_command(
            v_x=v_x, v_y=v_y, v_rot=v_rot, params=params
        )
        end_time = time.time() + duration
        cmd_id = self.cmd_client.robot_command(cmd, end_time_secs=end_time)
        return {
            "status": "ok",
            "msg":    (f"Moving vx={v_x:.2f}, vy={v_y:.2f}, "
                       f"vrot={v_rot:.2f} for {duration:.2f}s "
                       f"(cmd_id={cmd_id})")
        }

    def go_to(self,
              x: float, y: float, yaw: float,
              duration: float = 10.0) -> dict:
        cmd = RobotCommandBuilder.synchro_se2_trajectory_point_command(
            goal_x=x,
            goal_y=y,
            goal_heading=yaw,
            frame_name=ODOM_FRAME_NAME
        )
        end_time = time.time() + duration
        cmd_id = self.cmd_client.robot_command(cmd, end_time_secs=end_time)
        return {
            "status": "ok",
            "msg":    (f"Going to ({x:.2f}, {y:.2f}, yaw={yaw:.2f}) "
                       f"over {duration:.2f}s (cmd_id={cmd_id})")
        }

    def follow_path(self,
                    path: list,
                    duration_per_leg: float = 10.0) -> dict:
        for pt in path:
            self.go_to(pt['x'], pt['y'], pt['yaw'], duration=duration_per_leg)
            time.sleep(0.1)
        return {"status": "ok",
                "msg":    f"Followed {len(path)} waypoints (≈{duration_per_leg}s each)"}

    def get_battery(self) -> dict:
        b = self.state_client.get_robot_state().battery_states[0]
        return {"status": "ok",
                "voltage": b.voltage_v,
                "percent": b.charge_percentage}

    def get_faults(self) -> dict:
        fs = self.state_client.get_robot_state().system_faults
        return {"status": "ok", "faults": [f.text for f in fs.active_faults]}

    def release_lease(self) -> dict:
        self.lease_client.return_lease(self.lease)
        return {"status": "ok", "msg": "Lease returned"}

    def time_sync(self) -> dict:
        self.robot.time_sync.wait_for_sync()
        return {"status": "ok", "msg": "Time re-synchronised"}

    def get_state(self) -> dict:
        state = self.state_client.get_robot_state()
        return {"status": "ok", "state": MessageToDict(state)}

    def get_pose(self) -> dict:
        """
        Return the current (x, y, yaw) by finding any transform edge
        whose key contains 'body' (e.g. 'body_T_odom', 'vision_T_body', etc.).
        """
        state_dict = MessageToDict(self.state_client.get_robot_state())
        edge_map   = state_dict['kinematicState']['transformsSnapshot']['childToParentEdgeMap']

        # Pick the first key that mentions 'body'
        body_key = next((k for k in edge_map if 'body' in k), None)
        if not body_key:
            return {"status":"error", "msg":"No body-frame transform found"}

        edge  = edge_map[body_key]
        trans = edge['transform']['translation']
        rot   = edge['transform']['rotation']
        x, y  = trans['x'], trans['y']

        # Compute yaw from quaternion
        yaw = math.atan2(
            2*(rot['w']*rot['z'] + rot['x']*rot['y']),
            1 - 2*(rot['y']**2 + rot['z']**2)
        )
        return {"status":"ok", "x": x, "y": y, "yaw": yaw}
    
async def udp_state_stream_loop(alias: str,
                                sock: socket.socket,
                                target: tuple,
                                interval: float):
    spot = robots[alias]
    try:
        while True:
            rd = spot.get_state()
            payload = json.dumps(rd["state"]).encode('utf-8')
            sock.sendto(payload, target)
            await asyncio.sleep(interval)
    except asyncio.CancelledError:
        pass

async def handler(websocket):
    async for message in websocket:
        try:
            data = json.loads(message)
            cmd = data.get("cmd")
            alias = data.get("robot", "default")

            if cmd == "connect":
                robots[alias] = SpotUnit(
                    ip=data["ip"],
                    username=data["username"],
                    password=data["password"]
                )
                await websocket.send(json.dumps({"status":"ok","msg":f"Connected to {alias}"}))
                continue

            if alias not in robots:
                await websocket.send(json.dumps({"status":"error","msg":f"Robot '{alias}' not found"}))
                continue

            spot = robots[alias]

            # UDP state streaming
            if cmd == "start_udp_stream":
                prev = udp_stream_tasks.pop(alias, None)
                if prev:
                    prev[0].cancel(); prev[1].close()
                ip = data["target_ip"]; port = data["target_port"]; iv = data.get("interval", 0.5)
                sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                task = asyncio.create_task(udp_state_stream_loop(alias, sock, (ip,port), iv))
                udp_stream_tasks[alias] = (task, sock)
                resp = {"status":"ok","msg":f"UDP stream → {ip}:{port} @{iv}s"}

            elif cmd == "stop_udp_stream":
                prev = udp_stream_tasks.pop(alias, None)
                if prev:
                    prev[0].cancel(); prev[1].close(); resp = {"status":"ok","msg":"UDP stream stopped"}
                else:
                    resp = {"status":"error","msg":"No UDP stream"}

            # Power & lease
            elif cmd == "power_on":        resp = spot.power_on(data.get("timeout",20.0))
            elif cmd == "power_off":       resp = spot.power_off(data.get("timeout",10.0))
            elif cmd == "self_right":      resp = spot.self_right()
            elif cmd == "release_lease":   resp = spot.release_lease()
            elif cmd == "time_sync":       resp = spot.time_sync()

            # Motion / navigation
            elif cmd == "stand":           resp = spot.stand(
                                              body_height=data.get("bodyHeight",0.0),
                                              yaw=data.get("orientation",{}).get("yaw",0.0),
                                              roll=data.get("orientation",{}).get("roll",0.0),
                                              pitch=data.get("orientation",{}).get("pitch",0.0)
                                          )
            elif cmd == "sit":             resp = spot.sit()
            elif cmd == "move":            resp = spot.move(
                                              data["vX"],data["vY"],data["vRot"],
                                              data.get("bodyHeight",0.0),data.get("duration",2.0)
                                          )
            elif cmd == "go_to":           resp = spot.go_to(
                                              data["x"],data["y"],data["yaw"],
                                              duration=data.get("duration",10.0)
                                          )
            elif cmd == "follow_path":     resp = spot.follow_path(
                                              data.get("path",[]),
                                              duration_per_leg=data.get("duration_per_leg",10.0)
                                          )

            # Queries
            elif cmd == "state":          resp = spot.get_state()
            elif cmd == "get_pose":       resp = spot.get_pose()
            elif cmd == "get_battery":    resp = spot.get_battery()
            elif cmd == "get_faults":     resp = spot.get_faults()

            else:                         resp = {"status":"error","msg":"Unknown command"}

            await websocket.send(json.dumps(resp))
        except Exception as e:
            await websocket.send(json.dumps({"status":"error","msg":str(e)}))

async def main():
    print("Starting bridge on ws://localhost:8765")
    async with websockets.serve(handler, "0.0.0.0", 8765):
        await asyncio.Future()

if __name__ == "__main__":
    asyncio.run(main())