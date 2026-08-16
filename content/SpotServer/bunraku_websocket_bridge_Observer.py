import asyncio
import json
import websockets

from bosdyn.client import create_standard_sdk
from bosdyn.client.robot_command import RobotCommandBuilder
from bosdyn.client.lease import LeaseKeepAlive
from google.protobuf.json_format import MessageToDict


# ------------------------------------------------------------
# Spot wrapper
# ------------------------------------------------------------

class SpotUnit:
    def __init__(self, ip, username, password, observe_only=False):
        self.observe_only = observe_only

        sdk = create_standard_sdk("BunrakuWSClient")
        self.robot = sdk.create_robot(ip)
        self.robot.authenticate(username, password)
        self.robot.time_sync.wait_for_sync()

        # Always allowed
        self.state_client = self.robot.ensure_client("robot-state")

        # Only created if we want control
        self.cmd_client = None
        self.lease_client = None
        self.lease = None
        self.lease_keep_alive = None

        if not observe_only:
            self.lease_client = self.robot.ensure_client("lease")

            try:
                self.lease = self.lease_client.acquire()
            except Exception:
                # Tablet or other client may already hold it
                self.lease = self.lease_client.take()

            self.lease_keep_alive = LeaseKeepAlive(self.lease_client)
            self.cmd_client = self.robot.ensure_client("robot-command")

    # -----------------------
    # Commands (control only)
    # -----------------------

    def _ensure_control(self):
        if self.observe_only or self.cmd_client is None:
            raise RuntimeError("Robot is in observe-only mode")

    def sit(self):
        self._ensure_control()
        cmd = RobotCommandBuilder.synchro_sit_command()
        self.cmd_client.robot_command(self.lease, cmd)
        return {"status": "ok", "msg": "Sit command sent"}

    def stand(self, body_height=0.0):
        self._ensure_control()
        cmd = RobotCommandBuilder.synchro_stand_command(body_height=body_height)
        self.cmd_client.robot_command(self.lease, cmd)
        return {"status": "ok", "msg": f"Stand command sent (height={body_height})"}

    def move(self, v_x, v_y, v_rot, body_height=0.0):
        self._ensure_control()
        params = RobotCommandBuilder.mobility_params(body_height=body_height)
        cmd = RobotCommandBuilder.synchro_velocity_command(
            v_x=v_x,
            v_y=v_y,
            v_rot=v_rot,
            params=params,
        )
        self.cmd_client.robot_command(self.lease, cmd)
        return {"status": "ok", "msg": "Velocity command sent"}

    # -----------------------
    # Observation (always OK)
    # -----------------------

    def get_state(self):
        state = self.state_client.get_robot_state()
        return {
            "status": "ok",
            "observe_only": self.observe_only,
            "state": MessageToDict(state),
        }


# ------------------------------------------------------------
# WebSocket server
# ------------------------------------------------------------

robots = {}


async def handler(websocket):
    async for message in websocket:
        try:
            data = json.loads(message)
            cmd = data.get("cmd")
            alias = data.get("robot", "default")

            # -----------------------
            # Connect
            # -----------------------
            if cmd == "connect":
                observe_only = data.get("observe", False)

                robots[alias] = SpotUnit(
                    ip=data["ip"],
                    username=data["username"],
                    password=data["password"],
                    observe_only=observe_only,
                )

                await websocket.send(json.dumps({
                    "status": "ok",
                    "robot": alias,
                    "observe_only": observe_only
                }))
                continue

            if alias not in robots:
                await websocket.send(json.dumps({
                    "status": "error",
                    "msg": f"Robot '{alias}' not connected"
                }))
                continue

            robot = robots[alias]

            # -----------------------
            # Commands
            # -----------------------
            if cmd == "sit":
                resp = robot.sit()

            elif cmd == "stand":
                resp = robot.stand(data.get("bodyHeight", 0.0))

            elif cmd == "move":
                resp = robot.move(
                    data["vX"],
                    data["vY"],
                    data["vRot"],
                    data.get("bodyHeight", 0.0),
                )

            elif cmd == "state":
                resp = robot.get_state()

            else:
                resp = {"status": "error", "msg": f"Unknown command '{cmd}'"}

            await websocket.send(json.dumps(resp))

        except Exception as e:
            await websocket.send(json.dumps({
                "status": "error",
                "msg": str(e)
            }))


async def main():
    print("Bunraku Spot WebSocket bridge running on ws://localhost:8765")
    async with websockets.serve(handler, "0.0.0.0", 8765):
        await asyncio.Future()


if __name__ == "__main__":
    asyncio.run(main())
