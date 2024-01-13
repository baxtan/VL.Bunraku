from __future__ import annotations
from choregraphy import JumpMove, Vector2d
from http.server import BaseHTTPRequestHandler
import json
from state import State, MOVEMENT_MODE_DEFAULT, MOVEMENT_MODE_STAIR
import threading
from bosdyn.geometry import EulerZXY
from spot_interface import SpotInterface
from google.protobuf.json_format import MessageToJson


class HTTPRequestHandler(BaseHTTPRequestHandler):

    def __init__(self, spot: SpotInterface, state: State, *args, **kwargs):
        self._spot = spot
        self._state = state
        self.parser = SpotCommandJsonParser(self, state)
        super().__init__(*args, **kwargs)
    
    def do_POST(self):
        if self._state.log_http_requests:
            print("Received HTTP POST request '" + self.path + "'")

        if self.path == '/api/test':
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.end_headers()
            data = json.dumps({ 'message': 'HTTP Post request is working'})
            self.wfile.write(data.encode('utf8'))

        elif self.path == '/api/shutdown':
            # Must shutdown in another thread or we'll hang
            def kill_me_please():
                self.server.shutdown()
            threading.Thread(target=kill_me_please).start()

            # Send out a 200 before we go
            self.send_response(200)

        elif self.path == '/api/commands/move_forward':
            try:
                data = self.parser.read()
                def command():
                    if self._spot is not None:
                        self._spot.move_forward(
                            data.get_property('velocity_speed').asFloat().asAbs().value,
                            data.get_property('body_height').asFloat().value
                        )
                threading.Thread(target=command).start()
                self.send_response(200)
            except:
                self.send_response(400)

        elif self.path == '/api/commands/move_backward':
            try:
                data = self.parser.read()
                def command():
                    if self._spot is not None:
                        self._spot.move_backward(
                            data.get_property('velocity_speed').asFloat().asAbs().value,
                            data.get_property('body_height').asFloat().value
                        )
                threading.Thread(target=command).start()
                self.send_response(200)
            except:
                self.send_response(400)

        elif self.path == '/api/commands/move_left':
            try:
                data = self.parser.read()
                def command():
                    if self._spot is not None:
                        self._spot.move_left(
                            data.get_property('velocity_speed').asFloat().asAbs().value,
                            data.get_property('body_height').asFloat().value
                        )
                threading.Thread(target=command).start()
                self.send_response(200)
            except:
                self.send_response(400)

        elif self.path == '/api/commands/move_right':
            try:
                data = self.parser.read()
                def command():
                    if self._spot is not None:
                        self._spot.move_right(
                            data.get_property('velocity_speed').asFloat().asAbs().value,
                            data.get_property('body_height').asFloat().value
                        )
                threading.Thread(target=command).start()
                self.send_response(200)
            except:
                self.send_response(400)

        elif self.path == '/api/commands/move':
            try:
                data = self.parser.read()
                def command():
                    if self._spot is not None:
                        self._spot.move(
                            data.get_property('v_x').asFloat().asFloat().value,
                            data.get_property('v_y').asFloat().asFloat().value,
                            data.get_property('v_rot').asFloat().asFloat().value,
                            data.get_property('body_height').asFloat().value
                        )
                threading.Thread(target=command).start()
                self.send_response(200)
            except:
                self.send_response(400)

        elif self.path == '/api/commands/turn_left':
            try:
                data = self.parser.read()
                def command():
                    if self._spot is not None:
                        self._spot.turn_left(
                            data.get_property('velocity_angular').asFloat().asAbs().value,
                            data.get_property('body_height').asFloat().value
                        )
                threading.Thread(target=command).start()
                self.send_response(200)
            except:
                self.send_response(400)
                
        elif self.path == '/api/commands/turn_right':
            try:
                data = self.parser.read()
                def command():
                    if self._spot is not None:
                        self._spot.turn_right(
                            data.get_property('velocity_angular').asFloat().asAbs().value,
                            data.get_property('body_height').asFloat().value
                        )
                threading.Thread(target=command).start()
                self.send_response(200)
            except:
                self.send_response(400)

        elif self.path == '/api/commands/self_right':
            def command():
                if self._spot is not None:
                    self._spot.self_right()
            threading.Thread(target=command).start()
            self.send_response(200)

        elif self.path == '/api/commands/sit':
            def command():
                if self._spot is not None:
                    self._spot.sit()
            threading.Thread(target=command).start()
            self.send_response(200)

        elif self.path == '/api/commands/return_to_origin':
            def command():
                if self._spot is not None:
                    self._spot.return_to_origin()
            threading.Thread(target=command).start()
            self.send_response(200)            

        elif self.path == '/api/commands/stand':
            data = self.parser.read()
            orientation = data.get_object('body_orientation').to_EulerZXY()
            def command():
                if self._spot is not None:
                    self._spot.stand(
                        data.get_property('body_height').asFloat().value,
                        orientation
                    )
            threading.Thread(target=command).start()
            self.send_response(200)

        elif self.path == '/api/power/toggle':
            def command():
                if self._spot is not None:
                    self._spot.toggle_power()
            threading.Thread(target=command).start()
            self.send_response(200)

        elif self.path == '/api/commands/timesync':
            def command():
                if self._spot is not None:
                    self._spot.toggle_time_sync()
            threading.Thread(target=command).start()
            self.send_response(200)

        elif self.path == '/api/estop/toggle':
            def command():
                if self._spot is not None:
                    self._spot.toggle_estop()
            threading.Thread(target=command).start()
            self.send_response(200)

        elif self.path == '/api/estop/cut_power':
            def command():
                if self._spot is not None:
                    self._spot.estop()
            threading.Thread(target=command).start()
            self.send_response(200)

        elif self.path == '/api/logging/http/toggle':
            self._state.log_http_requests = not self._state.log_http_requests
            self.send_response(200)
        
        elif self.path == '/api/commands/jump':
            data = self.parser.read()
            jump_move = data.get_object('parameters').to_JumpMove()
            if self._spot is not None:
                self._spot.jump(jump_move)
            self.send_response(200)

        elif self.path == '/api/mobility':
            data = self.parser.read()
            mode:str = data.get_property('mode').value
            if mode is None:
                self.send_response(400)
            elif mode.casefold() == MOVEMENT_MODE_STAIR.casefold():
                self._state.movement.set_to_stairs_mode()
                self.send_response(200)
            elif mode.casefold() == MOVEMENT_MODE_DEFAULT.casefold():
                self._state.movement.set_to_default_mode()
                self.send_response(200)
            else:
                self._state.movement.set_to_default_mode()
                self.send_response(400)

        else:
            # HTTP 403: not found
            self.send_response(404)

        self.end_headers()

    def do_GET(self):
        if self._state.log_http_requests:
            print("Received HTTP GET request '" + self.path + "'")
        
        if self.path == '/api/test':    
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.end_headers()
            data = json.dumps({ 'message': 'HTTP Get request is working'})
            self.wfile.write(data.encode('utf8'))
        
        elif self.path == '/api/robot/state':
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.end_headers()
            if self._spot is None:
                data = json.dumps({})
            else:
                data = MessageToJson(self._spot.robot_state)
            self.wfile.write(data.encode('utf8'))

        elif self.path == '/api/robot/metrics':
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.end_headers()
            if self._spot is None:
                data = json.dumps({})
            else:
                data = MessageToJson(self._spot.robot_state)
            self.wfile.write(data.encode('utf8'))

        elif self.path == '/api/commands/feedback/last':
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.end_headers()
            if self._spot is None:
                data = json.dumps({})
            else:
                feedback = self._spot.get_last_command_feedback()
                if feedback is None:
                    data = json.dumps({})
                else:
                    data = MessageToJson(feedback)
            self.wfile.write(data.encode('utf8'))
        
        elif self.path == '/api/mobility':            
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.end_headers()
            data = json.dumps({
                "mode": self._state.movement.mode
            })
            self.wfile.write(data.encode('utf8'))
        
        else:
            self.send_response(404)

        self.end_headers()

    def log_message(self, format: str, *args) -> None:
        if self._state.log_http_requests:
            super().log_message(format, *args)

class JsonProperty:
    def __init__(self, value = None) -> None:
        self.value = value

    def asFloat(self) -> JsonProperty:
        if self.value is None:
            return JsonProperty()
        return JsonProperty(float(self.value))

    def asBool(self) -> JsonProperty:
        if self.value is None:
            return JsonProperty()
        return JsonProperty(bool(self.value))

    def asAbs(self) -> JsonProperty:
        if self.value is None:
            return JsonProperty()
        return JsonProperty(abs(self.value))

    def value(self):
        return self.value

class JsonObject:
    def __init__(self, data = None) -> None:
        self.data = data

    def get_property(self, name: str) -> JsonProperty:
        try:
            return JsonProperty(self.data[name])
        except:
            return JsonProperty()

    def get_object(self, name: str) -> JsonObject:
        try:
            return JsonObject(self.data[name])
        except:
            return JsonObject()

    def to_EulerZXY(self) -> EulerZXY:
        if self.data is None:
            return EulerZXY()
        yaw = self.get_property('yaw').asFloat().value
        roll = self.get_property('roll').asFloat().value
        pitch = self.get_property('pitch').asFloat().value
        if yaw is None:
            yaw = 0.0
        if roll is None:
            roll = 0.0
        if pitch is None:
            pitch = 0.0
        return EulerZXY(yaw, roll, pitch)

    def to_Vector2d(self) -> Vector2d:
        if self.data is None:
            return Vector2d()
        return Vector2d(
            x=self.get_property('x').asFloat().value,
            y=self.get_property('y').asFloat().value
        )

    def to_JumpMove(self) -> JumpMove:
        if self.data is None:
            return JumpMove()
        return JumpMove(
            yaw=self.get_property('yaw').asFloat().value,
            flight_slices=self.get_property('flight_slices').asFloat().value,
            stance_width=self.get_property('stance_width').asFloat().value,
            stance_length=self.get_property('stance_length').asFloat().value,
            absolute=self.get_property('absolute').asBool().value,
            translation=self.get_object('translation').to_Vector2d(),
            split_fraction=self.get_property('split_fraction').asFloat().value
        )

class SpotCommandJsonParser:
    def __init__(self, handler: HTTPRequestHandler, state: State) -> None:
        self._handler = handler
        self._state = state

    def read(self) -> JsonObject:
        return JsonObject(self._read_body())

    def _read_body(self):
        content_length = int(self._handler.headers['Content-Length'])
        if content_length > 0:
            raw_data = self._handler.rfile.read(content_length)
            data = json.loads(raw_data)
            if self._state.log_http_requests:
                print(data)
            return data
        else:
            print({})
        return None
