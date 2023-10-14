from __future__ import print_function
import argparse
from functools import partial
from http.server import HTTPServer
from spot_interface import SpotInterface
import os
from logger import setup_logging
from http_request_handler import HTTPRequestHandler
from bosdyn.client import create_standard_sdk, ResponseError, RpcError
import bosdyn.client.util
from bosdyn.client.robot import Robot
from bosdyn.choreography.client.choreography import ChoreographyClient
from state import State

def main():
    parser = argparse.ArgumentParser(description='Spot Server')
    parser.add_argument('--port', type=int, help='Listening port for HTTP Server')
    parser.add_argument('--ip', help='HTTP Server IP')
    parser.add_argument('--simulate', action='store_true', help="Doesn't make a connection to spot for testing purposes.")
    bosdyn.client.util.add_common_arguments(parser)
    parser.add_argument('--time-sync-interval-sec',
                        help='The interval (seconds) that time-sync estimate should be updated.',
                        type=float)

    args = parser.parse_args()
    state = State()

    if args.simulate:
        print("Started in simulation mode...")
        state.log_http_requests = True
    setup_logging(args.verbose)

    if not args.simulate:
        # Create robot object.
        sdk = create_standard_sdk('Spot Client')
        try:
            sdk.register_service_client(ChoreographyClient)
        except:
            print("Could not register ChoreographyClient. License might have been expired.")
        robot:Robot = sdk.create_robot(args.hostname)
        try:
            robot.authenticate(args.username, args.password)
            robot.start_time_sync(args.time_sync_interval_sec)
        except RpcError as err:
            print("Failed to communicate with robot: %s" % err)
            return False

        spot = SpotInterface(robot, state)
        try:
            spot.start()
            spot.toggle_estop()
            spot.toggle_power()
        except (ResponseError, RpcError) as err:
            print("Failed to initialize robot communication: %s" % err)
            return False
    else:
        spot = None

    try:
        handler = partial(HTTPRequestHandler, spot, state)
        server = HTTPServer((args.ip, args.port), handler)
        print("HTTP Server running on http://"+ args.ip + ":" + str(args.port))
        server.serve_forever()
    finally:
        print('Shutting down...')
        if not args.simulate:
            spot.shutdown()
    
    return True


if __name__ == '__main__':
    if not main():
        os._exit(1)
    os._exit(0)