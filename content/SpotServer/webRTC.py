import asyncio
import cv2
import av
import urllib3

from bosdyn.client import create_standard_sdk
from bosdyn.client.util import authenticate
from bosdyn.client.spot_cam.webrtc_client import SpotCamWebRTCClient

# suppress HTTPS warnings (Spot uses self-signed certs)
urllib3.disable_warnings(urllib3.exceptions.InsecureRequestWarning)

ROBOT_IP = "192.168.8.103"
USERNAME = "admin"
PASSWORD = "vk0isdc2bv0n"


async def main():
    sdk = create_standard_sdk("SpotCAM-WebRTC")
    robot = sdk.create_robot(ROBOT_IP)
    authenticate(robot, USERNAME, PASSWORD)

    # create Spot CAM WebRTC client
    webrtc_client = SpotCamWebRTCClient(robot)

    # start streaming video frames
    async for frame in webrtc_client.video_frames():
        img = frame.to_ndarray(format="bgr24")
        cv2.imshow("Spot CAM", img)
        cv2.waitKey(1)


if __name__ == "__main__":
    asyncio.run(main())
