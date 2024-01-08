'''
pyWebRTC - ROS2 node to publish and subscribe to WebRTC events
  Based on aiortc examples - especially webcam.py

Build (in the docker)

cd /workspaces/isaac_ros-dev
colcon build --symlink-install --packages-select pywebrtc
source install/setup.bash
ros2 run pywebrtc websrvr

*then realsense in another terminal*
source install/setup.bash
ros2 launch realsense2_camera rs_launch.py

*test if realsense seems off...*
realsense-viewer

*that, and use a third window to look at published topics - make sure all are there*
'''


# aiortc imports
import argparse     # eventually get rid of this

import asyncio
import json
import logging
import os
import platform
import ssl

import math
import cv2
import numpy

from aiohttp import web
from aiortc import RTCPeerConnection, RTCSessionDescription, VideoStreamTrack
from aiortc.contrib.media import MediaPlayer, MediaRelay
from aiortc.rtcrtpsender import RTCRtpSender

from av import VideoFrame

# ros2 imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import String     # for carrying json to/from the browser - ROS2 
from cv_bridge import CvBridge
from sensor_msgs.msg import Image   # image, internal to ROS2 (and eventually published to browser via WebRTC)

import threading



####################################
# aiortc - mainly from aiortc, examples/webcam.py
####################################

ROOT = os.path.dirname(__file__)
height, width = 480, 640
ros2vst = None      # this gets set when we're valid

class ROS2VideoStreamTrack(VideoStreamTrack):
    """
    A video track returns the last set image

    Will show gray if not set
    t.b.d. or if no images within N seconds
    
    """

    def __init__(self):
        super().__init__()  # don't forget this!
        global ros2vst
        ros2vst = self

        self.greyImg = numpy.zeros((height, width, 3), numpy.uint8)
        self.greyImg[:, :] = (64, 64, 64)        # gray
        self.currentImg = self.greyImg


    def setImg(self, img):
        self.currentImg = img       # hopefully atomically...

    async def recv(self):
        img = self.currentImg
        frame = VideoFrame.from_ndarray(img,format="bgr24")
        pts, time_base = await self.next_timestamp()
        frame.pts = pts
        frame.time_base = time_base
        return frame

    def __del__(self):
        global ros2vst
        ros2vst = None
        self.currentImg = self.greyImg

## Now, some boiler plate from webcam.py

relay = None
webcam = None

# this is modified - we're only using a video track - defined above
def create_local_tracks(play_from, decode):
    global relay, webcam, vidStream
    vidStream = ROS2VideoStreamTrack()
    return None,vidStream

def force_codec(pc, sender, forced_codec):
    kind = forced_codec.split("/")[0]
    codecs = RTCRtpSender.getCapabilities(kind).codecs
    transceiver = next(t for t in pc.getTransceivers() if t.sender == sender)
    transceiver.setCodecPreferences(
        [codec for codec in codecs if codec.mimeType == forced_codec]
    )

async def index(request):
    content = open(os.path.join(ROOT, "index.html"), "r").read()
    return web.Response(content_type="text/html", text=content)

async def javascript(request):
    content = open(os.path.join(ROOT, "client.js"), "r").read()
    return web.Response(content_type="application/javascript", text=content)

async def offer(request):
    params = await request.json()
    offer = RTCSessionDescription(sdp=params["sdp"], type=params["type"])
    pc = RTCPeerConnection()
    pcs.add(pc)

    @pc.on("connectionstatechange")
    async def on_connectionstatechange():
        print("Connection state is %s" % pc.connectionState)
        if pc.connectionState == "failed":
            await pc.close()
            pcs.discard(pc)

    # open media source
    audio, video = create_local_tracks(
        args.play_from, decode=not args.play_without_decoding
    )

    if audio:
        audio_sender = pc.addTrack(audio)
        if args.audio_codec:
            force_codec(pc, audio_sender, args.audio_codec)
        elif args.play_without_decoding:
            raise Exception("You must specify the audio codec using --audio-codec")

    if video:
        video_sender = pc.addTrack(video)
        if args.video_codec:
            force_codec(pc, video_sender, args.video_codec)
        elif args.play_without_decoding:
            raise Exception("You must specify the video codec using --video-codec")

    await pc.setRemoteDescription(offer)

    answer = await pc.createAnswer()
    await pc.setLocalDescription(answer)

    return web.Response(
        content_type="application/json",
        text=json.dumps(
            {"sdp": pc.localDescription.sdp, "type": pc.localDescription.type}
        ),
    )


pcs = set()

async def on_shutdown(app):
    # close peer connections
    coros = [pc.close() for pc in pcs]
    await asyncio.gather(*coros)
    pcs.clear()



####################################
# ROS2 - publish and subscribe
####################################

jsonTopic = 'webrtcpub'

imageTopic = 'rtcimage'     # pre-processed image

# for use with basic realsense
imageTopic = "/camera/color/image_raw"

# for use with Isaac sym demo
imageTopic = "/front/stereo_camera/left/rgb"

# WebRTC node publish/subscribe
class WebRTCPubSub(Node):

    def __init__(self):
        super().__init__('pywebrtc')

        self.bridge = CvBridge()

        # subscriber - t.b.d.
        self.subscription = self.create_subscription(String,jsonTopic,self.json_listener_callback,10)
        self.subscription  # prevent unused variable warning

        self.imageSubscription = self.create_subscription(Image,imageTopic,self.listener_image_callback,10)
        self.imageSubscription  # prevent unused variable warning

        # publisher
        self.publisher_ = self.create_publisher(String, jsonTopic, 10)

        self.get_logger().info('WebRTCPubSub initialized')
        
    def json_listener_callback(self, msg):
        # publish the json to the browser via the data channel
        self.get_logger().info('JSON: I heard: "%s"' % msg.data)

    def listener_image_callback(self, image_message):
        cv_image = self.bridge.imgmsg_to_cv2(image_message, desired_encoding='passthrough')

        # and push to the WebRTC object - if it exists - defined as a global above
        if ros2vst is not None:
            ros2vst.setImg(cv_image)
        # self.get_logger().info('Rcvd new image')

    def json_pub(self,payload):        
        msg = String()
        msg.data = payload
        self.jsonpublisher_.publish(msg)
        self.get_logger().info('Published: "%s"' % msg.data)

def runROSNode(args=None):
    rclpy.init(args=args)
    webrtc_pubsub = WebRTCPubSub()
    rclpy.spin(webrtc_pubsub)

args=None
def main():
    global args
    # ROS2 node runs in a separate thread
    # communication via a global variable - 'ros2vst'

    t = threading.Thread(target=runROSNode, args=())
    t.start()

    parser = argparse.ArgumentParser(description="WebRTC webcam demo")
    parser.add_argument("--cert-file", help="SSL certificate file (for HTTPS)")
    parser.add_argument("--key-file", help="SSL key file (for HTTPS)")
    parser.add_argument("--play-from", help="Read the media from a file and sent it."),
    parser.add_argument(
        "--play-without-decoding",
        help=(
            "Read the media without decoding it (experimental). "
            "For now it only works with an MPEGTS container with only H.264 video."
        ),
        action="store_true",
    )
    parser.add_argument(
        "--host", default="0.0.0.0", help="Host for HTTP server (default: 0.0.0.0)"
    )
    parser.add_argument(
        "--port", type=int, default=8080, help="Port for HTTP server (default: 8080)"
    )
    parser.add_argument("--verbose", "-v", action="count")
    parser.add_argument(
        "--audio-codec", help="Force a specific audio codec (e.g. audio/opus)"
    )
    parser.add_argument(
        "--video-codec", help="Force a specific video codec (e.g. video/H264)"
    )

    args = parser.parse_args()

    if args.verbose:
        logging.basicConfig(level=logging.DEBUG)
    else:
        logging.basicConfig(level=logging.INFO)

    if args.cert_file:
        ssl_context = ssl.SSLContext()
        ssl_context.load_cert_chain(args.cert_file, args.key_file)
    else:
        ssl_context = None

    app = web.Application()
    app.on_shutdown.append(on_shutdown)
    app.router.add_get("/", index)
    app.router.add_get("/client.js", javascript)
    app.router.add_post("/offer", offer)
    web.run_app(app, host=args.host, port=args.port, ssl_context=ssl_context)

## Full main 
        


if __name__ == '__main__':
    main()
