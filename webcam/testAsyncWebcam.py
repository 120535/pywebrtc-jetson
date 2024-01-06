import argparse
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

import glob
from natsort import natsorted


fpSamples = "/home/pg/repos/sample/*.png"
fpSamples = "/repos/sample/*.png"




ROOT = os.path.dirname(__file__)


class ROS2VideoStreamTrack(VideoStreamTrack):
    """
    A video track that returns an animated flag.

    New idea - read a sequence of images from a folder - return those in 'order'...

    Next - this class simply returns the last set image (timeout to gray after N seconds?)
        - someone else, e.g., ROS2 puts images there...
    """

    def __init__(self):
        super().__init__()  # don't forget this!
        self.counter = 0
        height, width = 480, 640
        self.greyImg = self._create_rectangle( width=width, height=height, color=(64, 64, 64) )
        self.currentImg = self.greyImg
        return
    
        # Get a sorted list of all files in the directory
        files = natsorted(glob.glob(fpSamples))

        # Iterate through the sorted list of files
        self.frames = []
        k = 0



        for file in files:
            img = cv2.imread(file)
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)  # Convert the image from BGR to RGB
            img = cv2.resize(img, (640, 480))  # Resize the image to 640x480

            self.frames.append( VideoFrame.from_ndarray( img ) )

                # VideoFrame.from_ndarray( img, "bgr24" )
                #     cv2.remap(data_bgr, map_x, map_y, cv2.INTER_LINEAR), format="bgr24"
                # )
            # )

            # print(file)

        return


        # generate flag
        data_bgr = numpy.hstack(
            [
                self._create_rectangle(
                    width=213, height=480, color=(255, 0, 0)
                ),  # blue
                self._create_rectangle(
                    width=214, height=480, color=(255, 255, 255)
                ),  # white
                self._create_rectangle(width=213, height=480, color=(0, 0, 255)),  # red
            ]
        )

        # shrink and center it
        M = numpy.float32([[0.5, 0, width / 4], [0, 0.5, height / 4]])
        data_bgr = cv2.warpAffine(data_bgr, M, (width, height))

        # compute animation
        omega = 2 * math.pi / height
        id_x = numpy.tile(numpy.array(range(width), dtype=numpy.float32), (height, 1))
        id_y = numpy.tile(
            numpy.array(range(height), dtype=numpy.float32), (width, 1)
        ).transpose()

        self.frames = []
        for k in range(30):
            phase = 2 * k * math.pi / 30
            map_x = id_x + 10 * numpy.cos(omega * id_x + phase)
            map_y = id_y + 10 * numpy.sin(omega * id_x + phase)
            self.frames.append(
                VideoFrame.from_ndarray(
                    cv2.remap(data_bgr, map_x, map_y, cv2.INTER_LINEAR), format="bgr24"
                )
            )

    def setImg(self, img):
        self.currentImg = img       # hopefully atomically...

    async def recv(self):

        # img = cels[self.counter % 30]

        img = self.currentImg
        frame = VideoFrame.from_ndarray(img,format="bgr24")

        pts, time_base = await self.next_timestamp()

        # frame = self.frames[self.counter % 100]
        frame.pts = pts
        frame.time_base = time_base
        self.counter += 1
        return frame

    def _create_rectangle(self, width, height, color):
        data_bgr = numpy.zeros((height, width, 3), numpy.uint8)
        data_bgr[:, :] = color
        return data_bgr





relay = None
webcam = None
vidStream = None

def create_local_tracks(play_from, decode):
    global relay, webcam, vidStream

    # options = {"framerate": "1", "video_size": "640x480"}
    # webcam = MediaPlayer("/dev/video0", format="v4l2", options=options)
    # relay = MediaRelay()

    vidStream = ROS2VideoStreamTrack()
    return None,vidStream

    if play_from:
        player = MediaPlayer(play_from, decode=decode)
        return player.audio, player.video
    else:
        options = {"framerate": "5", "video_size": "640x480"}
        if relay is None:
            if platform.system() == "Darwin":
                webcam = MediaPlayer(
                    "default:none", format="avfoundation", options=options
                )
            elif platform.system() == "Windows":
                webcam = MediaPlayer(
                    "video=Integrated Camera", format="dshow", options=options
                )
            else:
                webcam = MediaPlayer("/dev/video0", format="v4l2", options=options)
            relay = MediaRelay()
        return None, relay.subscribe(webcam.video)


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


# the async part...
def create_rectangle(width, height, color):
    data_bgr = numpy.zeros((height, width, 3), numpy.uint8)
    data_bgr[:, :] = color
    return data_bgr

height, width = 480, 640
cels = []
count = 0
# generate flag
data_bgr = numpy.hstack(
        [
            create_rectangle(width=213, height=480, color=(255, 0, 0)),  # blue
            create_rectangle( width=214, height=480, color=(255, 255, 255) ),  # white
            create_rectangle( width=213, height=480, color=(0, 0, 255)),  # red
        ]
    )


def prepATest():
    global data_bgr,cels
    # shrink and center it
    M = numpy.float32([[0.5, 0, width / 4], [0, 0.5, height / 4]])
    data_bgr = cv2.warpAffine(data_bgr, M, (width, height))

    # compute animation
    omega = 2 * math.pi / height
    id_x = numpy.tile(numpy.array(range(width), dtype=numpy.float32), (height, 1))
    id_y = numpy.tile(
        numpy.array(range(height), dtype=numpy.float32), (width, 1)
    ).transpose()

    for k in range(30):
        phase = 2 * k * math.pi / 30
        map_x = id_x + 10 * numpy.cos(omega * id_x + phase)
        map_y = id_y + 10 * numpy.sin(omega * id_x + phase)
        cels.append( cv2.remap(data_bgr, map_x, map_y, cv2.INTER_LINEAR) )


import time, threading, random
def doATest():
    global count
    print(time.ctime(),count)
    nextElapse = 2 * random.random()
    threading.Timer(nextElapse, doATest).start()
    count = count+1
    img = cels[count % 30]
    if vidStream is not None:
        vidStream.setImg(img)



if __name__ == "__main__":
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

    prepATest()
    doATest()
    
    app = web.Application()
    app.on_shutdown.append(on_shutdown)
    app.router.add_get("/", index)
    app.router.add_get("/client.js", javascript)
    app.router.add_post("/offer", offer)
    web.run_app(app, host=args.host, port=args.port, ssl_context=ssl_context)

 
