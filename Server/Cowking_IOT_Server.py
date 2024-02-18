#!/usr/bin/env python3

import asyncio
import socket
import urllib.request
import datetime
import sys
import logging

device_addresses = [
    '10.42.42.215',
    '10.42.42.103'
]

kBrightness_Day = 200       # Max brightness is 255, I just don't like it too bright
kBrightness_Night = 16

kMorningTime = 9.0  # Start fading in IOT lights at 9am
kEveningTime = 18.0  # Start fading out IOT lights at 6pm
kFadeDuration = 1.0  # Fade smoothly over 1 hour

logfile = f'./Cowking_IOT_Server.log'

logging.basicConfig(
    filename=logfile,
    format='%(message)s %(asctime)s',
    filemode='w+'
)
logger = logging.getLogger()
logger.setLevel(logging.INFO)
logger.addHandler(logging.StreamHandler(sys.stdout))

def server_log(message):
    logging.info('{}: {}'.format(datetime.datetime.now(), message))

# Broadcast counters
def broadcast_data():
    broadcast_data_low_freq()
    # broadcast_data_high_freq()


def broadcast_data_low_freq():  # Will be called once every ~2 minutes
    try:
        intensity = get_desired_iot_brightness()
        for address in device_addresses:    # Apply desired brightness to all devices
            device_set_intensity(intensity, address)

    except Exception as e:
        logging.exception("Exception encountered during low frequency broadcast", exc_info=e)


#def broadcast_data_high_freq():  # Will be called once every ~10 seconds
#    try:
#        
#    except:
#        logging.error('')


# Returns the desired counter intensity based on the time of day
def get_desired_iot_brightness():
    now = datetime.datetime.now()
    linear_time = now.hour + (now.minute / 60.0) + (now.second / 3600.0)

    fader_in = min((linear_time - (kMorningTime + kFadeDuration / 2)) / kFadeDuration, 0.5)
    fader_out = min(((kEveningTime + kFadeDuration / 2) - linear_time) / kFadeDuration, 0.5)
    return max(0.0, fader_in + fader_out)

# ------------------------------------------------------------------------------
# Heartbeats / Ticks
# Regularly updates the devices with data.
# Right now only the brightness is adjusted based on the time of day, to make
# the devices fade out during the night.
# ------------------------------------------------------------------------------

# 2-minutes heartbeat
async def heartbeat_120():
    while True:
        try:
            broadcast_data_low_freq()
            await asyncio.sleep(120)
        except Exception as e:
            logging.exception("Exception encountered during low frequency heartbeat", exc_info=e)

# 10-seconds heartbeat
# async def heartbeat_10():
#     while True:
#         try:
#             await asyncio.sleep(10)
#         except Exception as e:
#             logging.exception("Exception encountered during high frequency heartbeat", exc_info=e)

# ------------------------------------------------------------------------------
# Server management & message-handling callback
# ------------------------------------------------------------------------------

class EchoServerProtocol(asyncio.Protocol):
    def connection_made(self, transport):
        peername = transport.get_extra_info('peername')
        # server_log('Connection from {}'.format(peername))
        self.transport = transport

    def data_received(self, data):
        message = data.decode().rstrip() # rstrip: strip unwanted terminating newlines
        # server_log('Command received: {!r}'.format(message))
        server_log('Unknown command received: "{}"'.format(message))
        self.transport.close()

# ------------------------------------------------------------------------------
# Main entrypoint
# ------------------------------------------------------------------------------

async def main():
    server_log('Launching server...')

    # Get a reference to the event loop as we plan to use
    # low-level APIs.
    loop = asyncio.get_running_loop()

    # Note: if heartbeats cause the devices to send us a command,
    # it will not be processed through this initial call because the server is not running yet

    asyncio.ensure_future(heartbeat_120(), loop=loop)
    # asyncio.ensure_future(heartbeat_10(), loop=loop)


    # Robustness check: What happens if we pull the network plug and then try to run this script?
    # Could happen if the machine reboots and this is launched via a daemon/cron job on startup or whatever.
    # What happens if we pull the network plug while the server is running? does it stop? is it able to
    # auto-reconnect by itself when network comes back online? IP could change,
    # maybe the server stops with an exception or something. Test this.
    self_ip = getSelfIP()
    self_port = 8888
    server = await loop.create_server(lambda: EchoServerProtocol(), self_ip, self_port)

    server_log('Server running @ {}'.format(self_ip))

    async with server:
        while True:
            try:
                await server.serve_forever()
            except Exception as e:
                logging.exception('Server crashed: ')

# ------------------------------------------------------------------------------
# HW DEVICES: Entrypoint for the various device 'set' functions, can be called directly
# ------------------------------------------------------------------------------

def device_set_intensity(value, device_address):
    try:
        deviceIntensity = remap(value, kBrightness_Night, kBrightness_Day)
        urllib.request.urlopen("http://{}/configure?bright={}".format(device_address, deviceIntensity), timeout=1)
    except:
        pass

# ------------------------------------------------------------------------------
# Various misc helpers
# ------------------------------------------------------------------------------

def remap(value01, newMin, newMax):
    return newMin + value01 * (newMax - newMin)


# get our own IP address
def getSelfIP():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.connect(("8.8.8.8", 80))
    return s.getsockname()[0]


# ------------------------------------------------------------------------------
# RUN

asyncio.run(main())
