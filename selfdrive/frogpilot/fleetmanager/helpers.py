# otisserv - Copyright (c) 2019-, Rick Lan, dragonpilot community, and a number of other of contributors.
# Fleet Manager - [actuallylemoncurd](https://github.com/actuallylemoncurd), [AlexandreSato](https://github.com/alexandreSato), [ntegan1](https://github.com/ntegan1), [royjr](https://github.com/royjr), and [sunnyhaibin] (https://github.com/sunnypilot)
# Almost everything else - ChatGPT
# dirty PR pusher - mike8643
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
import json
import math
import os
import requests
import subprocess
import time
import socket #UDP测试
import random
import struct
# otisserv conversion
from common.params import Params
from flask import render_template, request, session
from functools import wraps
from pathlib import Path

from openpilot.system.hardware import PC
from openpilot.system.hardware.hw import Paths
from openpilot.system.loggerd.uploader import listdir_by_creation
from tools.lib.route import SegmentName
from typing import List
from openpilot.system.loggerd.xattr_cache import getxattr

# otisserv conversion
from urllib.parse import parse_qs, quote

class CanMsg:
  ipaddr = ""
  GEARS = [1, 2, 3, 4, 5, 6, 7, 8, 9, 13, 14, 15]

  def __init__(self):
    self.length = 31
    self.version = 1
    self.steeringWheelAngle = random.randint(-540, 540)
    self.currentGearNum = random.choice(self.GEARS)
    self.driverMode = 0
    self.brakePressure = 0
    self.turnSignals = 0
    self.brakePressed = 0
    self.hazardLights = 0
    self.rpm = 0
    self.engineCoolantTemp = 0
    self.accSpeed = 0
    self.accGapLevel = 0
    self.leftBSM = 0
    self.rightBSM = 0
    self.flDoor = 0
    self.frDoor = 0
    self.rlDoor = 0
    self.rrDoor = 0
    self.gasPedal = 0
    self.distance = 0

    self.speed = 0
    self.flSpeed = 0
    self.frSpeed = 0
    self.rlSpeed = 0
    self.rrSpeed = 0
    self.nextGearNum = 0

  def pack(self):
    data = bytearray(32)
    data[0] = ((self.length - 1) << 3) | (self.version >> 5)
    tmpSteeringWheelAngle = self.steeringWheelAngle + 720
    data[1] = ((self.version & 0x1F) << 3) | (tmpSteeringWheelAngle >> 8)
    data[2] = tmpSteeringWheelAngle & 0xFF
    data[3] = (self.currentGearNum << 4) | self.driverMode
    data[4] = self.brakePressure & 0xFF
    tmpRpm = self.rpm
    data[5] = (self.turnSignals << 6) | (self.brakePressed << 5) | (self.hazardLights << 4) | (tmpRpm >> 8)
    data[6] = tmpRpm & 0xFF
    data[7] = (self.engineCoolantTemp + 40) & 0xFF
    data[8] = self.accSpeed & 0xFF
    data[9] = (self.accGapLevel << 6) | (self.leftBSM << 5) | (self.rightBSM << 4) | (self.flDoor << 3) | (self.frDoor << 2) | (self.rlDoor << 1) | self.rrDoor
    data[10] = self.gasPedal & 0xFF
    tmpDistance = self.distance
    data[11] = (tmpDistance >> 32) & 0xFF
    data[12] = (tmpDistance >> 24) & 0xFF
    data[13] = (tmpDistance >> 16) & 0xFF
    data[14] = (tmpDistance >> 8) & 0xFF
    data[15] = tmpDistance & 0xFF

    data[16] = (self.speed >> 4) & 0xFF
    data[17] = ((self.speed & 0xF) << 4) & 0xFF

    data[17] += ((self.flSpeed >> 8) & 0xF)
    data[18] = self.flSpeed & 0xFF

    data[19] = (self.frSpeed >> 4) & 0xFF
    data[20] = ((self.frSpeed & 0xF) << 4) & 0xFF

    data[20] += ((self.rlSpeed >> 8) & 0xF)
    data[21] = self.rlSpeed & 0xFF

    data[22] = (self.rrSpeed >> 4) & 0xFF
    data[23] = ((self.rrSpeed & 0xF) << 4) & 0xFF

    data[23] += (self.nextGearNum & 0xF)

    return bytes(data)

  def randomize(self):
    self.length = 31
    self.steeringWheelAngle = random.randint(-540, 540)
    self.currentGearNum = random.choice(self.GEARS)
    self.driverMode = random.randint(0, 3)
    self.brakePressure = random.randint(0, 100)
    self.turnSignals = random.randint(0, 2)
    self.brakePressed = 1 if self.brakePressure != 0 else 0
    self.hazardLights = random.randint(0, 1)
    self.rpm = 600 + random.randint(0, 5000)
    self.engineCoolantTemp = 60 + random.randint(0, 100)
    self.accSpeed = 20 + random.randint(0, 150)
    self.accGapLevel = 1 + random.randint(0, 2)
    self.leftBSM = random.randint(0, 1)
    self.rightBSM = random.randint(0, 1)
    self.flDoor = random.randint(0, 1)
    self.frDoor = random.randint(0, 1)
    self.rlDoor = random.randint(0, 1)
    self.rrDoor = random.randint(0, 1)
    self.gasPedal = random.randint(0, 100)
    self.distance = 0xffffffffff
    self.speed = random.randint(0, 180) * 10 + random.randint(0, 10)
    self.flSpeed = self.speed + random.randint(0, 30)
    self.frSpeed = self.speed + random.randint(0, 30)
    self.rrSpeed = self.speed + random.randint(0, 30)
    self.rlSpeed = self.speed + random.randint(0, 30)
    self.nextGearNum = random.choice(self.GEARS)

    packed_data = self.pack()
    # print(list(packed_data))

pi = 3.1415926535897932384626
x_pi = 3.14159265358979324 * 3000.0 / 180.0
a = 6378245.0
ee = 0.00669342162296594323

params = Params()
params_memory = Params("/dev/shm/params")
params_storage = Params("/persist/comma/params")

PRESERVE_ATTR_NAME = 'user.preserve'
PRESERVE_ATTR_VALUE = b'1'
PRESERVE_COUNT = 5

# path to openpilot screen recordings and error logs
if PC:
  SCREENRECORD_PATH = os.path.join(str(Path.home()), ".comma", "media", "0", "videos", "")
  ERROR_LOGS_PATH = os.path.join(str(Path.home()), ".comma", "community", "crashes", "")
else:
  SCREENRECORD_PATH = "/data/media/0/videos/"
  ERROR_LOGS_PATH = "/data/community/crashes/"

#UDP测试
UDP_PORT = 6499
can_msg = CanMsg()

def udp_send_message():
  UDP_SOCKET = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

  while True:
    try:
      if can_msg.ipaddr:
        can_msg.randomize()
        UDP_SOCKET.sendto(can_msg.pack(), (can_msg.ipaddr, UDP_PORT))
      time.sleep(1)
    except Exception:
      pass
    continue
    
  UDP_SOCKET.close()
  
def list_files(path): # still used for footage
  return sorted(listdir_by_creation(path), reverse=True)


def list_file(path): # new function for screenrecords/error-logs
  if os.path.exists(path):
    files = os.listdir(path)
    sorted_files = sorted(files, reverse=True)
  else:
    return []  # Return an empty list if there are no files or directory
  return sorted_files


def is_valid_segment(segment):
  try:
    segment_to_segment_name(Paths.log_root(), segment)
    return True
  except AssertionError:
    return False


def segment_to_segment_name(data_dir, segment):
  fake_dongle = "ffffffffffffffff"
  return SegmentName(str(os.path.join(data_dir, fake_dongle + "|" + segment)))


def all_segment_names():
  segments = []
  for segment in listdir_by_creation(Paths.log_root()):
    try:
      segments.append(segment_to_segment_name(Paths.log_root(), segment))
    except AssertionError:
      pass
  return segments


def all_routes():
  segment_names = all_segment_names()
  route_names = [segment_name.route_name for segment_name in segment_names]
  route_times = [route_name.time_str for route_name in route_names]
  unique_routes = list(dict.fromkeys(route_times))
  return sorted(unique_routes, reverse=True)

def preserved_routes():
  dirs = listdir_by_creation(Paths.log_root())
  preserved_segments = get_preserved_segments(dirs)
  return sorted(preserved_segments, reverse=True)

def has_preserve_xattr(d: str) -> bool:
  return getxattr(os.path.join(Paths.log_root(), d), PRESERVE_ATTR_NAME) == PRESERVE_ATTR_VALUE

def get_preserved_segments(dirs_by_creation: List[str]) -> List[str]:
  preserved = []
  for n, d in enumerate(filter(has_preserve_xattr, reversed(dirs_by_creation))):
    if n == PRESERVE_COUNT:
      break
    date_str, _, seg_str = d.rpartition("--")

    # ignore non-segment directories
    if not date_str:
      continue
    try:
      seg_num = int(seg_str)
    except ValueError:
      continue
    # preserve segment and its prior
    preserved.append(d)

  return preserved

def video_to_gif(input_path, output_path, fps=1, duration=6): # not used right now but can if want longer animated gif
  if os.path.exists(output_path):
    return
  command = [
    'ffmpeg', '-y', '-i', input_path,
    '-filter_complex',
    f'fps={fps},scale=240:-1:flags=lanczos,setpts=0.1*PTS,split[s0][s1];[s0]palettegen=max_colors=32[p];[s1][p]paletteuse=dither=bayer',
    '-t', str(duration), output_path
  ]
  subprocess.run(command)
  print(f"GIF file created: {output_path}")

def video_to_img(input_path, output_path, fps=1, duration=6):
  if os.path.exists(output_path):
    return
  subprocess.run(['ffmpeg', '-y', '-i', input_path, '-ss', '5', '-vframes', '1', output_path])
  print(f"GIF file created: {output_path}")

def segments_in_route(route):
  segment_names = [segment_name for segment_name in all_segment_names() if segment_name.time_str == route]
  segments = [segment_name.time_str + "--" + str(segment_name.segment_num) for segment_name in segment_names]
  return segments


def ffmpeg_mp4_concat_wrap_process_builder(file_list, cameratype, chunk_size=1024*512):
  command_line = ["ffmpeg"]
  if not cameratype == "qcamera":
    command_line += ["-f", "hevc"]
  command_line += ["-r", "20"]
  command_line += ["-i", "concat:" + file_list]
  command_line += ["-c", "copy"]
  command_line += ["-map", "0"]
  if not cameratype == "qcamera":
    command_line += ["-vtag", "hvc1"]
  command_line += ["-f", "mp4"]
  command_line += ["-movflags", "empty_moov"]
  command_line += ["-"]
  return subprocess.Popen(
    command_line, stdout=subprocess.PIPE,
    bufsize=chunk_size
  )


def ffmpeg_mp4_wrap_process_builder(filename):
  """Returns a process that will wrap the given filename
     inside a mp4 container, for easier playback by browsers
     and other devices. Primary use case is streaming segment videos
     to the vidserver tool.
     filename is expected to be a pathname to one of the following
       /path/to/a/qcamera.ts
       /path/to/a/dcamera.hevc
       /path/to/a/ecamera.hevc
       /path/to/a/fcamera.hevc
  """
  basename = filename.rsplit("/")[-1]
  extension = basename.rsplit(".")[-1]
  command_line = ["ffmpeg"]
  if extension == "hevc":
    command_line += ["-f", "hevc"]
  command_line += ["-r", "20"]
  command_line += ["-i", filename]
  command_line += ["-c", "copy"]
  command_line += ["-map", "0"]
  if extension == "hevc":
    command_line += ["-vtag", "hvc1"]
  command_line += ["-f", "mp4"]
  command_line += ["-movflags", "empty_moov"]
  command_line += ["-"]
  return subprocess.Popen(
    command_line, stdout=subprocess.PIPE
  )


def ffplay_mp4_wrap_process_builder(file_name):
  command_line = ["ffmpeg"]
  command_line += ["-i", file_name]
  command_line += ["-c", "copy"]
  command_line += ["-map", "0"]
  command_line += ["-f", "mp4"]
  command_line += ["-movflags", "empty_moov"]
  command_line += ["-"]
  return subprocess.Popen(
    command_line, stdout=subprocess.PIPE
  )

def get_nav_active():
  if params.get("NavDestination", encoding='utf8') is not None:
    return True
  else:
    return False

def get_public_token():
  token = params.get("MapboxPublicKey", encoding='utf8')
  return token.strip() if token is not None else None

def get_app_token():
  token = params.get("MapboxSecretKey", encoding='utf8')
  return token.strip() if token is not None else None

def get_gmap_key():
  token = params.get("GMapKey", encoding='utf8')
  return token.strip() if token is not None else None

def get_amap_key():
  token = params.get("AMapKey1", encoding='utf8')
  token2 = params.get("AMapKey2", encoding='utf8')
  return (token.strip() if token is not None else None, token2.strip() if token2 is not None else None)

def get_SearchInput():
  SearchInput = params.get_int("SearchInput")
  return SearchInput

def get_PrimeType():
  PrimeType = params.get_int("PrimeType")
  return PrimeType

def get_last_lon_lat():
  last_pos = params.get("LastGPSPosition")
  if last_pos:
    l = json.loads(last_pos)
  else:
    return 0.0, 0.0
  return l["longitude"], l["latitude"]

def get_locations():
  data = params.get("ApiCache_NavDestinations", encoding='utf-8')
  return data

def preload_favs():
  try:
    nav_destinations = json.loads(params.get("ApiCache_NavDestinations", encoding='utf8'))
  except TypeError:
    return (None, None, None, None, None)

  locations = {"home": None, "work": None, "fav1": None, "fav2": None, "fav3": None}

  for item in nav_destinations:
    label = item.get("label")
    if label in locations and locations[label] is None:
      locations[label] = item.get("place_name")

  return tuple(locations.values())

def parse_addr(postvars, lon, lat, valid_addr, token):
  addr = postvars.get("fav_val", [""])
  real_addr = None
  if addr != "favorites":
    try:
      dests = json.loads(params.get("ApiCache_NavDestinations", encoding='utf8'))
    except TypeError:
      dests = json.loads("[]")
    for item in dests:
      if "label" in item and item["label"] == addr:
        lat, lon, real_addr = item["latitude"], item["longitude"], item["place_name"]
        break
  return (real_addr, lon, lat, real_addr is not None, token)

def search_addr(postvars, lon, lat, valid_addr, token):
  if "addr_val" in postvars:
    addr = postvars.get("addr_val")
    if addr != "":
      # Properly encode the address to handle spaces
      addr_encoded = quote(addr)
      query = f"https://api.mapbox.com/geocoding/v5/mapbox.places/{addr_encoded}.json?access_token={token}&limit=1"
      # focus on place around last gps position
      lngi, lati = get_last_lon_lat()
      query += "&proximity=%s,%s" % (lngi, lati)
      r = requests.get(query)
      if r.status_code != 200:
        return (addr, lon, lat, valid_addr, token)
      j = json.loads(r.text)
      if not j["features"]:
        return (addr, lon, lat, valid_addr, token)
      lon, lat = j["features"][0]["geometry"]["coordinates"]
      valid_addr = True
  return (addr, lon, lat, valid_addr, token)

def set_destination(postvars, valid_addr):
  if postvars.get("latitude") is not None and postvars.get("longitude") is not None:
    postvars["lat"] = postvars.get("latitude")
    postvars["lon"] = postvars.get("longitude")
    postvars["save_type"] = "recent"
    nav_confirmed(postvars)
    valid_addr = True
  else:
    addr = postvars.get("place_name")
    token = get_public_token()
    data, lon, lat, valid_addr, token = search_addr(addr, lon, lat, valid_addr, token)
    postvars["lat"] = lat
    postvars["lon"] = lon
    postvars["save_type"] = "recent"
    nav_confirmed(postvars)
    valid_addr= True
  return postvars, valid_addr

def nav_confirmed(postvars):
  if postvars is not None:
    lat = float(postvars.get("lat"))
    lng = float(postvars.get("lon"))
    save_type = postvars.get("save_type")
    name = postvars.get("name") if postvars.get("name") is not None else ""
    if params.get_int("SearchInput") == 1:
      lng, lat = gcj02towgs84(lng, lat)
    params.put("NavDestination", "{\"latitude\": %f, \"longitude\": %f, \"place_name\": \"%s\"}" % (lat, lng, name))
    if name == "":
      name =  str(lat) + "," + str(lng)
    new_dest = {"latitude": float(lat), "longitude": float(lng), "place_name": name}
    if save_type == "recent":
      new_dest["save_type"] = "recent"
    else:
      new_dest["save_type"] = "favorite"
      new_dest["label"] = save_type
    val = params.get("ApiCache_NavDestinations", encoding='utf8')
    if val is not None:
      val = val.rstrip('\x00')
    dests = [] if val is None else json.loads(val)
    # type idx
    type_label_ids = {"home": None, "work": None, "fav1": None, "fav2": None, "fav3": None, "recent": []}
    idx = 0
    for d in dests:
      if d["save_type"] == "favorite":
        type_label_ids[d["label"]] = idx
      else:
        type_label_ids["recent"].append(idx)
      idx += 1
    if save_type == "recent":
      id = None
      if len(type_label_ids["recent"]) > 10:
        dests.pop(type_label_ids["recent"][-1])
    else:
      id = type_label_ids[save_type]
    if id is None:
      dests.insert(0, new_dest)
    else:
      dests[id] = new_dest
    params.put("ApiCache_NavDestinations", json.dumps(dests).rstrip("\n\r"))

def public_token_input(postvars):
  if postvars is None or "pk_token_val" not in postvars or postvars.get("pk_token_val")[0] == "":
    return postvars
  else:
    token = postvars.get("pk_token_val").strip()
    if "pk." not in token:
      return postvars
    else:
        params.put("MapboxPublicKey", token)
  return token

def app_token_input(postvars):
  if postvars is None or "sk_token_val" not in postvars or postvars.get("sk_token_val")[0] == "":
    return postvars
  else:
    token = postvars.get("sk_token_val").strip()
    if "sk." not in token:
      return postvars
    else:
        params.put("MapboxSecretKey", token)
  return token

def gmap_key_input(postvars):
  if postvars is None or "gmap_key_val" not in postvars or postvars.get("gmap_key_val")[0] == "":
    return postvars
  else:
    token = postvars.get("gmap_key_val").strip()
    params.put("GMapKey", token)
  return token

def amap_key_input(postvars):
  if postvars is None or "amap_key_val" not in postvars or postvars.get("amap_key_val")[0] == "":
    return postvars
  else:
    token = postvars.get("amap_key_val").strip()
    token2 = postvars.get("amap_key_val_2").strip()
    params.put("AMapKey1", token)
    params.put("AMapKey2", token2)
  return token

def gcj02towgs84(lng, lat):
  dlat = transform_lat(lng - 105.0, lat - 35.0)
  dlng = transform_lng(lng - 105.0, lat - 35.0)
  radlat = lat / 180.0 * pi
  magic = math.sin(radlat)
  magic = 1 - ee * magic * magic
  sqrtmagic = math.sqrt(magic)
  dlat = (dlat * 180.0) / ((a * (1 - ee)) / (magic * sqrtmagic) * pi)
  dlng = (dlng * 180.0) / (a / sqrtmagic * math.cos(radlat) * pi)
  mglat = lat + dlat
  mglng = lng + dlng
  return [lng * 2 - mglng, lat * 2 - mglat]

def transform_lat(lng, lat):
  ret = -100.0 + 2.0 * lng + 3.0 * lat + 0.2 * lat * lat + 0.1 * lng * lat + 0.2 * math.sqrt(abs(lng))
  ret += (20.0 * math.sin(6.0 * lng * pi) + 20.0 * math.sin(2.0 * lng * pi)) * 2.0 / 3.0
  ret += (20.0 * math.sin(lat * pi) + 40.0 * math.sin(lat / 3.0 * pi)) * 2.0 / 3.0
  ret += (160.0 * math.sin(lat / 12.0 * pi) + 320 * math.sin(lat * pi / 30.0)) * 2.0 / 3.0
  return ret

def transform_lng(lng, lat):
  ret = 300.0 + lng + 2.0 * lat + 0.1 * lng * lng + 0.1 * lng * lat + 0.1 * math.sqrt(abs(lng))
  ret += (20.0 * math.sin(6.0 * lng * pi) + 20.0 * math.sin(2.0 * lng * pi)) * 2.0 / 3.0
  ret += (20.0 * math.sin(lng * pi) + 40.0 * math.sin(lng / 3.0 * pi)) * 2.0 / 3.0
  ret += (150.0 * math.sin(lng / 12.0 * pi) + 300.0 * math.sin(lng / 30.0 * pi)) * 2.0 / 3.0
  return ret

def get_all_toggle_values():
  all_keys = [
    "AdjustablePersonalities", "PersonalitiesViaWheel", "PersonalitiesViaScreen", "AlwaysOnLateral", "AlwaysOnLateralMain", "HideAOLStatusBar",
    "ConditionalExperimental", "CESpeed", "CESpeedLead", "CECurves", "CECurvesLead", "CENavigation", "CENavigationIntersections",
    "CENavigationLead", "CENavigationTurns", "CESlowerLead", "CEStopLights", "CEStopLightsLead", "CESignal", "HideCEMStatusBar",
    "CustomPersonalities", "AggressiveFollow", "AggressiveJerk", "StandardFollow", "StandardJerk", "RelaxedFollow", "RelaxedJerk", "DeviceShutdown",
    "ExperimentalModeActivation", "ExperimentalModeViaDistance", "ExperimentalModeViaLKAS", "ExperimentalModeViaScreen", "FireTheBabysitter",
    "NoLogging", "NoDashCam", "NoUploads", "MuteOverheated", "OfflineMode", "LateralTune", "ForceAutoTune", "NNFF", "SteerRatio", "UseLateralJerk", "LongitudinalTune",
    "AccelerationProfile", "DecelerationProfile", "AggressiveAcceleration", "StoppingDistance", "SmoothBraking", "Model", "MTSCEnabled",
    "DisableMTSCSmoothing", "MTSCAggressiveness", "MTSCCurvatureCheck", "MTSCLimit", "NudgelessLaneChange", "LaneChangeTime", "LaneDetection",
    "LaneDetectionWidth", "OneLaneChange", "QOLControls", "DisableOnroadUploads", "HigherBitrate", "NavChill", "PauseLateralOnSignal",
    "ReverseCruise", "ReverseCruiseUI", "SetSpeedLimit", "SetSpeedOffset",  "SpeedLimitController", "Offset1", "Offset2", "Offset3", "Offset4",
    "SLCConfirmation", "SLCConfirmationLower", "SLCConfirmationHigher", "SLCFallback", "SLCPriority1", "SLCPriority2", "SLCPriority3", "SLCOverride",
    "TurnDesires", "VisionTurnControl", "DisableVTSCSmoothing", "CurveSensitivity", "TurnAggressiveness",

    "EVTable", "GasRegenCmd", "LongPitch", "LowerVolt", "CrosstrekTorque", "CydiaTune", "DragonPilotTune", "FrogsGoMooTune", "LockDoors", "SNGHack",

    "CustomTheme", "CustomColors", "CustomIcons", "CustomSignals", "CustomSounds", "GoatScream", "AlertVolumeControl", "DisengageVolume",
    "EngageVolume", "PromptVolume", "PromptDistractedVolume", "RefuseVolume", "WarningSoftVolume", "WarningImmediateVolume", "CameraView",
    "Compass", "CustomAlerts", "GreenLightAlert", "LeadDepartingAlert", "LoudBlindspotAlert", "SpeedLimitChangedAlert", "CustomUI", "AccelerationPath",
    "AdjacentPath", "AdjacentPathMetrics", "BlindSpotPath", "FPSCounter", "LeadInfo", "UseSI", "PedalsOnUI", "RoadNameUI", "UseVienna", "DriverCamera",
    "ModelUI", "DynamicPathWidth", "HideLeadMarker", "LaneLinesWidth", "PathEdgeWidth", "PathWidth", "RoadEdgesWidth", "UnlimitedLength", "QOLVisuals",
    "DriveStats", "FullMap", "HideSpeed", "HideSpeedUI", "ShowSLCOffset", "SpeedLimitChangedAlert", "WheelSpeed", "RandomEvents", "ScreenManagement",
    "HideUIElements", "HideAlerts", "HideMapIcon", "HideMaxSpeed", "ScreenBrightness", "ScreenBrightnessOnroad", "ScreenTimeout", "ScreenTimeoutOnroad",
    "ScreenRecorder", "StandbyMode", "WheelIcon", "RotatingWheel", "NumericalTemp", "Fahrenheit", "ShowCPU", "ShowGPU", "ShowIP", "ShowMemoryUsage",
    "ShowStorageLeft", "ShowStorageUsed", "Sidebar",

    "MinSteerSpeedStandard", "MinSteerSpeedEngage",
    "DashSpeedRatio1", "DashSpeedRatio2", "DashSpeedRatio3", "SetSpeedRatio1", "SetSpeedRatio2", "SetSpeedRatio3", "SpeedDecimal",
    "FrogPilotDrives", "FrogPilotKilometers", "FrogPilotMinutes", "CarMake", "CarModel", 
    "DriverPrivacyProtectionFake", "CSLCEnabled", "CalibrationCycles",
  ]

  toggle_values = {}
  for key in all_keys:
    try:
      value = params.get(key)
    except Exception:
      value = b"0"
    toggle_values[key] = value.decode('utf-8') if value is not None else "0"

  return toggle_values

def store_toggle_values(updated_values):
  for key, value in updated_values.items():
    try:
      # readonly key
      if key not in ["FrogPilotDrives", "FrogPilotKilometers", "FrogPilotMinutes", "CarMake", "CarModel"]:
        params.put(key, value.encode('utf-8'))
        params_storage.put(key, value.encode('utf-8'))
    except Exception as e:
      print(f"Failed to update {key}: {e}")

  params_memory.put_bool("FrogPilotTogglesUpdated", True)
  time.sleep(1)
  params_memory.put_bool("FrogPilotTogglesUpdated", False)

def screen_recorder_button(toggle):
  params_memory.put_int("ScreenRecorderToggle", toggle)

  params_memory.put_bool("FrogPilotTogglesUpdated", True)
  # time.sleep(1)
  # params_memory.put_bool("FrogPilotTogglesUpdated", False)

def sidebar_button(toggle):
  params_memory.put_int("SidebarToggle", toggle)

  params_memory.put_bool("FrogPilotTogglesUpdated", True)
  time.sleep(1)
  params_memory.put_bool("FrogPilotTogglesUpdated", False)  

def reverse_cruise_button(toggle):
  if toggle==1: #enabled
    params_memory.put_int("ReverseCruiseRunTime", 2)
  elif toggle==2: #disabled
    params_memory.put_int("ReverseCruiseRunTime", 1)
  else: #toggled
    params_memory.put_int("ReverseCruiseRunTime", 1 if params_memory.get_int("ReverseCruiseRunTime")==2 else 2)

  params_memory.put_bool("FrogPilotTogglesUpdated", True)
  time.sleep(1)
  params_memory.put_bool("FrogPilotTogglesUpdated", False)  

def nudgeless_button(toggle):
  if toggle==1: #enabled
    params.put_bool_nonblocking("NudgelessLaneChange", True)
  elif toggle==2: #disabled
    params.put_bool_nonblocking("NudgelessLaneChange", False)
  else: #toggled
    params.put_bool_nonblocking("NudgelessLaneChange", not params.get_bool("NudgelessLaneChange"))

  params_memory.put_bool("FrogPilotTogglesUpdated", True)
  time.sleep(1)
  params_memory.put_bool("FrogPilotTogglesUpdated", False)

def lateral_control_button(toggle):
  if toggle==1: #enabled
    params_memory.put_bool("LateralDisableRunTime", False)
  elif toggle==2: #disabled
    params_memory.put_bool("LateralDisableRunTime", True)
  else: #toggled
    params_memory.put_int("LateralDisableRunTime", not params_memory.get_bool("LateralDisableRunTime"))

  params_memory.put_bool("FrogPilotTogglesUpdated", True)
  time.sleep(1)
  params_memory.put_bool("FrogPilotTogglesUpdated", False)

def udp_broadcast_ip(ipaddr):
  can_msg.ipaddr = ipaddr if ipaddr else ""