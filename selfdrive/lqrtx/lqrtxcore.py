import socket
import sys
import requests
import subprocess
import time
import threading
import json
from common.params import Params
import cereal.messaging as messaging
from enum import Enum

params = Params()
params_memory = Params("/dev/shm/params")
params_storage = Params("/persist/comma/params")


# 定义一个枚举类型
class OPMessageActionType(Enum):
    """ All the action type defined here for reference """
    DEVICE_STATUS = 0 #OpenPilot Device status message
    AUTO_RESUME_CRUISE = 1 #OpenPilot using ESP auto resume cruise
    C3_AUTO_RESTART = 2 # OpenPilot auto restart event, used by informe user why it is restart, for now just used for tracing restart event when OP is not in ONROAD status


class C3UDPSendHelper(threading.Thread):
    ''' 负责发送C3的消息类 '''
    __opLocalIP = None # C3 自己的IP Address
    __c3UDPPort = 6599 # UDP port used by C3
    __udpSocket = None
    def __init__(self, port=6599, buffer_size=1024):
        '''
        初始化C3UDPSendHelper
        '''
        super().__init__(daemon=True)  # 设置为守护线程
        if port is not None:
            self.__c3UDPPort=port

    def getOPLocalIP(self):
        ''' get OpenPolit C3 local IPv4 IP Address '''
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            ip = s.getsockname()[0]
            s.close()
            return ip
        except Exception :
            return None

    def sendMsg(self,msg):
        try:
            if self.__udpSocket is not None:
                self.__udpSocket.sendto(json.dumps(msg).encode(), ("255.255.255.255", self.__c3UDPPort))
        except:
            pass

    def run(self):
        while True:
            try:
                self.__opLocalIP=self.getOPLocalIP()
                if self.__opLocalIP is None:
                    time.sleep(5)
                    continue
                if self.__udpSocket is not None:
                    try:
                        self.__udpSocket.close()
                    except:
                        None

                try:
                    self.__udpSocket=socket.socket(socket.AF_INET, socket.SOCK_DGRAM,socket.IPPROTO_UDP)
                    self.__udpSocket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
                except Exception as e:
                    #49 Can't assign requested address
                    #print(e.errno,e.strerror)
                    time.sleep(5)
                    continue
             
                while True:
                    try:
                        # 发送广播数据包到 255.255.255.255（全网广播）或 192.168.X.255（子网广播）
                        msg = {
                            "ACT":OPMessageActionType.DEVICE_STATUS.value, # ACT is integer type, need to add .value get the Enum is value. 
                            "a":params_memory.get_bool("CruiseAutoResumeActivated"), #CruiseAutoResumeActivated
                            "b":params_memory.get_bool("ESP32HasIP"), # has ESP32 or not.
                            "c":params_memory.get_bool("LqrtxOnRoad"), # OnRoad status
                            "a1":"", #C3 IP Address
                            "a2":""  #ESP32 IP Address
                        }
                        if params_memory.get_bool("ESP32HasIP"):
                            msg["a2"] = params_memory.get("ESP32IPAddress").decode()
                        if self.__opLocalIP is not None:
                            msg["a1"] = self.__opLocalIP
                        self.__udpSocket.sendto(json.dumps(msg).encode(), ("255.255.255.255", self.__c3UDPPort))
                        time.sleep(1)
                    except Exception as e:
                        #print("error",e)
                        break
            except Exception as e:
                None
                #print(f" catched excepiton: {e}")
            time.sleep(5)
  

class ESP32Helper(threading.Thread):
    ''' 负责接收ESP32的消息类 '''
    __opLocalIP = None # C3 自己的IP Address
    __esp32UDPPort = 6499 # UDP port used by ESP32
    __udpSocket = None
    __esp32IPAddress=None
    __lastResPlusClickTime=None
    __isEnginePowerOn=False
    __enginePowerOnTime=None
    __isOnRoadEver=False
    __udpSender=None
    def __init__(self, udpSender=None, port=6499, buffer_size=1024):
        '''
        初始化ESP32Helper
        '''
        super().__init__(daemon=True)  # 设置为守护线程
        if port is not None:
            self.__esp32UDPPort=port
        self.__udpSender = udpSender

    def getOPLocalIP(self):
        ''' get OpenPolit C3 local IPv4 IP Address '''
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            ip = s.getsockname()[0]
            s.close()
            return ip
        except Exception :
            return None

    def unpack(self,data_bytes):
        data = [byte for byte in data_bytes]  # Convert byte array to list of integers
        current_gear_num = data[3] >> 4
        rpm = (data[5] & 0xF) << 8 | data[6]
        ip_address = f"{data[16]}.{data[17]}.{data[18]}.{data[19]}"
        return {
            "gear": current_gear_num,
            "rpm": rpm,
            "ESP32IPAddress": ip_address
        }
    
    def __processOnRoadInAdvance(self):
        try:
            LqrtxOnRoad = params_memory.get_bool("LqrtxOnRoad")
            if not self.__isOnRoadEver and LqrtxOnRoad:
                # save if it on road ever or not, if is on road, it will not reboot in any case.
                self.__isOnRoadEver = True
            #if not self.__isOnRoadEver and self.__isEnginePowerOn: 
            if not LqrtxOnRoad and self.__isEnginePowerOn: 
                # Engine is power on and not on road will reboot
                msg = {
                    "ACT":OPMessageActionType.C3_AUTO_RESTART.value, # ACT is integer type, need to add .value get the Enum is value. 
                    "a":"Engine is power on but C3 not in ONROAD status after 60 seconds." # Notification message. 
                }
                self.notifyMsg(msg)
                # subprocess.check_output(["sudo", "reboot"]) # need to reboot.
                params.put_bool("DoReboot", True)

        except:
            None

    def notifyMsg(self,msg):
        try:
            if self.__udpSender is not None:
                self.__udpSender.sendMsg(msg)
        except Exception as e:
            pass

    def processAutResumeInAdvance(self):
        """ 
            Check if need to click res+ or not and will reset ESP32AutoResume to false after click res+ 
            the time gap must be at least 5 second between two clicks.
        """
        try:
            if params_memory.get_bool("ESP32AutoResume"):
                if self.__esp32IPAddress is not None and ( self.__lastResPlusClickTime is None or time.monotonic() - self.__lastResPlusClickTime >= 5) :
                    self.__sendResPlusClicked()
                    self.__lastResPlusClickTime=time.monotonic()
                    params_memory.put_bool("ESP32AutoResume",False)
                    msg = {
                        "ACT":OPMessageActionType.AUTO_RESUME_CRUISE.value, # ACT is integer type, need to add .value get the Enum is value. 
                        "a":"Vehicle Cruise Auto Resume" # Notification message. 
                    }
                    self.notifyMsg(msg)
                else:
                    params_memory.put_bool("ESP32AutoResume",False)
                
        except:
            None

    def __sendResPlusClicked(self):
        """ let esp click on Res+ button via http protocol """
        try:
            if self.__esp32IPAddress is not None and self.__esp32IPAddress != "":
                #"http://192.168.2.15/admin?CMD=104&Type=2"
                url = "http://" + self.__esp32IPAddress + "/admin?CMD=104&Type=2"
                requests.get(url)
        except:
            None

    def run(self):
        while True:
            try:
                """ Will reset has ESP32 to false before init UDP Receiver or catch any exception when received from ESP32 UDP. """
                params_memory.put_bool("ESP32HasIP",False)
                self.__isEnginePowerOn = False
                self.__enginePowerOnTime = None
                self.__esp32IPAddress = None
                self.__opLocalIP=self.getOPLocalIP()
                if self.__opLocalIP is None:
                    #print("ESP32Helper cannot get IP Address, waiting for 5 seconds")
                    time.sleep(5)
                    continue
                if self.__udpSocket is not None:
                    try:
                        self.__udpSocket.close()
                    except:
                        None
                try:
                    self.__udpSocket=socket.socket(socket.AF_INET, socket.SOCK_DGRAM,socket.IPPROTO_UDP)
                    self.__udpSocket.bind(("", self.__esp32UDPPort))
                    self.__udpSocket.settimeout(5)  # 设置超时时间为 5 秒
                except Exception as e:
                    #49 Can't assign requested address
                    #print(e.errno,e.strerror)
                    time.sleep(5)
                    continue
                while True:
                    try:
                        data, addr = self.__udpSocket.recvfrom(1024)
                        msg = self.unpack(data)
                        if(msg['ESP32IPAddress'] is not None and msg['ESP32IPAddress'] != self.__esp32IPAddress):
                            # get and set ESP32 IP Address and add to shared memory. 
                            self.__esp32IPAddress = msg['ESP32IPAddress']
                            params_memory.put("ESP32IPAddress",self.__esp32IPAddress)
                            params_memory.put_bool("ESP32HasIP",True)
                            #print(f"Received message: {msg} from {addr} {msg['ESP32IPAddress']}")

                        if msg['rpm'] is not None :
                            if msg["rpm"] > 100:
                                # get rpm, if rpm is larger than 100, it means the engine power is on and save engine power on time for later use.
                                if self.__enginePowerOnTime is None:
                                    # just set it at the first time RPM is larger than 100 
                                    self.__enginePowerOnTime = time.monotonic()
                                self.__isEnginePowerOn = True                                
                                powerOnTimePast = time.monotonic() - self.__enginePowerOnTime
                                #Only check for Engine Power between 60 and 300 seconds. out of range, will not checking.
                                if  powerOnTimePast > 30 and powerOnTimePast < 300:
                                    #check if need to reboot or not.
                                    self.__processOnRoadInAdvance()
                            else:
                                self.__enginePowerOnTime = None
                                self.__isEnginePowerOn = False
                        
                        params_memory.put_bool("ESP32EngineOn",self.__isEnginePowerOn)

                    except socket.timeout:
                        #print("Timeout! No message received.")
                        break
            except Exception as e:
                #print(f" catched excepiton: {e}")
                pass
            time.sleep(5)

def main():
    c3UDPSendHelper = C3UDPSendHelper()
    c3UDPSendHelper.start()
    esp32Helper = ESP32Helper(c3UDPSendHelper)
    esp32Helper.start()
    while True:
        try:
            # Need to do nessary operator if there has need state changes
            #print("recevied message from manager ",sm['lqrtxDeviceState'].eventType,sm['lqrtxDeviceState'].eventJson)
            # Do not care what is the envet type and event json. just do the check.
            esp32Helper.processAutResumeInAdvance() # check if need to auto resume or not.
        except:
            None
        time.sleep(0.2)


if __name__ == "__main__":
    main()
