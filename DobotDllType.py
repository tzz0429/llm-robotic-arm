# -*- coding: utf-8 -*-
import os
import time
import platform
from ctypes import *

def enum(**enums):
    return type('Enum', (), enums)

# API result
DobotConnect = enum(
    DobotConnect_NoError=0,
    DobotConnect_NotFound=1,
    DobotConnect_Occupied=2)

DobotCommunicate = enum(
    DobotCommunicate_NoError=0,
    DobotCommunicate_BufferFull=1,
    DobotCommunicate_Timeout=2)

# 实时位姿
class Pose(Structure):
    _pack_ = 1
    _fields_ = [
        ("x", c_float),
        ("y", c_float),
        ("z", c_float),
        ("rHead", c_float),
        ("joint1Angle", c_float),
        ("joint2Angle", c_float),
        ("joint3Angle", c_float),
        ("joint4Angle", c_float)
        ]

# 报警功能
class AlarmsState(Structure):
    _pack_ = 1
    _fields_ = [
        ("alarmsState", c_int32)
        ]

# 回零功能
class HOMEParams(Structure):
    _pack_ = 1
    _fields_ = [
        ("x", c_float), 
        ("y", c_float), 
        ("z", c_float), 
        ("r", c_float)
        ]

class HOMECmd(Structure):
    _pack_ = 1
    _fields_ = [
        ("temp", c_float)
        ]

# 末端执行器
class EndTypeParams(Structure):
    _pack_ = 1
    _fields_ = [
        ("xBias", c_float),
        ("yBias", c_float),
        ("zBias", c_float)
        ]

# JOG功能
class JOGJointParams(Structure):
    _pack_ = 1
    _fields_ = [
        ("joint1Velocity", c_float), 
        ("joint2Velocity", c_float), 
        ("joint3Velocity", c_float), 
        ("joint4Velocity", c_float), 
        ("joint1Acceleration", c_float),
        ("joint2Acceleration", c_float),
        ("joint3Acceleration", c_float),
        ("joint4Acceleration", c_float)
        ]

class JOGCoordinateParams(Structure):
    _pack_ = 1
    _fields_ = [
        ("xVelocity", c_float), 
        ("yVelocity", c_float), 
        ("zVelocity", c_float), 
        ("rVelocity", c_float), 
        ("xAcceleration", c_float),
        ("yAcceleration", c_float),
        ("zAcceleration", c_float),
        ("rAcceleration", c_float)
        ]

class JOGLParams(Structure):
    _pack_ = 1
    _fields_ = [
        ("velocity",  c_float), 
        ("acceleration",  c_float)
    ]

class JOGCommonParams(Structure):
    _pack_ = 1
    _fields_ = [
        ("velocityRatio", c_float), 
        ("accelerationRatio", c_float)
        ]

JC = enum(JogIdle=0, 
    JogAPPressed=1, 
    JogANPressed=2, 
    JogBPPressed=3, 
    JogBNPressed=4,
    JogCPPressed=5,
    JogCNPressed=6,
    JogDPPressed=7,
    JogDNPressed=8,
    JogEPPressed=9,
    JogENPressed=10)

class JOGCmd(Structure):
    _pack_ = 1
    _fields_ = [
        ("isJoint", c_byte), 
        ("cmd", c_byte)
        ]

# PTP功能
class PTPJointParams(Structure):
    _fields_ = [
        ("joint1Velocity", c_float), 
        ("joint2Velocity", c_float), 
        ("joint3Velocity", c_float), 
        ("joint4Velocity", c_float), 
        ("joint1Acceleration", c_float),
        ("joint2Acceleration", c_float),
        ("joint3Acceleration", c_float),
        ("joint4Acceleration", c_float)
        ]
        
class PTPCoordinateParams(Structure):
    _fields_ = [
        ("xyzVelocity", c_float), 
        ("rVelocity", c_float),
        ("xyzAcceleration", c_float), 
        ("rAcceleration", c_float)
        ]

class PTPJumpParams(Structure):
    _pack_ = 1
    _fields_ = [
        ("jumpHeight", c_float), 
        ("zLimit", c_float)
        ]

class PTPCommonParams(Structure):
    _pack_ = 1
    _fields_ = [
        ("velocityRatio", c_float), 
        ("accelerationRatio", c_float)
        ]

PTPMode = enum(
    PTPJUMPXYZMode=0,
    PTPMOVJXYZMode=1,
    PTPMOVLXYZMode=2,
    
    PTPJUMPANGLEMode=3,
    PTPMOVJANGLEMode=4,
    PTPMOVLANGLEMode=5,
    
    PTPMOVJANGLEINCMode=6,
    PTPMOVLXYZINCMode=7, 
    PTPMOVJXYZINCMode=8, 
    
    PTPJUMPMOVLXYZMode=9)

InputPin = enum( InputPinNone=0,
    InputPin1=1,
    InputPin2=2,
    InputPin3=3,
    InputPin4=4,
    InputPin5=5,
    InputPin6=6,
    InputPin7=7,
    InputPin8=8)

InputLevel = enum(InputLevelBoth=0,
    InputLevelLow=1,
    InputLevelHigh=2)

OutputPin = enum(
    SIGNALS_O1=1,
    SIGNALS_O2=2,
    SIGNALS_O3=3,
    SIGNALS_O4=4,
    SIGNALS_O5=5,
    SIGNALS_O6=6,
    SIGNALS_O7=7,
    SIGNALS_O8=8)
    
class PTPCmd(Structure):
    _pack_ = 1
    _fields_ = [
        ("ptpMode", c_byte),
        ("x", c_float),
        ("y", c_float),
        ("z", c_float),
        ("rHead", c_float),
        ("gripper", c_float)
        ]
        
class PTPWithLCmd(Structure):
    _pack_ = 1
    _fields_ = [
        ("ptpMode", c_byte),
        ("x", c_float),
        ("y", c_float),
        ("z", c_float),
        ("rHead", c_float),
        ("l", c_float),
        ("gripper", c_float)
        ]

# CP功能
class CPParams(Structure):
    _pack_ = 1
    _fields_ = [
        ("planAcc", c_float),
        ("juncitionVel", c_float),
        ("acc", c_float), 
        ("realTimeTrack",  c_byte)
        ]

ContinuousPathMode = enum(
    CPRelativeMode=0,
    CPAbsoluteMode=1)

class CPCmd(Structure):
    _pack_ = 1
    _fields_ = [
        ("cpMode", c_byte),
        ("x", c_float),
        ("y", c_float),
        ("z", c_float),
        ("velocity", c_float)
        ]

# ARC功能
class ARCPoint(Structure):
    _pack_ = 1
    _fields_ = [
        ("x", c_float),
        ("y", c_float),
        ("z", c_float),
        ("rHead", c_float)
    ]
        
class ARCParams(Structure):
    _pack_ = 1
    _fields_ = [
        ("xyzVelocity", c_float), 
        ("rVelocity", c_float),
        ("xyzAcceleration", c_float), 
        ("rAcceleration", c_float)
        ]

class ARCCmd(Structure):
    _pack_ = 1
    _fields_ = [
        ("cirPoint", ARCPoint),
        ("toPoint", ARCPoint)
    ]

# WAIT功能
class WAITParams(Structure):
    _pack_ = 1
    _fields_ = [
        ("unitType", c_byte)
        ]

class WAITCmd(Structure):
    _pack_ = 1
    _fields_ = [
        ("waitTime", c_uint32)
        ]

# WAIT功能
TRIGMode = enum(
    TRIGInputIOMode = 0,
    TRIGADCMode=1)
    
TRIGInputIOCondition = enum(
    TRIGInputIOEqual = 0,
    TRIGInputIONotEqual=1)
    
TRIGADCCondition = enum(
    TRIGADCLT = 0,
    TRIGADCLE=1, 
    TRIGADCGE = 2,
    TRIGADCGT=3)
    
class TRIGCmd(Structure):
    _pack_ = 1
    _fields_ = [
        ("address", c_byte), 
        ("mode", c_byte), 
        ("condition",  c_byte), 
        ("threshold", c_uint16)
        ]

# EIO功能
GPIOType = enum(
    GPIOTypeDO = 1,
    GPIOTypePWM=2,
    GPIOTypeDI=3, 
    GPIOTypeADC=4)
    
class IOMultiplexing(Structure):
    _pack_ = 1
    _fields_ = [
        ("address", c_byte), 
        ("multiplex", c_byte)
        ]
        
class IODO(Structure):
    _pack_ = 1
    _fields_ = [
        ("address", c_byte), 
        ("level", c_byte)
        ]
        
class IOPWM(Structure):
    _pack_ = 1
    _fields_ = [
        ("address", c_byte), 
        ("frequency", c_float), 
        ("dutyCycle", c_float)
        ]
        
class IODI(Structure):
    _pack_ = 1
    _fields_ = [
        ("address", c_byte), 
        ("level", c_byte)
        ]
        
class IOADC(Structure):
    _pack_ = 1
    _fields_ = [
        ("address", c_byte), 
        ("value", c_int)
        ]

class EMotor(Structure):
    _pack_ = 1
    _fields_ = [
        ("index", c_byte), 
        ("isEnabled", c_byte), 
        ("speed", c_int32)
        ]
        
class EMotorS(Structure):
    _pack_ = 1
    _fields_ = [
        ("index", c_byte), 
        ("isEnabled", c_byte), 
        ("deltaPulse", c_int)
        ]

# 加载动态链接库
def load(dll_path=None):
    if platform.system() == "Windows":
        if dll_path is None:
            return CDLL("DobotDll.dll", RTLD_GLOBAL)
        return CDLL(dll_path, RTLD_GLOBAL)
    elif platform.system() == "Darwin":
        return CDLL("libDobotDll.dylib", RTLD_GLOBAL)
    else:
        return cdll.LoadLibrary(os.getcwd() + "/libDobotDll.so")

#**************************** 事件循环功能 ****************************#
def DobotExec(api):
    return api.DobotExec()

#**************************** 设置指令超时时间 ****************************#
# 延时函数 ms
def dSleep(ms):
    time.sleep(ms / 1000)

def output(str):
    #print(str)
    #sys.stdout.flush()
    pass

#****************************** 连接/断开 *********************************#
# 搜索Dobot
# ((len(str(maxLen)) + 4) * maxLen + 10)
def SearchDobot(api,  maxLen=1000):
    szPara = create_string_buffer(1000) 
    l = api.SearchDobot(szPara,  maxLen)
    if l == 0:
        return []
    ret = szPara.value.decode("utf-8") 
    return ret.split(" ")

# 连接Dobot
def ConnectDobot(api, portName,  baudrate):
    # 串口号
    szPara = create_string_buffer(100)
    szPara.raw = portName.encode("utf-8") 
    # 固件类型 Dobot和Marlin
    fwType = create_string_buffer(100)
    # 版本号
    version = create_string_buffer(100)
    # DobotDll API接口函数
    result = api.ConnectDobot(szPara,  baudrate,  fwType,  version)
    return [result, fwType.value.decode("utf-8"), version.value.decode("utf-8")]

# 断开Dobot
def DisconnectDobot(api):
    api.DisconnectDobot()

# 设置指令超时时间
def SetCmdTimeout(api, times):
    api.SetCmdTimeout(times)

#****************************** 指令队列控制 ****************************#
# 执行队列指令
def SetQueuedCmdStartExec(api):
    while(True):
        result = api.SetQueuedCmdStartExec()
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break

# 停止执行队列指令
def SetQueuedCmdStopExec(api):
    while(True):
        result = api.SetQueuedCmdStopExec()
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break

# 强制停止队列指令
def SetQueuedCmdForceStopExec(api):
    while(True):
        result = api.SetQueuedCmdForceStopExec()
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break

# 下载指令
def SetQueuedCmdStartDownload(api,  totalLoop, linePerLoop):
    while(True):
        result = api.SetQueuedCmdStartDownload(totalLoop, linePerLoop)
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break

# 停止下载指令       
def SetQueuedCmdStopDownload(api):
    while(True):
        result = api.SetQueuedCmdStopDownload()
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break

# 清空队列指令
def SetQueuedCmdClear(api):
    return api.SetQueuedCmdClear()

# 获取指令索引
def GetQueuedCmdCurrentIndex(api):
    queuedCmdIndex = c_uint64(0)
    while(True):
        result = api.GetQueuedCmdCurrentIndex(byref(queuedCmdIndex))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(2)
            continue
        break
    return [queuedCmdIndex.value]

#****************************** 设备信息 *********************************#
# 设置设备序列号
def SetDeviceSN(api, str): 
    szPara = create_string_buffer(25)
    szPara.raw = str.encode("utf-8")
    while(True):
        result = api.SetDeviceSN(szPara)
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break

# 获取设备序列号
def GetDeviceSN(api): 
    szPara = create_string_buffer(25)
    while(True):
        result = api.GetDeviceSN(szPara,  25)
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    ret = szPara.value.decode("utf-8") 
    output('GetDeviceSN: ' + ret)
    return ret

# 设置设备名称
def SetDeviceName(api, str): 
    szPara = create_string_buffer(66)
    szPara.raw = str.encode("utf-8")
    while(True):
        result = api.SetDeviceName(szPara)
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break

# 获取设备名称
def GetDeviceName(api): 
    szPara = create_string_buffer(66)  
    while(True):
        result = api.GetDeviceName(szPara,  100)
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    ret = szPara.value.decode("utf-8")
    output('GetDeviceName: ' + ret)
    return ret

# 获取设备版本号    
def GetDeviceVersion(api):
    majorVersion = c_byte(0)
    minorVersion = c_byte(0)
    revision = c_byte(0)
    while(True):
        result = api.GetDeviceVersion(byref(majorVersion),  byref(minorVersion),  byref(revision))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    output('GetDeviceVersion: V%d.%d.%d' %(majorVersion.value,  minorVersion.value,  revision.value))
    return [majorVersion.value,  minorVersion.value,  revision.value]

# 设置滑轨状态
def SetDeviceWithL(api,  isWithL):
    cIsWithL = c_bool(isWithL)
    while(True):
        result = api.SetDeviceWithL(cIsWithL)
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break

# 获取滑轨状态    
def GetDeviceWithL(api):
    isWithL = c_bool(False)
    while(True):
        result = api.GetDeviceWithL(byref(isWithL))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    return isWithL.value

#****************************** 实时位姿 ****************************#
# 获取机械臂实时姿态
def GetPose(api):
    pose = Pose()
    while(True):
        result = api.GetPose(byref(pose))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    output('GetPose: %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f' %(pose.x,pose.y,pose.z,pose.rHead, pose.joint1Angle,pose.joint2Angle,pose.joint3Angle,pose.joint4Angle))
    return [pose.x, pose.y, pose.z,pose.rHead, pose.joint1Angle, pose.joint2Angle, pose.joint3Angle, pose.joint4Angle]

# 获取滑轨实时位置
def GetPoseL(api):
    l = c_float(0)
    while(True):
        result = api.GetPoseL(byref(l))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    return [l.value]

# 重设机器人实时位姿
def ResetPose(api, manual, rearArmAngle, frontArmAngle):
    c_rearArmAngle = c_float(rearArmAngle)
    c_frontArmAngle = c_float(frontArmAngle)
    while(True):
        result = api.ResetPose(manual, c_rearArmAngle, c_frontArmAngle)
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break

#****************************** 报警功能 ****************************#
# 获取系统报警状态
def GetAlarmsState(api,  maxLen=1000):
    alarmsState = create_string_buffer(maxLen) 
    #alarmsState = c_byte(0)
    len = c_int(0)
    while(True):
        result = api.GetAlarmsState(alarmsState,  byref(len),  maxLen)
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    #output('GetAlarmsState: alarmsState=%.4f len=%.4f' %(alarmsState.value, len.value))
    return [alarmsState.raw, len.value]

# 清除系统所有报警    
def ClearAllAlarmsState(api):
    while(True):
        result = api.ClearAllAlarmsState()
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break

#****************************** 回零功能 ****************************#
# 设置回零位置
def SetHOMEParams(api,  x,  y,  z,  r,  isQueued=0):
    param = HOMEParams()
    param.x = x
    param.y = y
    param.z = z
    param.r = r
    queuedCmdIndex = c_uint64(0)
    while(True):
        result = api.SetHOMEParams(byref(param),  isQueued, byref(queuedCmdIndex))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    return [queuedCmdIndex.value]

# 获取回零位置
def GetHOMEParams(api):
    param = HOMEParams()
    while(True):
        result = api.GetHOMEParams(byref(param))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    output('GetUserParams: %.4f' %(param.temp))
    return [param.temp]

# 执行回零功能
def SetHOMECmd(api, temp, isQueued=0):
    cmd = HOMECmd()
    cmd.temp = temp
    queuedCmdIndex = c_uint64(0)
    while(True):
        result = api.SetHOMECmd(byref(cmd), isQueued, byref(queuedCmdIndex))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    return [queuedCmdIndex.value]

#****************************** HHT功能 *******************************#
# 设置触发模式
def SetHHTTrigMode(api, hhtTrigMode):
    while(True):
        result = api.SetHHTTrigMode(hhtTrigMode)
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break

# 获取触发模式        
def GetHHTTrigMode(api):
    hhtTrigMode = c_int(0)
    while(True):
        result = api.GetHHTTrigMode(byref(hhtTrigMode))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    return [hhtTrigMode.value]

# 设置手持示教使能状态
def SetHHTTrigOutputEnabled(api, isEnabled):
    while(True):
        result = api.SetHHTTrigOutputEnabled(isEnabled)
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break

# 获取手持示教使能状态
def GetHHTTrigOutputEnabled(api):
    isEnabled = c_int32(0)
    while(True):
        result = api.GetHHTTrigOutputEnabled(byref(isEnabled))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    return [isEnabled.value]

# 获取手持示教触发信号
def GetHHTTrigOutput(api):
    isAvailable = c_int32(0)
    result = api.GetHHTTrigOutput(byref(isAvailable))
    if result != DobotCommunicate.DobotCommunicate_NoError or isAvailable.value == 0:
        return False
    return True

#****************************** 末端执行器 ****************************#
# 设置末端坐标偏移量
def SetEndEffectorParams(api, xBias, yBias, zBias, isQueued=0):
    param = EndTypeParams()
    param.xBias = xBias
    param.yBias = yBias
    param.zBias = zBias
    queuedCmdIndex = c_uint64(0)
    while(True):
        result = api.SetEndEffectorParams(byref(param),  isQueued,  byref(queuedCmdIndex))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    return [queuedCmdIndex.value]

# 获取末端坐标偏移量
def GetEndEffectorParams(api):
    param = EndTypeParams()
    while(True):
        result = api.GetEndEffectorParams(byref(param))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    output('GetEndEffectorParams: xBias=%.4f yBias=%.4f zBias=%.4f' %(param.xBias, param.yBias, param.zBias))
    return [param.xBias, param.yBias, param.zBias]

# 设置激光状态
def SetEndEffectorLaser(api, enableCtrl,  on, isQueued=0):
    queuedCmdIndex = c_uint64(0)
    while(True):
        result = api.SetEndEffectorLaser(enableCtrl,  on,  isQueued,  byref(queuedCmdIndex))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    return [queuedCmdIndex.value]

# 获取激光状态
def GetEndEffectorLaser(api):
    isCtrlEnabled = c_int(0)
    isOn = c_int(0)
    while(True):
        result = api.GetEndEffectorLaser(byref(isCtrlEnabled),  byref(isOn))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    output('GetEndEffectorLaser: isCtrlEnabled=%d, isOn=%4f' %(isCtrlEnabled.value,  isOn.value))
    return [isCtrlEnabled.value, isOn.value]

# 设置气泵状态
def SetEndEffectorSuctionCup(api, enableCtrl,  on, isQueued=0):
    queuedCmdIndex = c_uint64(0)
    while(True):
        result = api.SetEndEffectorSuctionCup(enableCtrl,  on,  isQueued,  byref(queuedCmdIndex))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    return [queuedCmdIndex.value]

# 获取气泵状态
def GetEndEffectorSuctionCup(api):
    enableCtrl = c_int(0)
    isOn = c_int(0)
    while(True):
        result = api.GetEndEffectorSuctionCup(byref(enableCtrl),  byref(isOn))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    output('GetEndEffectorSuctionCup: isOn=%.4f' %(isOn.value))
    return [isOn.value]

# 设置夹爪状态
def SetEndEffectorGripper(api, enableCtrl,  on, isQueued=0):
    queuedCmdIndex = c_uint64(0)
    while(True):
        result = api.SetEndEffectorGripper(enableCtrl,  on,  isQueued,  byref(queuedCmdIndex))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    return [queuedCmdIndex.value]

# 获取夹爪状态        
def GetEndEffectorGripper(api):
    enableCtrl = c_int(0)
    isOn = c_int(0)
    while(True):
        result = api.GetEndEffectorGripper(byref(enableCtrl),  byref(isOn))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    output('GetEndEffectorGripper: isOn=%.4f' %(isOn.value))
    return [isOn.value]

#****************************** JOG功能 ****************************#
# 设置点动时各关节坐标轴的速度和加速度
def SetJOGJointParams(api, j1Velocity, j1Acceleration, j2Velocity, j2Acceleration, j3Velocity, j3Acceleration, j4Velocity, j4Acceleration, isQueued=0):
    jogParam = JOGJointParams()
    jogParam.joint1Velocity = j1Velocity
    jogParam.joint1Acceleration = j1Acceleration
    jogParam.joint2Velocity = j2Velocity
    jogParam.joint2Acceleration = j2Acceleration
    jogParam.joint3Velocity = j3Velocity
    jogParam.joint3Acceleration = j3Acceleration
    jogParam.joint4Velocity = j4Velocity
    jogParam.joint4Acceleration = j4Acceleration
    queuedCmdIndex = c_uint64(0)
    while(True):
        result = api.SetJOGJointParams(byref(jogParam), isQueued, byref(queuedCmdIndex))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    return [queuedCmdIndex.value]

# 获取点动时各关节坐标轴的速度和加速度
def GetJOGJointParams(api):
    param = JOGJointParams()
    while(True):
        result = api.GetJOGJointParams(byref(param))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    output('GetJOGJointParams: %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f' %(param.joint1Velocity, param.joint1Acceleration, param.joint2Velocity, param.joint2Acceleration, param.joint3Velocity, param.joint3Acceleration, param.joint4Velocity, param.joint4Acceleration))
    return [param.joint1Velocity, param.joint1Acceleration, param.joint2Velocity, param.joint2Acceleration, param.joint3Velocity, param.joint3Acceleration, param.joint4Velocity, param.joint4Acceleration]

# 设置点动时笛卡尔坐标轴的速度和加速度
def SetJOGCoordinateParams(api, xVelocity, xAcceleration, yVelocity, yAcceleration, zVelocity, zAcceleration, rVelocity, rAcceleration, isQueued=0):
    param = JOGCoordinateParams()
    param.xVelocity = xVelocity
    param.xAcceleration = xAcceleration
    param.yVelocity = yVelocity
    param.yAcceleration = yAcceleration
    param.zVelocity = zVelocity
    param.zAcceleration = zAcceleration
    param.rVelocity = rVelocity
    param.rAcceleration = rAcceleration
    queuedCmdIndex = c_uint64(0)
    while(True):
        result = api.SetJOGCoordinateParams(byref(param), isQueued, byref(queuedCmdIndex))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    return [queuedCmdIndex.value]

# 获取点动时笛卡尔坐标轴的速度和加速度
def GetJOGCoordinateParams(api):
    param = JOGCoordinateParams()
    while(True):
        result = api.GetJOGCoordinateParams(byref(param))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    output('GetJOGCoordinateParams: %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f' %(param.xVelocity, param.xAcceleration, param.yVelocity, param.yVelocity, param.zVelocity, param.zAcceleration, param.rVelocity, param.rAcceleration))
    return [param.xVelocity, param.xAcceleration, param.yVelocity, param.yVelocity, param.zVelocity, param.zAcceleration, param.rVelocity, param.rAcceleration]

# 设置点动时滑轨速度和加速度
def SetJOGLParams(api, velocity, acceleration, isQueued=0):
    param = JOGLParams()
    param.velocity = velocity
    param.acceleration = acceleration
    queuedCmdIndex = c_uint64(0)
    while(True):
        result = api.SetJOGLParams(byref(param), isQueued, byref(queuedCmdIndex))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    return [queuedCmdIndex.value]

# 获取时滑轨速度和加速度 
def GetJOGLParams(api):
    param = JOGLParams()
    while(True):
        result = api.GetJOGLParams(byref(param))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    return [param.velocity,  param.acceleration]

# 设置点动速度百分比和加速度百分比
def SetJOGCommonParams(api, value_velocityratio, value_accelerationratio, isQueued=0):
    param = JOGCommonParams()
    param.velocityRatio = value_velocityratio
    param.accelerationRatio = value_accelerationratio
    queuedCmdIndex = c_uint64(0)
    while(True):
        result = api.SetJOGCommonParams(byref(param), isQueued, byref(queuedCmdIndex))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    return [queuedCmdIndex.value]

# 获取点动速度百分比和加速度百分比
def GetJOGCommonParams(api):
    param = JOGCommonParams()
    while(True):
        result = api.GetJOGCommonParams(byref(param))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    output('GetJOGCommonParams: velocityRatio=%.4f accelerationRatio=%.4f' %(param.velocityRatio, param.accelerationRatio))
    return [param.velocityRatio, param.accelerationRatio]

# 执行点动指令
def SetJOGCmd(api, isJoint, cmd, isQueued=0):
    cmdParam = JOGCmd()
    cmdParam.isJoint = isJoint
    cmdParam.cmd = cmd
    queuedCmdIndex = c_uint64(0)
    while(True):
        result = api.SetJOGCmd(byref(cmdParam), isQueued, byref(queuedCmdIndex))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    return [queuedCmdIndex.value]

#****************************** PTP功能 ****************************#
# 设置PTP模式下各关节坐标轴的速度和加速度
def SetPTPJointParams(api, j1Velocity, j1Acceleration, j2Velocity, j2Acceleration, j3Velocity, j3Acceleration, j4Velocity, j4Acceleration, isQueued=0):
    pbParam = PTPJointParams()
    pbParam.joint1Velocity = j1Velocity
    pbParam.joint1Acceleration = j1Acceleration
    pbParam.joint2Velocity = j2Velocity
    pbParam.joint2Acceleration = j2Acceleration
    pbParam.joint3Velocity = j3Velocity
    pbParam.joint3Acceleration = j3Acceleration
    pbParam.joint4Velocity = j4Velocity
    pbParam.joint4Acceleration = j4Acceleration
    queuedCmdIndex = c_uint64(0)
    while(True):
        result = api.SetPTPJointParams(byref(pbParam), isQueued, byref(queuedCmdIndex))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    return [queuedCmdIndex.value]

# 获取PTP模式下各关节坐标轴的速度和加速度
def GetPTPJointParams(api):
    pbParam = PTPJointParams()
    while(True):
        result = api.GetPTPJointParams(byref(pbParam))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    output('GetPTPJointParams: %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f' %(pbParam.joint1Velocity,pbParam.joint1Acceleration,pbParam.joint2Velocity,pbParam.joint2Acceleration,pbParam.joint3Velocity,pbParam.joint3Acceleration,pbParam.joint4Velocity,pbParam.joint4Acceleration))
    return [pbParam.joint1Velocity,pbParam.joint1Acceleration,pbParam.joint2Velocity,pbParam.joint2Acceleration,pbParam.joint3Velocity,pbParam.joint3Acceleration,pbParam.joint4Velocity,pbParam.joint4Acceleration]

# 设置PTP模式下各笛卡尔坐标轴的速度和加速度
def SetPTPCoordinateParams(api, xyzVelocity, xyzAcceleration, rVelocity,  rAcceleration,  isQueued=0):
    pbParam = PTPCoordinateParams()
    pbParam.xyzVelocity = xyzVelocity
    pbParam.rVelocity = rVelocity
    pbParam.xyzAcceleration = xyzAcceleration
    pbParam.rAcceleration = rAcceleration
    queuedCmdIndex = c_uint64(0)
    while(True):
        result = api.SetPTPCoordinateParams(byref(pbParam), isQueued, byref(queuedCmdIndex))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    return [queuedCmdIndex.value]

# 获取PTP模式下各笛卡尔坐标轴的速度和加速度
def GetPTPCoordinateParams(api):
    pbParam = PTPCoordinateParams()
    while(True):
        result = api.GetPTPCoordinateParams(byref(pbParam))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    output('GetPTPCoordinateParams: xyzVelocity=%.4f rVelocity=%.4f xyzAcceleration=%.4f rAcceleration=%.4f' %(pbParam.xyzVelocity, pbParam.rVelocity, pbParam.xyzAcceleration, pbParam.rAcceleration))
    return [pbParam.xyzVelocity, pbParam.rVelocity, pbParam.xyzAcceleration, pbParam.rAcceleration]
    
# 设置JUMP模式下抬升高度和最大抬升高度
def SetPTPJumpParams(api, jumpHeight, zLimit, isQueued=0):
    pbParam = PTPJumpParams()
    pbParam.jumpHeight = jumpHeight
    pbParam.zLimit = zLimit
    queuedCmdIndex = c_uint64(0)
    while(True):
        result = api.SetPTPJumpParams(byref(pbParam), isQueued, byref(queuedCmdIndex))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    return [queuedCmdIndex.value]

# 获取JUMP模式下抬升高度和最大抬升高度
def GetPTPJumpParams(api):
    pbParam = PTPJumpParams()
    while(True):
        result = api.GetPTPJumpParams(byref(pbParam))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    output('GetPTPJumpParams: jumpHeight=%.4f zLimit=%.4f' %(pbParam.jumpHeight, pbParam.zLimit))
    return [pbParam.jumpHeight, pbParam.zLimit]

# 设置PTP运动速度百分比和加速度百分比
def SetPTPCommonParams(api, velocityRatio, accelerationRatio, isQueued=0):
    pbParam = PTPCommonParams()
    pbParam.velocityRatio = velocityRatio
    pbParam.accelerationRatio = accelerationRatio
    queuedCmdIndex = c_uint64(0)
    while(True):
        result = api.SetPTPCommonParams(byref(pbParam), isQueued, byref(queuedCmdIndex))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    return [queuedCmdIndex.value]

# 获取PTP运动速度百分比和加速度百分比
def GetPTPCommonParams(api):
    pbParam = PTPCommonParams()
    while(True):
        result = api.GetPTPCommonParams(byref(pbParam ))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    output('GetPTPCommonParams: velocityRatio=%.4f accelerationRatio=%.4f' %(pbParam.velocityRatio, pbParam.accelerationRatio))
    return [pbParam.velocityRatio, pbParam.accelerationRatio]

# 执行PTP指令
def SetPTPCmd(api, ptpMode, x, y, z, rHead, isQueued=0):
    cmd = PTPCmd()
    cmd.ptpMode = ptpMode
    cmd.x = x
    cmd.y = y
    cmd.z = z
    cmd.rHead = rHead
    queuedCmdIndex = c_uint64(0)
    while(True):
        result = api.SetPTPCmd(byref(cmd), isQueued, byref(queuedCmdIndex))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(2)
            continue
        break
    return [queuedCmdIndex.value]

# 执行带滑轨的PTP指令
def SetPTPWithLCmd(api, ptpMode, x, y, z, rHead, l, isQueued=0):
    cmd = PTPWithLCmd()
    cmd.ptpMode=ptpMode
    cmd.x=x
    cmd.y=y
    cmd.z=z
    cmd.rHead=rHead
    cmd.l = l
    queuedCmdIndex = c_uint64(0)
    while(True):
        result = api.SetPTPWithLCmd(byref(cmd), isQueued, byref(queuedCmdIndex))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(2)
            continue
        break
    return [queuedCmdIndex.value]

#****************************** CP功能 ****************************#
# 设置CP运动的速度和加速度
def SetCPParams(api, planAcc, juncitionVel, acc, realTimeTrack = 0,  isQueued=0):
    parm = CPParams()
    parm.planAcc = planAcc
    parm.juncitionVel = juncitionVel
    parm.acc = acc
    parm.realTimeTrack = realTimeTrack
    queuedCmdIndex = c_uint64(0)
    while(True):
        result = api.SetCPParams(byref(parm), isQueued, byref(queuedCmdIndex))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    return [queuedCmdIndex.value]

# 获取CP运动的速度和加速度
def GetCPParams(api):
    parm = CPParams()
    while(True):
        result = api.GetCPParams(byref(parm))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    output('GetCPParams: planAcc=%.4f juncitionVel=%.4f acc=%.4f' %(parm.planAcc, parm.juncitionVel, parm.acc))
    return [parm.planAcc, parm.juncitionVel, parm.acc]

# 执行CP指令
def SetCPCmd(api, cpMode, x, y, z, velocity, isQueued=0):
    cmd = CPCmd()
    cmd.cpMode = cpMode
    cmd.x = x
    cmd.y = y
    cmd.z = z
    cmd.velocity = velocity
    queuedCmdIndex = c_uint64(0)
    while(True):
        result = api.SetCPCmd(byref(cmd), isQueued, byref(queuedCmdIndex))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(2)
            continue
        break
    return [queuedCmdIndex.value]

# 执行带激光雕刻的CP指令  
def SetCPLECmd(api, cpMode, x, y, z, power, isQueued=0):
    cmd = CPCmd()
    cmd.cpMode = cpMode
    cmd.x = x
    cmd.y = y
    cmd.z = z
    cmd.velocity = power
    queuedCmdIndex = c_uint64(0)
    while(True):
        result = api.SetCPLECmd(byref(cmd), isQueued, byref(queuedCmdIndex))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(2)
            continue
        break
    return [queuedCmdIndex.value]

#****************************** ARC功能 ****************************#
# 设置ARC运动的速度和加速度
def SetARCParams(api,  xyzVelocity, rVelocity, xyzAcceleration, rAcceleration,  isQueued=0):
    param = ARCParams()
    param.xyzVelocity = xyzVelocity
    param.rVelocity = rVelocity
    param.xyzAcceleration = xyzAcceleration
    param.rAcceleration = rAcceleration
    queuedCmdIndex = c_uint64(0)
    while(True):
        result = api.SetARCParams(byref(param), isQueued, byref(queuedCmdIndex))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    return [queuedCmdIndex.value]

# 获取ARC运动的速度和加速度        
def GetARCParams(api):
    parm = ARCParams()
    while(True):
        result = api.GetARCParams(byref(parm))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    output('GetARCParams: xyzVelocity=%.4f,rVelocity=%.4f,xyzAcceleration=%.4f,rAcceleration=%.4f' %(parm.xyzVelocity, parm.rVelocity, parm.xyzAcceleration, parm.rAcceleration))
    return [parm.xyzVelocity, parm.rVelocity, parm.xyzAcceleration, parm.rAcceleration]

# 执行ARC指令    
def SetARCCmd(api, cirPoint, toPoint,  isQueued=0):
    cmd = ARCCmd()
    cmd.cirPoint.x = cirPoint[0];cmd.cirPoint.y = cirPoint[1];cmd.cirPoint.z = cirPoint[2];cmd.cirPoint.rHead = cirPoint[3]
    cmd.toPoint.x = toPoint[0];cmd.toPoint.y = toPoint[1];cmd.toPoint.z = toPoint[2];cmd.toPoint.rHead = toPoint[3]
    queuedCmdIndex = c_uint64(0)
    while(True):
        result = api.SetARCCmd(byref(cmd), isQueued, byref(queuedCmdIndex))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    return [queuedCmdIndex.value]

#****************************** WAIT功能 ****************************#
# 执行时间等待指令
def SetWAITCmd(api, waitTime, isQueued=0):
    param = WAITCmd()
    param.waitTime = int(waitTime * 1000)
    queuedCmdIndex = c_uint64(0)
    while(True):
        result = api.SetWAITCmd(byref(param), isQueued, byref(queuedCmdIndex))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(100)
            continue
        break
    return [queuedCmdIndex.value]

# 指令触发指令
def SetTRIGCmd(api, address, mode,  condition,  threshold,  isQueued=0):
    param = TRIGCmd()
    param.address = address
    param.mode = mode
    param.condition = condition
    param.threshold = threshold
    queuedCmdIndex = c_uint64(0)
    while(True):
        result = api.SetTRIGCmd(byref(param), isQueued, byref(queuedCmdIndex))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    return [queuedCmdIndex.value]

#****************************** EIO功能 ****************************#
# 设置I/O复用
def SetIOMultiplexing(api, address, multiplex, isQueued=0):
    param = IOMultiplexing()
    param.address = address
    param.multiplex = multiplex
    queuedCmdIndex = c_uint64(0)
    while(True):
        result = api.SetIOMultiplexing(byref(param), isQueued, byref(queuedCmdIndex))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    return [queuedCmdIndex.value]

# 读取I/O复用
def GetIOMultiplexing(api,  addr):
    param = IOMultiplexing()
    param.address = addr
    while(True):
        result = api.GetIOMultiplexing(byref(param))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    output('GetIOMultiplexing: address=%.4f multiplex=%.4f' %(param.address,  param.multiplex))
    return [param.multiplex]

# 设置I/O输出电平
def SetIODO(api, address, level, isQueued=0):
    param = IODO()
    param.address = address
    param.level = level
    queuedCmdIndex = c_uint64(0)
    while(True):
        result = api.SetIODO(byref(param), isQueued, byref(queuedCmdIndex))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    return [queuedCmdIndex.value]

# 读取I/O输出电平
def GetIODO(api,  addr):
    param = IODO()
    param.address = addr
    while(True):
        result = api.GetIODO(byref(param))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    output('GetIODO: address=%.4f level=%.4f' %(param.address,  param.level))
    return [param.level]

# 设置PWM输出
def SetIOPWM(api, address, frequency, dutyCycle,  isQueued=0):
    param = IOPWM()
    param.address = address
    param.frequency = frequency
    param.dutyCycle = dutyCycle
    queuedCmdIndex = c_uint64(0)
    while(True):
        result = api.SetIOPWM(byref(param), isQueued, byref(queuedCmdIndex))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    return [queuedCmdIndex.value]

# 读取PWM输出
def GetIOPWM(api,  addr):
    param = IOPWM()
    param.address = addr
    while(True):
        result = api.GetIOPWM(byref(param))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    output('GetIOPWM: address=%.4f frequency=%.4f dutyCycle=%.4f' %(param.address,  param.frequency,  param.dutyCycle))
    return [param.frequency,  param.dutyCycle]

# 读取I/O输入电平
def GetIODI(api,  addr):
    param = IODI()
    param.address = addr
    while(True):
        result = api.GetIODI(byref(param))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    output('GetIODI: address=%d level=%d' %(param.address,  param.level))
    return [param.level]

# 读取A/D输入
def GetIOADC(api,  addr):
    param = IOADC()
    param.address = addr
    while(True):
        result = api.GetIOADC(byref(param))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    output('GetIOADC: address=%.4f value=%.4f' %(param.address,  param.value))
    return [param.value]

# 设置扩展电机速度    
def SetEMotor(api, index, isEnabled, speed,  isQueued=0):
    emotor = EMotor()
    emotor.index = index
    emotor.isEnabled = isEnabled
    emotor.speed = speed
    queuedCmdIndex = c_uint64(0)
    while(True):
        result = api.SetEMotor(byref(emotor), isQueued, byref(queuedCmdIndex))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    return [queuedCmdIndex.value]

# Ex扩展函数,该套函数会检测每一条指令运行完毕
def SetEMotorEx(api, index, isEnabled, speed,  isQueued=0):
    ret = SetEMotor(api, index, isEnabled, speed,  isQueued)
    while(True):
        if ret[0] <= GetQueuedCmdCurrentIndex(api)[0]:
            break;
        dSleep(5)

# 设置扩展电机速度和移动距离    
def SetEMotorS(api, index, isEnabled, deltaPulse, isQueued=0):
    emotor = EMotorS()
    emotor.index = index
    emotor.isEnabled = isEnabled
    emotor.deltaPulse = deltaPulse
    queuedCmdIndex = c_uint64(0)
    while(True):
        result = api.SetEMotor(byref(emotor), isQueued, byref(queuedCmdIndex))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    return [queuedCmdIndex.value]

def SetEMotorSEx(api, index, isEnabled, speed, deltaPulse,  isQueued=0):
    ret = SetEMotorS(api, index, isEnabled, speed, deltaPulse,   isQueued)
    while(True):
        if ret[0] <= GetQueuedCmdCurrentIndex(api)[0]:
            break;
        dSleep(5)

# 光电传感器/颜色传感器接口说明：
# GP1接口: infraredPort=0/colorPort=0
# GP2接口: infraredPort=1/colorPort=1
# GP4接口: infraredPort=2/colorPort=2
# GP5接口: infraredPort=3/colorPort=3
# V1版本光电传感器/颜色传感器：version=0
# V2版本光电传感器/颜色传感器：version=1
# 使能光电传感器 
def SetInfraredSensor(api, isEnable, infraredPort, version=0):
    enable = c_bool(isEnable)
    port = c_uint8(infraredPort)
    version = c_uint8(version)
    while(True):
        result = api.SetInfraredSensor(enable, port, version)
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break

# 获取光电传感器数值  
def GetInfraredSensor(api, infraredPort):
    port = c_uint8(infraredPort)
    value = c_ubyte(0)
    
    while(True):
        result = api.GetInfraredSensor(port,  byref(value))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    return [value.value]

# 示例：实时读取光电传感器数值
# 采用V1版本光电传感器，连接机械臂的GP4接口
# while True:
#     dType.SetInfraredSensor(api, isEnable=1, infraredPort=2, version=0)
#     result = dType.GetInfraredSensor(api, infraredPort=2)[0]
#     print(result)
#     dType.dSleep(500) 

# 使能颜色传感器
def SetColorSensor(api, isEnable, colorPort, version=0):
    enable = c_bool(isEnable)
    port = c_uint8(colorPort)
    version = c_uint8(version)
    while(True):
        result = api.SetColorSensor(enable, port, version)
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break

# 获取颜色传感器数值   
def GetColorSensor(api):
    r = c_int8(0)
    g = c_int8(0)
    b = c_int8(0)
    while(True):
        result = api.GetColorSensor(byref(r),  byref(g),  byref(b))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    return [r.value, g.value, b.value]

# 获取颜色传感器指定颜色数值
def GetColorSensorEx(api, index):
    result = GetColorSensor(api)
    return result[index]

# 示例：实时读取颜色传感器数值
# 采用V2版本颜色传感器，连接机械臂的GP2接口
# while True:
#     dType.SetColorSensor(api, isEnable=1, colorPort=1, version=1)
#     result = dType.GetColorSensorEx(api, index=0)  
#     print(result)
#     dType.dSleep(200) 

#****************************** CAL功能 ****************************#
# 设置角度传感器静态偏差
def SetAngleSensorStaticError(api,  rearArmAngleError, frontArmAngleError):
    c_rearArmAngleError = c_float(rearArmAngleError)
    c_frontArmAngleError = c_float(frontArmAngleError)
    while(True):
        result = api.SetAngleSensorStaticError(c_rearArmAngleError, c_frontArmAngleError)
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break

# 读取角度传感器静态偏差       
def GetAngleSensorStaticError(api):
    rearArmAngleError = c_float(0)
    frontArmAngleError = c_float(0)
    while(True):
        result = api.GetAngleSensorStaticError(byref(rearArmAngleError),  byref(frontArmAngleError))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    output('GetAngleSensorStaticError: rearArmAngleError=%.4f,frontArmAngleError=%.4f' %(rearArmAngleError.value, frontArmAngleError.value))
    return [rearArmAngleError.value, frontArmAngleError.value]

# 设置角度传感器线性化参数    
def SetAngleSensorCoef(api,  rearArmAngleCoef, frontArmAngleCoef):
    c_rearArmAngleCoef = c_float(rearArmAngleCoef)
    c_frontArmAngleCoef = c_float(frontArmAngleCoef)
    while(True):
        result = api.SetAngleSensorCoef(c_rearArmAngleCoef, c_frontArmAngleCoef)
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break

# 读取角度传感器线性化参数        
def GetAngleSensorCoef(api):
    rearArmAngleCoef = c_float(0)
    frontArmAngleCoef = c_float(0)
    while(True):
        result = api.GetAngleSensorCoef(byref(rearArmAngleCoef),  byref(frontArmAngleCoef))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    output('GetAngleSensorStaticCoef: rearArmAngleCoef=%.4f,frontArmAngleCoef=%.4f' %(rearArmAngleCoef.value, frontArmAngleCoef.value))
    return [rearArmAngleCoef.value, frontArmAngleCoef.value]

# 设置基座编码器静态偏差
def SetBaseDecoderStaticError(api,  baseDecoderError):
    c_baseDecoderError = c_float(baseDecoderError)
    while(True):
        result = api.SetBaseDecoderStaticError(c_baseDecoderError)
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break

# 读取基座编码器静态偏差  
def GetBaseDecoderStaticError(api):
    baseDecoderError = c_float(0)
    while(True):
        result = api.GetBaseDecoderStaticError(byref(baseDecoderError))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    output('GetBaseDecoderStaticError: baseDecoderError=%.4f' %(baseDecoderError.value))
    return [baseDecoderError.value]

