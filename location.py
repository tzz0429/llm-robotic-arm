import DllType as dType
import os
import time

# 定义机械臂状态,用键值存储
CON_STR = {
    dType.DobotConnect.DobotConnect_NoError:  "DobotConnect_NoError",
    dType.DobotConnect.DobotConnect_NotFound: "DobotConnect_NotFound",
    dType.DobotConnect.DobotConnect_Occupied: "DobotConnect_Occupied"}

# 获取当前文件所在目录的绝对路径
current_dir = os.path.dirname(os.path.abspath(__file__))
# 设置 DLL 文件的完整路径
dll_path = os.path.join(current_dir, "DobotDll.dll")
# 加载机械臂动态链接库
api = dType.load(dll_path)

# 连接机械臂
state = dType.ConnectDobot(api, "COM3", 115200)[0]
# 打印机械臂连接状态
print("Connect status:",CON_STR[state])

# 设置机械臂PTP运动模式参数(门型运动时抬升高度为20, 最大高度为30)
dType.SetPTPJumpParams(api, 20, 30, isQueued=0)
# 设置PTP运动速度百分比和加速度百分比，默认30
dType.SetPTPCommonParams(api, 30, 30, isQueued=0)

# 获取当前位置
current_pose = dType.GetPose(api)
print("Current position:", current_pose)

# 向右移动10mm (Y轴增加10)
target_x = current_pose[0]
target_y = current_pose[1] + 100  # 向右移动10mm
target_z = current_pose[2]
target_r = current_pose[3]

# 执行PTP运动 (PTPMOVJXYZMode=1 表示关节运动)
dType.SetPTPCmd(api, 1, target_x, target_y, target_z, target_r, isQueued=1)
# 等待运动完成
time.sleep(1)
dType.SetEndEffectorSuctionCup(api, 1, 1, isQueued=1) 
time.sleep(10)

# 断开连接
dType.DisconnectDobot(api)