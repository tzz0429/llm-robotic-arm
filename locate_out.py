import DobotDllType as dType
import os

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
print("Connect status:", CON_STR[state])

# 获取当前位置
current_pose = dType.GetPose(api)
print("\n当前机械臂终端坐标：")
print(f"X坐标: {current_pose[0]:.2f} mm")
print(f"Y坐标: {current_pose[1]:.2f} mm")
print(f"Z坐标: {current_pose[2]:.2f} mm")
print(f"R角度: {current_pose[3]:.2f} 度")

# 断开连接
dType.DisconnectDobot(api)
