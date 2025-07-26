"""
****************** VLM指导机械臂运动 ***********************
# @Time      : 2025/6/21
# @Author    : Tzz
# @File name : general.py
# Demo说明   : 使用VLM指导机械臂完成复杂任务
*******************************************************
"""
import time
import cv2
import numpy as np
import os
from openai import OpenAI
import base64
import DobotDllType as dType
import config
import re

# 定义机械臂状态
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

# 创建image目录（如果不存在）
image_dir = os.path.join(current_dir, "image")
if not os.path.exists(image_dir):
    os.makedirs(image_dir)

# 图像保存路径
image_path = os.path.join(image_dir, "image.png")

# 图像三点坐标
# a = np.array([[203, 199, 1],
#               [429, 194, 1],
#               [223, 406, 1]
#             ])
a = config.a

# 机械臂三点坐标
# b = np.array([[156.26, 113, 1],
#               [184.90, 115.29, 1],
#               [325.03, -1.98, 1]
#             ])
b = config.b


def encode_image(path):
    """将图片编码为base64格式"""
    with open(path, "rb") as f:
        return base64.b64encode(f.read()).decode()


def add_coordinate_system(img_path, output_path, grid_size=100):
    """为图片添加坐标系，原点在左上角"""
    img = cv2.imread(img_path)
    if img is None:
        raise ValueError("无法读取图像")
    
    h, w = img.shape[:2]
    overlay = np.ones_like(img) * 255
    
    # 绘制网格线
    grid_color = (80, 180, 80)
    for x in range(0, w, grid_size):
        cv2.line(overlay, (x, 0), (x, h), grid_color, 2)
    for y in range(0, h, grid_size):
        cv2.line(overlay, (0, y), (w, y), grid_color, 2)
    
    # 绘制坐标轴（从左上角开始）
    cv2.line(overlay, (0, 0), (w, 0), (0, 0, 220), 3)  # X轴
    cv2.line(overlay, (0, 0), (0, h), (220, 0, 0), 3)  # Y轴
    
    # 添加坐标标签
    font = cv2.FONT_HERSHEY_DUPLEX
    label_bg_color = (240, 240, 240)
    
    # X轴标签
    for x in range(0, w, grid_size):
        text = f"{x}"
        text_size = cv2.getTextSize(text, font, 0.6, 1)[0]
        cv2.rectangle(overlay, 
                      (x - 5, 10), 
                      (x + text_size[0] + 5, 10 + text_size[1] + 5), 
                      label_bg_color, -1)
        cv2.putText(overlay, text, (x, 10 + text_size[1]), 
                   font, 0.6, (0, 0, 0), 1, cv2.LINE_AA)
    
    # Y轴标签
    for y in range(0, h, grid_size):
        text = f"{y}"
        text_size = cv2.getTextSize(text, font, 0.6, 1)[0]
        cv2.rectangle(overlay, 
                      (10, y - 5), 
                      (10 + text_size[0] + 15, y + text_size[1] + 5), 
                      label_bg_color, -1)
        cv2.putText(overlay, text, (10, y + text_size[1]), 
                   font, 0.6, (0, 0, 0), 1, cv2.LINE_AA)
    
    # 添加原点标记
    cv2.circle(overlay, (0, 0), 6, (0, 200, 200), -1)
    cv2.putText(overlay, "O", (10, 20), font, 0.7, (10, 10, 200), 2, cv2.LINE_AA)
    
    # 添加边界标记
    cv2.putText(overlay, f"W: {w}px", (w - 120, 30), font, 0.7, (0, 0, 0), 2)
    cv2.putText(overlay, f"H: {h}px", (w - 120, 70), font, 0.7, (0, 0, 0), 2)
    
    # 智能叠加
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    avg_brightness = np.mean(gray)
    alpha = 0.25 if avg_brightness < 128 else 0.35
    result = cv2.addWeighted(overlay, alpha, img, 1 - alpha, 0)
    
    cv2.imwrite(output_path, result)
    print(f"坐标系增强完成！输出保存至: {output_path}")


def camera(cap):
    """捕获图像"""
    width = 640
    height = 480
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    image_saved = False
    while True:
        ret, img = cap.read()
        if not ret:
            print("无法获取图像")
            break
        cv2.imshow('camera', img)
        key = cv2.waitKey(1)
        if key == ord(" "):
            cv2.imwrite(image_path, img)
            image_saved = True
            print("图像已保存到:", image_path)
            break
        if key == 27:
            break   
    cv2.destroyAllWindows()
    return image_saved


def get_vlm_guidance(image_path, prompt):
    """获取VLM的指导"""
    client = OpenAI(
        api_key="sk-zk20b881405b980fd6cdd6b1a2d3f945a7168516bc334b29",
        base_url="https://api.zhizengzeng.com/v1"
    )
    
    b64 = encode_image(image_path)

    resp = client.responses.create(
        model="o3",
        tools=[
            {
                "type": "code_interpreter",
                "container": {"type": "auto"}
            }
        ],
        instructions=("你是一个代码执行助手，你每次都应该使用pillow和opencv库为用户解决问题，并且你输出答案的时候要严格按照要求格式输出"
                      "请你给出每次移动的原始位置和目标位置坐标，要求坐标非常精确，以如下格式输出：\n"
                      "(x1,y1)(x2,y2)(x3,y3)(x4,y4)... 依次表示每一对操作\n"
                      "例如前两对表示第1次抓取从(x1,y1) → (x2,y2)，第2次为(x3,y3) → (x4,y4)，以此类推。\n"
                      "不要加入说明文字，只输出坐标即可。"
                      ),
        input=[
            {
                "role": "user",
                "content": [
                    {"type": "input_text", "text": prompt},
                    {"type": "input_image", "image_url": f"data:image/jpeg;base64,{b64}"},
                ],
            }
        ],
    )

    return resp.output_text


# 实现多目标抓取
def extract_multiple_coordinates(vlm_response):
    """
    提取多个坐标对，格式假设为连续的 (x1,y1)(x2,y2)...，两两配对为 source→target
    """
    try:
        # 匹配所有坐标对
        matches = re.findall(r'\(([-\d.]+),\s*([-\d.]+)\)', vlm_response)
        print(matches)
        if len(matches) % 2 != 0:
            print("坐标对数不是偶数，无法两两配对")
            return []

        coords = [(float(x), float(y)) for x, y in matches]
        tasks = [(coords[i], coords[i + 1]) for i in range(0, len(coords), 2)]  # 两两配对
        return tasks  # list of (source_center, target_center)

    except Exception as e:
        print(f"多坐标提取错误: {str(e)}")
        return []


def Dobot():
    """初始化机械臂"""
    state = dType.ConnectDobot(api, "COM3", 115200)[0]
    print("Connect status:", CON_STR[state])
    dType.SetEndEffectorParams(api, 59.7, 0, 0, isQueued=0)
    dType.SetPTPJumpParams(api, 20, 30, isQueued=0)
    dType.SetPTPCommonParams(api, 30, 30, isQueued=0)
    dType.SetQueuedCmdClear(api)
    dType.SetQueuedCmdStartExec(api)
    dType.SetPTPCmd(api, 0, 150, -150, 50, 0, isQueued=1)
    print('Starting...')


def move_to_position(x, y, z=-65.8, mode=1):
    """移动机械臂到指定位置
    mode: 0 - JUMP模式（门型运动）
          1 - MOVJ模式（关节运动）
          2 - MOVL模式（直线运动）
    """
    # 计算机械臂坐标
    X = np.linalg.solve(a, b)
    a1 = np.array([x, y, 1])
    b1 = np.dot(a1, X)
    
    # 移动到目标位置
    dType.SetPTPCmd(api, mode, b1[0], b1[1], z, 0, isQueued=1)

def execute_task(prompt, height):
    """执行VLM指导的任务"""
    # 初始化机械臂
    Dobot()
    
    print('请将目标物体放置在摄像头正下方')
    print('开始捕获图像，按空格键确认拍照，ESC键关闭窗口')
    
    cap = cv2.VideoCapture(1)
    while True:
        # 捕获图像
        image_saved = camera(cap)
        if image_saved:
            # 添加坐标系
            add_coordinate_system(image_path, image_path)
            print("\nVLM运行中...")
            while True:
                # 获取VLM指导
                vlm_response = get_vlm_guidance(image_path, prompt)
                print("\nVLM响应:", vlm_response)
                tasks = extract_multiple_coordinates(vlm_response)
                height = -70 + float(height)
                if tasks:
                    print(f"\n共提取到 {len(tasks)} 个任务：")
                    for i, (source, target) in enumerate(tasks):
                        print(f"任务{i + 1}: Source = {source}, Target = {target}")

                    user_input = input("\n是否执行机械臂操作？(yes/no): ").lower()
                    if user_input == 'yes':
                        for i, (source, target) in enumerate(tasks):
                            print(f"\n---- 执行第 {i + 1} 个抓取任务 ----")
                            move_to_position(source[0], source[1], -20, mode=0)  # 上升到安全高度
                            move_to_position(source[0], source[1], height, mode=2)  # 下降到物体拾取位置
                            dType.SetEndEffectorSuctionCup(api, 1, 1, isQueued=1)  # 打开吸盘
                            move_to_position(source[0], source[1], -20, mode=2)  # 上升到安全高度

                            move_to_position(target[0], target[1], -20, mode=0)  # 移动到目标位置
                            move_to_position(target[0], target[1], height+5, mode=2)  # 下降到安全高度
                            dType.SetEndEffectorSuctionCup(api, 0, 0, isQueued=1)  # 关闭吸盘
                            move_to_position(target[0], target[1], -20, mode=2)  # 上升到安全高度

                        print("\n所有任务完成，机械臂返回初始位置")
                        dType.SetPTPCmd(api, 0, 150, -150, 50, 0, isQueued=1)  # 返回初始位置
                        cap.release()  # 释放摄像头
                        cv2.destroyAllWindows()  # 销毁窗口
                        return  # 直接跳出 execute_task()

                    else:
                        # 询问是否重新获取VLM指导
                        retry = input("是否重新获取VLM指导？(yes/no): ").lower()
                        if retry != 'yes':
                            break
                else:
                    print("无法提取有效坐标")
                    retry = input("是否重新获取VLM指导？(yes/no): ").lower()
                    if retry != 'yes':
                        break
            
            time.sleep(1)
        else:
            break


if __name__ == "__main__":
    # 示例提示词
    prompt = input("请输入任务内容，例如“请将红色方块移到左上角目标区域”：\n")
    height = input("请输入需要拾取物品的大致高度(单位mm):\n")
    execute_task(prompt, height)
