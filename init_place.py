import DobotDllType as dType
import os
import cv2
import re

CONFIG_PATH = "config.py"

def remove_var_from_config(var_names):
    """移除config.py中已有的变量定义"""
    if not os.path.exists(CONFIG_PATH):
        return
    with open(CONFIG_PATH, "r", encoding="utf-8") as f:
        lines = f.readlines()
    new_lines = []
    skip = False
    for line in lines:
        # 检查是否是要移除的变量
        for var in var_names:
            if re.match(rf"\s*{var}\s*=", line):
                skip = True
        # 跳过变量定义块
        if skip:
            if line.strip().endswith("]") or line.strip().endswith("}"):
                skip = False
            continue
        new_lines.append(line)
    with open(CONFIG_PATH, "w", encoding="utf-8") as f:
        f.writelines(new_lines)

def save_board_places(board_places):
    # 覆盖已有变量
    remove_var_from_config(["board_place_robot"])
    with open(CONFIG_PATH, "a", encoding="utf-8") as f:
        f.write("\nboard_place_robot = {\n")
        for k, v in board_places.items():
            if isinstance(k, str):
                f.write(f"    '{k}': [{v[0]:.2f}, {v[1]:.2f}, -46],\n")
            else:
                f.write(f"    {k}: [{v[0]:.2f}, {v[1]:.2f}, -46],\n")
        f.write("}\n")

def save_matrix(name, mat):
    # 覆盖已有变量
    remove_var_from_config([name])
    with open(CONFIG_PATH, "a", encoding="utf-8") as f:
        f.write(f"\n{name} = [\n")
        for row in mat:
            f.write(f"    [{row[0]:.2f}, {row[1]:.2f}, {row[2]:.2f}],\n")
        f.write("]\n")

def get_pose(api):
    pose = dType.GetPose(api)
    return pose[0], pose[1]

def init_board_places(api):
    print("请依次将机械臂末端移动到棋盘1-9格和棋篓格中心，每次移动后按回车。")
    board_places = {}
    for i in range(1, 10):
        input(f"请移动到第{i}格中心并回车...")
        x, y = get_pose(api)
        board_places[i] = [x, y]
        print(f"已记录第{i}格: x={x:.2f}, y={y:.2f}")
    input("请移动到棋篓格并回车...")
    x, y = get_pose(api)
    board_places['basket'] = [x, y]
    print(f"已记录棋篓格: x={x:.2f}, y={y:.2f}")
    save_board_places(board_places)
    print("所有棋盘格和棋篓格坐标已写入config.py。")

def camera_capture(filename="abc_img.png", cam_id=1):
    print('摄像头预览中，按空格拍照，ESC退出...')
    cap = cv2.VideoCapture(cam_id)
    width = 640
    height = 480
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    while True:
        ret, frame = cap.read()
        if not ret:
            print("摄像头读取失败")
            continue
        cv2.imshow('camera', frame)
        key = cv2.waitKey(1)
        if key == 32:  # 空格键
            cv2.imwrite(filename, frame)
            print("已保存图片:", filename)
            break
        elif key == 27:  # ESC
            print("已取消拍照")
            break
    cap.release()
    cv2.destroyAllWindows()

def init_robot_points(api):
    print("请依次将机械臂末端移动到A、B、C三点，每次移动后按回车。")
    b = []
    for name in ['A', 'B', 'C']:
        input(f"请移动到{name}点并回车...")
        x, y = get_pose(api)
        b.append([x, y, 1])
        print(f"已记录{name}点: x={x:.2f}, y={y:.2f}")
    save_matrix('b', b)
    print("机械臂三点坐标已写入config.py。")

def init_camera_points():
    # 拍照并标点
    camera_capture("abc_img.png", cam_id=1)
    img = cv2.imread("abc_img.png")
    points = []

    print("请用鼠标依次点击图像上的A、B、C三点。")
    def on_mouse(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            points.append([x, y, 1])
            cv2.circle(img, (x, y), 5, (0, 0, 255), -1)
            cv2.imshow("img", img)
            if len(points) == 3:
                cv2.destroyAllWindows()

    cv2.imshow("img", img)
    cv2.setMouseCallback("img", on_mouse)
    cv2.waitKey(0)
    save_matrix('a', points)
    print("图像三点像素坐标已写入config.py。")

def main():
    # base_dir = os.path.dirname(os.path.abspath(__file__))
    # print(base_dir)
    dll_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "DobotDll.dll")
    api = dType.load(dll_path)
    state = dType.ConnectDobot(api, "COM3", 115200)[0]
    if state != dType.DobotConnect.DobotConnect_NoError:
        print("机械臂连接失败")
        return
    print("初始化菜单：\n1. 初始化棋盘九格和棋篓格\n2. 机械臂标定\n3. 摄像头标定")
    choice = input("请输入1或2或3：")
    if choice == "1":
        init_board_places(api)
    elif choice == "2":
        init_robot_points(api)
    elif choice == "3":
        init_camera_points()
    else:
        print("无效输入")
    dType.DisconnectDobot(api)

if __name__ == "__main__":
    main()