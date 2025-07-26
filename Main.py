"""
****************** 实现垃圾分类 ***********************
# @Time      : 2025/6/1
# @Author    : Jiaxing Wu
# @File name : Main.py
# Demo说明   : 
*******************************************************
"""
# -*- coding: utf-8 -*-
import time
import cv2
import numpy as np
import torch
import torchvision
import torch.nn as nn
import torch.nn.functional as F
from torchvision import models, transforms
from PIL import ImageFont, ImageDraw, Image
import DllType as dType
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

# 创建image目录（如果不存在）
image_dir = os.path.join(current_dir, "image")
if not os.path.exists(image_dir):
    os.makedirs(image_dir)

# 图像保存路径
image_path = os.path.join(image_dir, "image.png")

# 图像三点坐标
a = np.array([[245, 253, 1 ],
              [288, 144, 1 ],
              [333, 294, 1 ]
            ])
print(a)

# 机械臂三点坐标
b = np.array([[183.17, 45.66, 1 ],
              [150.30, 110.35, 1 ],
              [235.35, 72.02, 1 ]
            ])
print(b)

# 图片裁剪,scope为需要截取的轮廓的范围
# 形式为[x1,x2,y1,y2], (x1,y1)为左上点,（x2,y2）为右下点
def crop_image(image,scope):
    x1 = scope[0]
    x2 = scope[1]
    y1 = scope[2]
    y2 = scope[3]
    img = image[y1:y2, x1:x2]
    return img

# 返回图片轮廓的总面积
def area_summax(image):
    s = []
    # 检测到图片的轮廓和继承关系
    contours, hierarchy = cv2.findContours(image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    for i in range(len(contours)):
        cnt = contours[i]
        # 计算轮廓的面积
        area = cv2.contourArea(cnt)
        s.append(area)
    area = 0
    for x in s:
        area += x
    return area

# RGB图转灰度图
def select_image(image):
    # cv2.split函数分离得到各个通道的灰度值(单通道图像)
    b, g, r = cv2.split(image)
    # cv2.GaussianBlur()函数用高斯滤波器（GaussianFilter）对图像进行平滑处理,高斯核大小为（5,5）
    blurB = cv2.GaussianBlur(b, (5, 5), 0)
    blurG = cv2.GaussianBlur(g, (5, 5), 0)
    blurR = cv2.GaussianBlur(r, (5, 5), 0)
    # cv2.threshold图像阈值处理 阈值100 最大值255 返回ret得到的阈值,th阈值化后的图像
    ret1, th1 = cv2.threshold(blurB, 100, 255, cv2.THRESH_BINARY)
    ret2, th2 = cv2.threshold(blurG, 100, 255, cv2.THRESH_BINARY)
    ret3, th3 = cv2.threshold(blurR, 100, 255, cv2.THRESH_BINARY)
    areamax = []
    areamax.append(area_summax(th1))
    areamax.append(area_summax(th2))
    areamax.append(area_summax(th3))
    index = areamax.index(max(areamax))
    if index == 0: 
        gray = b
    elif index == 1:
        gray = g
    elif index == 2:
        gray = r
    return gray

# 输出参数：img为Mat类型原始图像, tups为包含元组的数组, 每个元组顺序为截取图像中心点、左上点、右下点
def image_cut(imagesource):
    images = []
    tups = []
    img = cv2.cvtColor(imagesource, cv2.COLOR_BGR2RGB)  
    # shape返回的是图像的高,宽,色彩通道数
    h_s, w_s, _ = img.shape   
    # 进行滤波去掉噪声        
    blured = cv2.blur(img, (3, 3))   
    mask = np.zeros((h_s + 2, w_s + 2), np.uint8)
    cv2.floodFill(blured, mask, (w_s - 1, h_s - 1), (255, 255, 255), (1, 1, 1), (5, 5, 5), 8)
    # 显示灰度图
    gray = select_image(blured)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (15, 15))
    # 开闭运算，先开运算去除背景噪声，再继续闭运算填充目标内的孔洞
    opened = cv2.morphologyEx(gray, cv2.MORPH_OPEN, kernel)
    closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, kernel)
    # 图像腐蚀
    ret, binary = cv2.threshold(closed, 160, 255, cv2.THRESH_BINARY)
    erode = cv2.erode(binary, kernel)
    # 找到轮廓
    contours, hierarchy = cv2.findContours(erode, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    area_max = np.multiply(h_s - 10, w_s - 10)
    area_min = np.multiply(h_s / 20, w_s / 20)
    # print(len(contours))
    for i in range(len(contours)):
        scope = []
        cnt = contours[i]
        area = cv2.contourArea(cnt)
        # 处理分割轮廓区域，这个区域的最小值为拍摄产品尺寸大于整个图像的1/20
        if (area > area_min and area < area_max):
            x, y, w, h = cv2.boundingRect(cnt)
            M = cv2.moments(cnt)
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            rec_origin_x = x
            rec_origin_y = y
            rec_corner_x = x + w
            rec_corner_y = y + h
            if w < (w_s *0.7) and h < (h_s*0.7):
                scope.append(rec_origin_x)
                scope.append(rec_corner_x)
                scope.append(rec_origin_y)
                scope.append(rec_corner_y)
                img_cut = crop_image(img, scope)
                cv2.rectangle(img, (scope[0], scope[2]), (scope[1], scope[3]), (0, 255, 0), 2)
                tup = [cx, cy, rec_origin_x, rec_origin_y, rec_corner_x, rec_corner_y]
                tups.append(tup)
                images.append(img_cut)
                continue
    return img,tups

# 模型推理
def modelpred(cropimg):
    model_eval = models.resnet18(pretrained=False)
    num_ftrs = model_eval.fc.in_features
    model_eval.fc = nn.Linear(num_ftrs, 4)
    # 0: 可回收垃圾, 1: 干垃圾, 2: 有害垃圾, 3: 湿垃圾
    classes = ('Recyclable Waste','Residual Waste','Hazardous Waste','Household Food Waste')
    # 加载训练模型
    device = torch.device('cpu')
    model_eval.load_state_dict(torch.load('./GarbageModel-20200305.pkl', map_location=device))
    model_eval.eval()
    img = Image.fromarray(cv2.cvtColor(cropimg, cv2.COLOR_BGR2RGB))
    
    tsfrm = transforms.Compose([
        transforms.Grayscale(3),
        transforms.Resize((120, 120)),
        transforms.ToTensor(),
        transforms.Normalize([0.5, 0.5, 0.5], [0.5, 0.5, 0.5])
    ])
    img = tsfrm(img)
    img = img.to(device)
    img = img.unsqueeze(0)  # 图片扩展多一维,因为输入到保存的模型中是4维的[batch_size,通道,长,宽]
                            # 而普通图片只有三维，[通道,长,宽]
    output = model_eval(img)
    prob = F.softmax(output, dim=1)          # prob是4个分类的概率
    print(prob)                              # 打印识别概率
    value, predicted = torch.max(output.data, 1)
    label = predicted.numpy()[0]
    # print(label)                           # 打印分类标签
    pred_class = classes[predicted.item()]
    print(pred_class)                        # 打印分类名称
    return label

# 图片添加文字
def addtext(n,cropimg):
    if n==0:
        text = "可回收垃圾\nRecyclable Waste"
    elif n==1:
        text = "干垃圾\nResidual Waste" 
    elif n==2:
        text = "有害垃圾\nHazardous Waste"
    elif n==3:
        text = "湿垃圾\nHousehold Food Waste"
    # 将分割后的图片高和宽分别赋值给x1,y1
    x1, y1 = cropimg.shape[0:2]
    # 图片放大到原来的5倍，输出尺寸格式为（宽,高）
    enlarge_img = cv2.resize(cropimg, (int(y1*5), int(x1*5)))
    fontpath = "font/simsun.ttc"
    font = ImageFont.truetype(fontpath, 32)  
    img_pil = Image.fromarray(enlarge_img)
    draw = ImageDraw.Draw(img_pil)
    # 绘制文字信息
    draw.text((10, 100), text, font=font, fill = (0, 0, 255))
    bk_img = np.array(img_pil)
    cv2.imshow('frame', bk_img)
    # 持续显示5s
    key = cv2.waitKey(3000)
    cv2.destroyAllWindows()

def camera(cap):
    width = 640
    height = 480
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    image_saved = False
    while True:
        # 捕获视频帧
        ret, img = cap.read()
        if not ret:
            print("无法获取图像")
            break
        cv2.imshow('camera', img)
        # 保持画面持续
        key = cv2.waitKey(1)
        # 空格键保存
        if key == ord(" "):
            cv2.imwrite(image_path, img)
            image_saved = True
            print("图像已保存到:", image_path)
            break
        # Esc退出
        if key == 27:
            break   
    # 关闭窗口
    cv2.destroyAllWindows()
    return image_saved

def Dobot():
    # 连接机械臂
    state = dType.ConnectDobot(api, "COM3", 115200)[0]
    # 打印机械臂连接状态
    print("Connect status:",CON_STR[state])
    # 设置机械臂末端为吸盘
    dType.SetEndEffectorParams(api, 59.7, 0, 0, isQueued=0)
    # 设置机械臂PTP运动模式参数(门型运动时抬升高度为20, 最大高度为30)
    dType.SetPTPJumpParams(api, 20, 30, isQueued=0)
    # 设置PTP运动速度百分比和加速度百分比，默认30
    dType.SetPTPCommonParams(api, 30, 30, isQueued=0)
    # 初始化清空机械臂的指令
    dType.SetQueuedCmdClear(api)
    # 开始执行队列指令
    dType.SetQueuedCmdStartExec(api)
    # 机械臂初始位置
    dType.SetPTPCmd(api, 0, 150, -150, 50, 0, isQueued=1)
    print('Starting...')

def show():
    # 使用绝对路径读取图像
    image_source = cv2.imread(image_path)
    if image_source is None:
        print("无法读取图像文件:", image_path)
        return
    # 返回图像中心点、左上角和右下角的像素坐标
    img, tupS = image_cut(image_source)
    cv2.imshow('img', img)
    cv2.waitKey(3000) 
    cv2.destroyAllWindows()
    for c in range(len(tupS)):
        # 按顺序显示截取图像，按任意键更新
        if len(tupS) == 0:
            break
        # 找出分割后的子图片框选出来
        crop_img = image_source[tupS[c][3]: tupS[c][5], tupS[c][2]: tupS[c][4]]
        # 模型预测,图片分类
        label = modelpred(crop_img)
        # 打印分类标签
        print(label)  
        # 图片添加文字      
        addtext(label,crop_img)
        # 分别打印各图像中心点坐标
        x = tupS[c][0];
        y = tupS[c][1];
        print('x', x)
        print('y', y)
        # 机械臂坐标转换
        X = np.linalg.solve(a,b)
        # dot()返回的是两个数组的点积
        a1 = np.array([x,y,1])
        b1 = np.dot(a1,X)
        # 打印转换后的机械臂坐标
        print(b1[0], b1[1])
        dType.SetPTPCmd(api, 0, 250, -100, -20, 0, isQueued=1)
        # 物品抓取点 
        dType.SetPTPCmd(api, 1, b1[0], b1[1], -20, 0, isQueued=1) 
        # Z轴下压   
        dType.SetPTPCmd(api, 1, b1[0], b1[1], -65, 0, isQueued=1)
         # 打开气泵          
        dType.SetEndEffectorSuctionCup(api, 1, 1, isQueued=1)           
        if label == 0:
            # 可回收垃圾放置区域
            dType.SetPTPCmd(api, 0, 46, -235, -40, 0, isQueued=1)   
        elif label == 1:
            # 干垃圾放置区域
            dType.SetPTPCmd(api, 0, 93, -235, -40, 0, isQueued=1)                
        elif label == 2:
            # 有害垃圾放置区域
            dType.SetPTPCmd(api, 0, 141, -235, -40, 0, isQueued=1)       
        elif label == 3:
            # 湿垃圾放置区域
            dType.SetPTPCmd(api, 0, 192, -235, -40, 0, isQueued=1)    
        # 关闭气泵
        dType.SetEndEffectorSuctionCup(api, 0, 0, isQueued=1)  
        # 机械臂回到初始位置       
        dType.SetPTPCmd(api, 0, 150, -150, 50, 0, isQueued=1)  

# 主程序入口
if __name__ == "__main__":
    # 初始化机械臂
    Dobot()
    print('请把垃圾图片放置摄像头正下方')
    print('开始捕获图像，按空格键确认拍照，ESC键关闭窗口')
    # 0: 笔记本内置摄像头 1: USB摄像头
    cap = cv2.VideoCapture(1)
    while True:
        # 图像捕获
        image_saved = camera(cap)
        if image_saved:
            # 结果显示及机械臂运动
            show()
            time.sleep(1)
        else:
            break

