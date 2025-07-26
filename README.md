# llm-robotic-arm
Only need to input natural language to operate the robotic arm to perform multi-task, multi-objective, and multi-stage picking and placing operations.<br>
项目初期， 通过使用Pillow与OpenCV库基于颜色阈值分割识别彩色方块与目标区域并自动标注其像素坐标。随后，调用OpenAI的GPT-4o模型api， 结合使用者自定义Prompt实现目标物体与投放区域的坐标提取，指导机械臂完成精准拾取与投放。（例如： 将圆珠笔前面的方块移动到其后面） 。 <br>
在此基础上，项目进一步引入OpenAI最新发布的GPT-o3推理模型，突破坐标标注预处理的限制，使模型能够直接从原始图像中识别多种样式的目标物体并通过code interpreter自动生成并运行代码，输出任务相关物体坐标，大幅提升系统的任务泛化能力。（例如： 规划解决三阶汉诺塔） 。 最终实现了基于自然语言的多任务、多目标、多阶段机械臂智能拾取操作。<br>
<img width="1896" height="876" alt="图片1" src="https://github.com/user-attachments/assets/063eb6f5-461c-4694-876d-3cd93097957e" />
<img width="2110" height="450" alt="图片2" src="https://github.com/user-attachments/assets/a9779e68-768f-4be6-9f80-99e711ba3421" />
# 运行实例：
![6d4425ba3ac82b127cdc3cb617c2972](https://github.com/user-attachments/assets/61a576f8-de99-457a-a382-2b32043b849b)
<img width="400" height="390" alt="图片4" src="https://github.com/user-attachments/assets/685123d4-02e6-4e13-8175-1a0332d76e73" />
<img width="2110" height="1172" alt="图片3" src="https://github.com/user-attachments/assets/c77e8bf8-4f70-43ae-b9bf-eef54dff2f65" />
