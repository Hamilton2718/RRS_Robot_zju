import class_rrsrobot
import class_Camera
import class_PID
import time
import threading
import numpy as np

# 创建锁
lock = threading.Lock()

# 画像サイズ（高さ、幅）とチャンネル数（ここでは3チャンネル = RGB）
height = 480
width = 640
channels = 3

# 空の画像を作成（全てのピクセルが0）
image = np.zeros((height, width, channels), dtype=np.uint8)

# PID系数
K_PID = [0.005, 0.000, 0.001] #0.015, 0.0001, 0.0051
k = 1
a = 1

# 创建对象
Robot = class_rrsrobot.rrsrobot()
camera = class_Camera.Camera()
pid = class_PID.PID(K_PID, k, a)

# 初始化
#Robot.set_up()
Robot.Initialize_posture()
pz_ini = Robot.ini_pos[2]
    
frame_count = 0
start_time = time.time()
img_start_time = time.time()
rob_start_time = time.time()
fps = 0  # 初期値として0を設定
img_fps = 0
rob_fps = 0


#ボールの座標
x = -1
y = -1
area = -1
goal = [-50, 50]

def get_img():
    global image, img_fps, img_start_time
    img_frame_count = 0
    while(1):
        new_image = camera.take_pic()
        if new_image is None:
            time.sleep(0.1)
            continue
        with lock:
            image = new_image
        #img_fpsの計算
        img_frame_count += 1
        if img_frame_count == 100:
            img_end_time = time.time()
            img_elapsed_time = img_end_time - img_start_time
            img_fps = 100 / img_elapsed_time
            img_start_time = img_end_time
            img_frame_count = 0

        

def cont_rob():
    global x, y, area, rob_fps, rob_start_time
    rob_frame_count = 0
    while(1):
        with lock:
            current_image = image.copy()
        try:
            x, y, area = camera.find_ball(current_image)
        except Exception as e:
            print(f"find_ball error: {e}")
            x, y, area = -1, -1, 0
        #rob_fpsの計算
        rob_frame_count += 1
        if rob_frame_count == 100:
            rob_end_time = time.time()
            rob_elapsed_time = rob_end_time - rob_start_time
            rob_fps = 100 / rob_elapsed_time
            rob_start_time = rob_end_time
            rob_frame_count = 0

try:
    camera_thread = threading.Thread(target=get_img)
    rob_thread = threading.Thread(target=cont_rob)
    camera_thread.start()
    rob_thread.start()

    while(1):
        if img_fps == 0:
            continue
        Current_value = [x, y, area]
        if x != -1:
            theta, phi = pid.compute(goal, Current_value)
            pos = [theta, phi, pz_ini]
            Robot.control_t_posture(pos, 0.01)
        print(f"img_fps: {img_fps}, rob_fps: {rob_fps}")

finally:
    #Robot.clean_up()
    camera.clean_up_cam()
