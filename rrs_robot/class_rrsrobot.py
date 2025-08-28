import sys
sys.path.append("../../src")

import time
import struct
import serial
import fashionstar_uart_sdk as uservo
import math
import time

class rrsrobot:
    def __init__(self):
        self.SERVO_PORT_NAME =  '/dev/ttyUSB0'		# 舵机串口号
        self.SERVO_BAUDRATE = 115200			# 舵机的波特率
        self.SERVO_HAS_MTURN_FUNC = False	# 舵机是否拥有多圈模式

        # 初始化串口
        uart = serial.Serial(port=self.SERVO_PORT_NAME, baudrate=self.SERVO_BAUDRATE,\
                            parity=serial.PARITY_NONE, stopbits=1,\
                            bytesize=8,timeout=0)
        # 初始化舵机管理器
        self.control = uservo.UartServoManager(uart, is_debug=True)

        # 连杆长度数组L = [底部, 下连杆, 上连杆, 顶盘]
        self.L = [0.056, 0.070, 0.102, 0.090]
        #初期姿勢(theta, phi, h)
        self.ini_pos = [0, 0, 0.13]
        self.h_max = 0.15
        self.h_min = 0.10
        self.phi_max = 20
        
        self.delay_time = 0

    # 逆运动学计算
    def kinema_inv(self, n, h):
        L = self.L
        #サーボ基準時の高さPmz導出(Pmzで+-反転)
        A = (L[0]+L[1])/h
        B = (h**2+L[2]**2-(L[0]+L[1])**2-L[3]**2)/(2*h)
        C = A**2+1
        D = 2*(A*B-(L[0]+L[1]))
        E = B**2+(L[0]+L[1])**2-L[2]**2
        Pmx = (-D+math.sqrt(D**2-4*C*E))/(2*C)
        Pmz = math.sqrt(L[2]**2-Pmx**2+2*(L[0]+L[1])*Pmx-(L[0]+L[1])**2)

        # 0号舵机球形关节坐标推导
        x0 = (L[3]/(math.sqrt(n[0]**2 + n[2]**2)))*(n[2])
        y0 = 0
        z0 = h + (L[3]/(math.sqrt(n[0]**2 + n[2]**2)))*(-n[0])
        S0 = [x0, y0, z0] # 球形关节S0

        A = (L[0]-S0[0])/S0[2]
        B = (S0[0]**2+S0[1]**2+S0[2]**2-L[2]**2-L[0]**2+L[1]**2)/(2*S0[2])
        C = A**2+1
        D = 2*(A*B-L[0])
        E = B**2+L[0]**2-L[1]**2
        a0 = (-D+math.sqrt(D**2-4*C*E))/(2*C)
        b0 = 0
        c0 = math.sqrt(L[1]**2-a0**2+2*L[0]*a0-L[0]**2)
        if ( z0 < h):
            c0 = -c0
        R0 = [a0, b0, c0] # 旋转关节R0
        theta0 = -math.degrees(math.atan2(R0[2] , R0[0]-L[0])) # 舵机旋转角度theta0

        # 1号舵机球形关节坐标推导
        x1 = (L[3]/(math.sqrt(n[0]**2+3*n[1]**2+4*n[2]**2+2*math.sqrt(3)*n[0]*n[1])))*(-n[2])
        y1 = (L[3]/(math.sqrt(n[0]**2+3*n[1]**2+4*n[2]**2+2*math.sqrt(3)*n[0]*n[1])))*(-math.sqrt(3)*n[2])
        z1 = h + (L[3]/(math.sqrt(n[0]**2+3*n[1]**2+4*n[2]**2+2*math.sqrt(3)*n[0]*n[1])))*(math.sqrt(3)*n[1]+n[0])
        S1 = [x1, y1, z1] # 球形关节S1

        A = -(S1[0]+math.sqrt(3)*S1[1]+2*L[0])/S1[2]
        B = (S1[0]**2+S1[1]**2+S1[2]**2+L[1]**2-L[2]**2-L[0]**2)/(2*S1[2])
        C = A**2+4
        D = 2*A*B+4*L[0]
        E = B**2+L[0]**2-L[1]**2
        a1 = (-D-math.sqrt(D**2-4*C*E))/(2*C)
        b1 = math.sqrt(3)*a1
        c1 = math.sqrt(L[1]**2-4*a1**2-4*L[0]*a1-L[0]**2)
        if (z1 < h):
            c1 = -c1
        R1 = [a1, b1, c1] #旋转关节R1
        theta1 = - math.degrees(math.atan2(R1[2], math.sqrt(R1[0]**2+R1[1]**2)-L[0])) # 舵机旋转角度theta1

        # 2号舵机球形关节坐标推导
        x2 = (L[3]/(math.sqrt(n[0]**2+3*n[1]**2+4*n[2]**2-2*math.sqrt(3)*n[0]*n[1])))*(-n[2])
        y2 = (L[3]/(math.sqrt(n[0]**2+3*n[1]**2+4*n[2]**2-2*math.sqrt(3)*n[0]*n[1])))*(math.sqrt(3)*n[2])
        z2 = h + (L[3]/(math.sqrt(n[0]**2+3*n[1]**2+4*n[2]**2-2*math.sqrt(3)*n[0]*n[1])))*(-math.sqrt(3)*n[1]+n[0])
        S2 = [x2, y2, z2] # 球形关节S2

        A = -(S2[0]-math.sqrt(3)*S2[1]+2*L[0])/S2[2]
        B = (S2[0]**2+S2[1]**2+S2[2]**2+L[1]**2-L[2]**2-L[0]**2)/(2*S2[2])
        C = A**2+4
        D = 2*A*B+4*L[0]
        E = B**2+L[0]**2-L[1]**2
        a2 = (-D-math.sqrt(D**2-4*C*E))/(2*C)
        b2 = -math.sqrt(3)*a2
        c2 = math.sqrt(L[1]**2-4*a2**2-4*L[0]*a2-L[0]**2)
        if (z2 < h):
            c2 = -c2
        R2 = [a2, b2, c2] # 旋转关节R2
        theta2 = - math.degrees(math.atan2(R2[2], math.sqrt(R2[0]**2+R2[1]**2)-L[0]))
        thetas = [theta0, theta1, theta2]
        return thetas

    def control_t_posture(self, pos ,t):
        theta = pos[0]
        phi = pos[1]
        # 动作制约
        if phi > self.phi_max:
            phi = self.phi_max
        h = pos[2]
        if h > self.h_max:
            h = self.h_max
        elif h < self.h_min:
            h = self.h_min 
        z = math.cos(math.radians(phi))
        r = math.sin(math.radians(phi))
        x = r*math.cos(math.radians(theta))
        y = r*math.sin(math.radians(theta))
        n = [x, y, z]
        angles = self.kinema_inv(n, h)
        print(angles)
        self.control.set_servo_angle(0, max(min(angles[0],-10.0),-75.0), interval=self.delay_time) # 设置舵机角度 极速模式
        self.control.set_servo_angle(1, max(min(angles[1],-10.0),-75.0), interval=self.delay_time) # 设置舵机角度 极速模式
        self.control.set_servo_angle(2, max(min(angles[2],-10.0),-75.0), interval=self.delay_time) # 设置舵机角度 极速模式
        #self.control.wait() # 等待舵机静止
        time.sleep(t)
    
    def Initialize_posture(self):
        pos = self.ini_pos
        t = 1.0
        self.control_t_posture(pos,t)
