import sys
import os
import struct
import time
import numpy as np
import scipy.linalg
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QLabel, QLineEdit, QPushButton, QGroupBox, QTextEdit,
                             QTabWidget)
from PyQt5.QtCore import QTimer
import pyqtgraph as pg
import serial
import serial.tools.list_ports

class MotorControllerGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("运动控制系统上位机")
        self.resize(1100, 700)
        
        self.serial_port = None
        self.is_testing = False
        self.active_mode = None # "POS" or "SPD"
        
        self.motor_params = {
            'K_val': '5.36e-08', 'Bv': '0', 'Cm': '0.02051933',
            'L': '0.00540926', 'J': '0.0000133722', 'Ce': '0.020701',
            'R_res': '0.109872', 'Ts': '0.001'
        }
        self.pos_lqr_params = {'Q_pos': '5', 'Q_vel': '0.001', 'Q_cur': '0.1', 'R_ctrl': '10'}
        self.spd_lqr_params = {'Q_vel': '100', 'Q_cur': '0.001', 'R_ctrl': '10'}
        
        self.init_ui()
        self.update_port_list()

        
        
    def init_ui(self):
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        layout = QHBoxLayout(main_widget)
        
        # 左侧面板
        control_panel = QVBoxLayout()
        layout.addLayout(control_panel, 1)
        
        # 1. 串口配置
        serial_group = QGroupBox("串口配置")
        serial_layout = QHBoxLayout()
        self.port_combo = QLineEdit()
        self.port_combo.setPlaceholderText("例如 COM3")
        serial_layout.addWidget(QLabel("串口:"))
        serial_layout.addWidget(self.port_combo)
        self.btn_connect = QPushButton("连接")
        self.btn_connect.clicked.connect(self.toggle_serial)
        serial_layout.addWidget(self.btn_connect)
        serial_group.setLayout(serial_layout)
        control_panel.addWidget(serial_group)
        
        #电机参数
        motor_group = QGroupBox("电机物理参数(共享)")
        motor_layout = QVBoxLayout()
        self.motor_inputs = {}
        row = None
        for i, (key, val) in enumerate(self.motor_params.items()):
            if i % 4 == 0:
                row = QHBoxLayout()
                motor_layout.addLayout(row)
            row.addWidget(QLabel(f"{key}:"))
            inp = QLineEdit(val)
            self.motor_inputs[key] = inp
            row.addWidget(inp)
        motor_group.setLayout(motor_layout)
        control_panel.addWidget(motor_group)

        self.tabs = QTabWidget()
        control_panel.addWidget(self.tabs)
        
        # Tab 1: 位控
        self.tab_pos = QWidget()
        pos_layout = QVBoxLayout(self.tab_pos)
        pos_param_group = QGroupBox("位置 LQR 参数")
        pos_param_layout = QHBoxLayout()
        self.pos_lqr_inputs = {}
        for key, val in self.pos_lqr_params.items():
            pos_param_layout.addWidget(QLabel(f"{key}:"))
            inp = QLineEdit(val)
            self.pos_lqr_inputs[key] = inp
            pos_param_layout.addWidget(inp)
        self.btn_calc_pos = QPushButton("下发 参数")
        self.btn_calc_pos.clicked.connect(self.calculate_and_send_pos)
        pos_param_layout.addWidget(self.btn_calc_pos)
        pos_param_group.setLayout(pos_param_layout)
        pos_layout.addWidget(pos_param_group)
        
        pos_test_group = QGroupBox("位控运行")
        pos_test_layout = QHBoxLayout()
        self.target_pos_input = QLineEdit("1080")
        pos_test_layout.addWidget(QLabel("位置(度):"))
        pos_test_layout.addWidget(self.target_pos_input)
        self.btn_test_pos = QPushButton("开始控制")
        self.btn_test_pos.clicked.connect(lambda: self.toggle_test("POS"))
        pos_test_layout.addWidget(self.btn_test_pos)
        pos_test_group.setLayout(pos_test_layout)
        pos_layout.addWidget(pos_test_group)
        pos_layout.addStretch()
        self.tabs.addTab(self.tab_pos, "位置控制")
        
        # Tab 2: 速控
        self.tab_spd = QWidget()
        spd_layout = QVBoxLayout(self.tab_spd)
        spd_param_group = QGroupBox("速度 LQR 参数")
        spd_param_layout = QHBoxLayout()
        self.spd_lqr_inputs = {}
        for key, val in self.spd_lqr_params.items():
            spd_param_layout.addWidget(QLabel(f"{key}:"))
            inp = QLineEdit(val)
            self.spd_lqr_inputs[key] = inp
            spd_param_layout.addWidget(inp)
        self.btn_calc_spd = QPushButton("下发 参数")
        self.btn_calc_spd.clicked.connect(self.calculate_and_send_spd)
        spd_param_layout.addWidget(self.btn_calc_spd)
        spd_param_group.setLayout(spd_param_layout)
        spd_layout.addWidget(spd_param_group)
        
        spd_test_group = QGroupBox("速控运行")
        spd_test_layout = QHBoxLayout()
        self.target_spd_input = QLineEdit("50")
        spd_test_layout.addWidget(QLabel("速度(rad/s):"))
        spd_test_layout.addWidget(self.target_spd_input)
        self.btn_test_spd = QPushButton("开始控制")
        self.btn_test_spd.clicked.connect(lambda: self.toggle_test("SPD"))
        spd_test_layout.addWidget(self.btn_test_spd)
        spd_test_group.setLayout(spd_test_layout)
        spd_layout.addWidget(spd_test_group)
        spd_layout.addStretch()
        self.tabs.addTab(self.tab_spd, "速度控制")
        
        # 日志
        log_group = QGroupBox("日志")
        log_layout = QVBoxLayout()
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        log_layout.addWidget(self.log_text)
        log_group.setLayout(log_layout)
        control_panel.addWidget(log_group)
        
        # 右侧图表
        plot_panel = QVBoxLayout()
        layout.addLayout(plot_panel, 3)
        status_group = QGroupBox("状态")
        status_layout = QHBoxLayout()
        self.lbl_angle = QLabel("当前角度: 0.00 °")
        self.lbl_speed = QLabel("当前速度: 0.00 °/s")
        self.lbl_current = QLabel("当前电流: 0")
        self.lbl_voltage = QLabel("当前电压: 0")
        self.lbl_angle.setStyleSheet("font-size: 16px; font-weight: bold; color: blue;")
        self.lbl_speed.setStyleSheet("font-size: 16px; font-weight: bold; color: green;")
        self.lbl_current.setStyleSheet("font-size: 16px; font-weight: bold; color: red;")
        self.lbl_voltage.setStyleSheet("font-size: 16px; font-weight: bold; color: orange;")
        status_layout.addWidget(self.lbl_angle)
        status_layout.addWidget(self.lbl_speed)
        status_layout.addWidget(self.lbl_current)
        status_layout.addWidget(self.lbl_voltage)
        status_group.setLayout(status_layout)
        plot_panel.addWidget(status_group)
        
        self.plot_widget = pg.PlotWidget(title="运行数据")
        self.plot_widget.setLabel('left', '值')
        self.plot_widget.setLabel('bottom', '时间(s)')
        self.plot_widget.addLegend()
        self.plot_main = self.plot_widget.plot(pen='y', name='主数据(Pos/Spd)')
        self.voltage_curve = self.plot_widget.plot(pen='r', name='电压')
        plot_panel.addWidget(self.plot_widget)
        
        self.time_data = []
        self.main_data = []
        self.vol_data = []
        self.start_time = time.time()
        
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(1)

    def log(self, msg):
        self.log_text.append(f"[{time.strftime('%H:%M:%S')}] {msg}")
        self.log_text.verticalScrollBar().setValue(self.log_text.verticalScrollBar().maximum())

    def update_port_list(self):
        ports = [port.device for port in serial.tools.list_ports.comports()]
        if ports:
            self.port_combo.setText(ports[0])

    def toggle_serial(self):
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            self.btn_connect.setText("连接串口")
            self.log("断开串口")
        else:
            try:
                self.serial_port = serial.Serial(self.port_combo.text(), 115200, timeout=0.1)
                self.btn_connect.setText("断开串口")
                self.log("连接成功！")
            except Exception as e:
                self.log(f"串口错误: {e}")

    def get_motor_params(self):
        return {k: float(v.text()) for k, v in self.motor_inputs.items()}

    def calculate_and_send_pos(self):
        try:
            m = self.get_motor_params()
            Q_pos = float(self.pos_lqr_inputs['Q_pos'].text())
            Q_vel = float(self.pos_lqr_inputs['Q_vel'].text())
            Q_cur = float(self.pos_lqr_inputs['Q_cur'].text())
            R_ctrl = float(self.pos_lqr_inputs['R_ctrl'].text())
            
            A = np.array([[0, 1, 0],
                          [-m['K_val']/m['J'], -m['Bv']/m['J'], m['Cm']/m['J']],
                          [0, -m['Ce']/m['L'], -m['R_res']/m['L']]])
            B = np.array([[0], [0], [1/m['L']]])
            C = np.eye(3)
            
            sys_d = scipy.signal.StateSpace(A, B, C, np.zeros((3,1))).to_discrete(m['Ts'], method='zoh')
            Ad, Bd = sys_d.A, sys_d.B
            
            Q = np.diag([Q_pos, Q_vel, Q_cur])
            R_w = np.array([[R_ctrl]])
            
            P = scipy.linalg.solve_discrete_are(Ad, Bd, Q, R_w)
            K = np.linalg.inv(Bd.T @ P @ Bd + R_w) @ (Bd.T @ P @ Ad)
            K = -K[0]
            Ad_cl = Ad + np.dot(Bd, K.reshape(1,3))
            
            M_noise = np.diag([1e-6, 1e-4, 1e-2])
            N_noise = np.diag([1e-5, 1e-3, 1e-1])
            Sigma = scipy.linalg.solve_discrete_are(Ad.T, C.T, M_noise, N_noise)
            Sigma_bar = Sigma - Sigma @ C.T @ np.linalg.inv(C @ Sigma @ C.T + N_noise) @ C @ Sigma
            Ld = Sigma_bar @ C.T @ np.linalg.inv(N_noise)
            
            N_bar = (np.linalg.pinv(Bd) @ (np.eye(3) - Ad) @ np.array([[1],[0],[0]]))[0,0]
            
            self.log("--- 【位置控制】矩阵计算完成 ---")
            self.log(f"K 矩阵: {np.array2string(K, precision=4)}")
            self.log(f"Kalman Ld:\n{np.array2string(Ld, precision=4)}")
            self.log(f"Ad_cl 闭环:\n{np.array2string(Ad_cl, precision=4)}")
            self.log(f"前馈 N_bar: {N_bar:.6g}")

            packed = struct.pack('<BB' + 'f'*22 + 'B', 0xBF, 0x06, 
                K[0], K[1], K[2],
                Ld[0,0], Ld[0,1], Ld[0,2],
                Ld[1,0], Ld[1,1], Ld[1,2],
                Ld[2,0], Ld[2,1], Ld[2,2],
                Ad_cl[0,0], Ad_cl[0,1], Ad_cl[0,2],
                Ad_cl[1,0], Ad_cl[1,1], Ad_cl[1,2],
                Ad_cl[2,0], Ad_cl[2,1], Ad_cl[2,2],
                N_bar, 0xFF)
            if self.serial_port and self.serial_port.is_open:
                self.serial_port.write(packed)
                self.log("✅ 位控 LQR参数(0x06)已下发！")
            else:
                self.log("⚠️ 串口未接！")
        except Exception as e:
            self.log(f"❌ 异常: {e}")

    def calculate_and_send_spd(self):
        try:
            m = self.get_motor_params()
            Q_vel = float(self.spd_lqr_inputs['Q_vel'].text())
            Q_cur = float(self.spd_lqr_inputs['Q_cur'].text())
            R_ctrl = float(self.spd_lqr_inputs['R_ctrl'].text())
            
            A = np.array([[-m['Bv']/m['J'], m['Cm']/m['J']],
                          [-m['Ce']/m['L'], -m['R_res']/m['L']]])
            B = np.array([[0], [1/m['L']]])
            C = np.eye(2)
            
            sys_d = scipy.signal.StateSpace(A, B, C, np.zeros((2,1))).to_discrete(m['Ts'], method='zoh')
            Ad, Bd = sys_d.A, sys_d.B
            
            Q = np.diag([Q_vel, Q_cur])
            R_w = np.array([[R_ctrl]])
            
            P = scipy.linalg.solve_discrete_are(Ad, Bd, Q, R_w)
            K = (np.linalg.inv(Bd.T @ P @ Bd + R_w) @ (Bd.T @ P @ Ad))[0]
            
            M_noise = np.diag([1e-4, 1e-2])
            N_noise = np.diag([1e-3, 1e-1])
            Sigma = scipy.linalg.solve_discrete_are(Ad.T, C.T, M_noise, N_noise)
            Sigma_bar = Sigma - Sigma @ C.T @ np.linalg.inv(C @ Sigma @ C.T + N_noise) @ C @ Sigma
            Ld = Sigma_bar @ C.T @ np.linalg.inv(N_noise)
            
            C_tracker = np.array([[1, 0]])
            Z = np.block([[Ad - np.eye(2), Bd], [C_tracker, np.zeros((1,1))]])
            N_vec = np.linalg.inv(Z) @ np.array([[0], [0], [1]])
            Nx_2 = N_vec[1, 0]
            Nu = N_vec[2, 0]
            
            Ad_cl = Ad - np.outer(Bd, K)

            self.log("--- 【速度控制】矩阵计算完成 ---")
            self.log(f"K 矩阵: {np.array2string(K, precision=4)}")
            self.log(f"Kalman Ld:\n{np.array2string(Ld, precision=4)}")
            self.log(f"Nx_2: {Nx_2:.6f}, Nu: {Nu:.6f}")
            self.log(f"闭环模型 Ad_cl:\n{np.array2string(Ad_cl, precision=4)}")

            packed = struct.pack('<BB' + 'f'*12 + 'B', 0xBF, 0x08, 
                K[0], K[1],
                Ld[0,0], Ld[0,1],
                Ld[1,0], Ld[1,1],
                Ad_cl[0,0], Ad_cl[0,1],
                Ad_cl[1,0], Ad_cl[1,1],
                Nx_2, Nu, 0xFF)
            if self.serial_port and self.serial_port.is_open:
                self.serial_port.write(packed)
                self.log("✅ 速控 LQR参数(0x08)已下发！")
            else:
                self.log("⚠️ 串口未接！")
        except Exception as e:
            self.log(f"❌ 异常: {e}")

    def toggle_test(self, mode):
        if not self.serial_port or not self.serial_port.is_open:
            self.log("⚠️ 串口未接！")
            return
        
        # 切换状态
        if self.is_testing and self.active_mode == mode:
            # 停止
            self.is_testing = False
            self.active_mode = None
            if mode == "POS":
                self.btn_test_pos.setText("开始控制")
            else:
                self.btn_test_spd.setText("开始控制")
            
            packed = struct.pack('>BBBhhhB', 0xBF, 0x01, 0x00, 0, 0, 0, 0xFF)
            self.serial_port.write(packed)
            self.log("⏹️ 停止运行！已下发 0占空比 强制停止。")
        else:
            # 如果另一个模式在跑先停止
            if self.is_testing:
                self.btn_test_pos.setText("开始控制")
                self.btn_test_spd.setText("开始控制")
                
            self.is_testing = True
            self.active_mode = mode
            self.time_data = []
            self.main_data = []
            self.vol_data = []
            self.start_time = time.time()
            
            if mode == "POS":
                self.btn_test_pos.setText("停止控制")
                deg = float(self.target_pos_input.text())
                pulse = int(deg / 360.0 * 1200.0)
                packed = struct.pack('>BBBhhhB', 0xBF, 0x02, 0x00, pulse, 0, 0, 0xFF)
                self.serial_port.write(packed)
                self.log(f"▶️ 位控，目标 {deg}° ({pulse}脉冲)")
            else:
                self.btn_test_spd.setText("停止控制")
                deg_spd = float(self.target_spd_input.text())
                pulse = int(deg_spd)
                packed = struct.pack('>BBBhhhB', 0xBF, 0x03, 0x00, 0, pulse, 0, 0xFF)
                self.serial_port.write(packed)
                self.log(f"▶️ 速控，目标 {deg_spd}°/s ({pulse}脉冲/s)")

    def update_plot(self):
        if self.serial_port and self.serial_port.in_waiting >= 7:
            while self.serial_port.in_waiting >= 7:
                header = self.serial_port.read(1)
                if header == b'\xaf':
                    if self.serial_port.in_waiting < 10: break
                    data = self.serial_port.read(10)
                    if len(data) == 10 and data[-1] == 0xFF:
                        code, delta_n, current, voltage, target, tail = struct.unpack('<BhhhhB', data)
                        delta_deg = delta_n / 1200.0 * 360.0 * 3.14 / 180.0
                        dt = (time.time() - self.last_time) if hasattr(self, 'last_time') else 0
                        print(dt)
                        speed = delta_deg / dt if dt>0 else getattr(self, 'last_speed', 0.0)
                        self.last_speed = speed
                        speed = speed
                        self.last_time = time.time()
                        self.lbl_speed.setText(f"当前速度: {speed:.2f} rad/s")
                        self.lbl_current.setText(f"当前电流: {current}")
                elif header == b'\xbf':
                    if self.serial_port.in_waiting < 10: break
                    data = self.serial_port.read(10)
                    if len(data) == 10 and data[0] == 0x07 and data[-1] == 0xFF:
                        code, path_rad, command, tail = struct.unpack('<BffB', data)
                        pos_deg = path_rad / (2 * np.pi) * 360.0
                        self.lbl_angle.setText(f"当前角度: {pos_deg:.2f} °")
                        self.lbl_voltage.setText(f"当前电压: {command:.2f} V")
                        
                        if self.is_testing:
                            self.time_data.append(time.time() - self.start_time)
                            # 根据当前控制模式，绘图主线切为对应的值
                            if self.active_mode == "POS":
                                self.main_data.append(pos_deg)
                            else:
                                self.main_data.append(getattr(self, 'last_speed', 0.0))
                            self.vol_data.append(command)
                            if len(self.time_data) > 1500:
                                self.time_data.pop(0)
                                self.main_data.pop(0)
                                self.vol_data.pop(0)
            if self.is_testing:
                self.plot_main.setData(self.time_data, self.main_data)
                self.voltage_curve.setData(self.time_data, self.vol_data)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MotorControllerGUI()
    window.show()
    sys.exit(app.exec_())
