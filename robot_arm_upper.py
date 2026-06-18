import sys
import time
import serial
import serial.tools.list_ports

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import (
    QApplication,
    QMainWindow,
    QWidget,
    QLabel,
    QPushButton,
    QComboBox,
    QSlider,
    QTextEdit,
    QGridLayout,
    QHBoxLayout,
    QVBoxLayout,
    QGroupBox,
    QMessageBox,
    QRadioButton,
    QCheckBox,
    QLineEdit,
    QSpinBox,
)


SERVO_MIN = 75
SERVO_MAX = 225
SERVO_MID = 150


class RobotArmUpper(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("STM32 六轴机械臂 PyQt 上位机")
        self.resize(760, 560)

        self.ser = None
        self.pc_mode_enabled = False

        self.joint_values = [SERVO_MID] * 5
        self.joint_dirty = False
        self.updating_sliders = False

        self.init_ui()

        self.read_timer = QTimer(self)
        self.read_timer.timeout.connect(self.read_serial_data)
        self.read_timer.start(50)

        self.send_timer = QTimer(self)
        self.send_timer.timeout.connect(self.send_joint_if_dirty)
        self.send_timer.start(80)

        self.auto_send_timer = QTimer(self)
        self.auto_send_timer.timeout.connect(self.auto_send_tick)

        self.refresh_ports()

    def init_ui(self):
        central = QWidget()
        self.setCentralWidget(central)

        main_layout = QVBoxLayout()

        # 串口设置区
        serial_group = QGroupBox("串口设置")
        serial_layout = QHBoxLayout()

        self.port_combo = QComboBox()

        self.baud_combo = QComboBox()
        self.baud_combo.addItems(["9600", "19200", "38400", "57600", "115200"])
        self.baud_combo.setCurrentText("115200")

        self.data_bits_combo = QComboBox()
        self.data_bits_combo.addItems(["8", "7", "6", "5"])
        self.data_bits_combo.setCurrentText("8")

        self.parity_combo = QComboBox()
        self.parity_combo.addItems(["None", "Even", "Odd"])
        self.parity_combo.setCurrentText("None")

        self.stop_bits_combo = QComboBox()
        self.stop_bits_combo.addItems(["1", "1.5", "2"])
        self.stop_bits_combo.setCurrentText("1")

        self.flow_combo = QComboBox()
        self.flow_combo.addItems(["None", "RTS/CTS", "XON/XOFF"])
        self.flow_combo.setCurrentText("None")

        self.refresh_btn = QPushButton("刷新串口")
        self.connect_btn = QPushButton("连接")
        self.disconnect_btn = QPushButton("断开")

        self.refresh_btn.clicked.connect(self.refresh_ports)
        self.connect_btn.clicked.connect(self.connect_serial)
        self.disconnect_btn.clicked.connect(self.disconnect_serial)

        serial_layout.addWidget(QLabel("COM:"))
        serial_layout.addWidget(self.port_combo)

        serial_layout.addWidget(QLabel("Baud:"))
        serial_layout.addWidget(self.baud_combo)

        serial_layout.addWidget(QLabel("Data:"))
        serial_layout.addWidget(self.data_bits_combo)

        serial_layout.addWidget(QLabel("Parity:"))
        serial_layout.addWidget(self.parity_combo)

        serial_layout.addWidget(QLabel("Stop:"))
        serial_layout.addWidget(self.stop_bits_combo)

        serial_layout.addWidget(QLabel("Flow:"))
        serial_layout.addWidget(self.flow_combo)

        serial_layout.addWidget(self.refresh_btn)
        serial_layout.addWidget(self.connect_btn)
        serial_layout.addWidget(self.disconnect_btn)

        serial_group.setLayout(serial_layout)

        # 接收设置区
        recv_group = QGroupBox("接收设置")
        recv_layout = QHBoxLayout()

        self.recv_ascii_radio = QRadioButton("ASCII")
        self.recv_hex_radio = QRadioButton("Hex")
        self.recv_ascii_radio.setChecked(True)

        self.recv_auto_newline_cb = QCheckBox("自动换行")
        self.recv_auto_newline_cb.setChecked(True)

        self.recv_show_tx_cb = QCheckBox("显示发送")
        self.recv_show_tx_cb.setChecked(True)

        self.recv_show_time_cb = QCheckBox("显示时间")
        self.recv_show_time_cb.setChecked(True)

        recv_layout.addWidget(self.recv_ascii_radio)
        recv_layout.addWidget(self.recv_hex_radio)
        recv_layout.addWidget(self.recv_auto_newline_cb)
        recv_layout.addWidget(self.recv_show_tx_cb)
        recv_layout.addWidget(self.recv_show_time_cb)

        recv_group.setLayout(recv_layout)

        # 发送设置区
        send_group = QGroupBox("发送设置")
        send_layout = QHBoxLayout()

        self.send_ascii_radio = QRadioButton("ASCII")
        self.send_hex_radio = QRadioButton("Hex")
        self.send_ascii_radio.setChecked(True)

        self.auto_send_cb = QCheckBox("自动重发")

        self.auto_send_interval = QSpinBox()
        self.auto_send_interval.setRange(100, 10000)
        self.auto_send_interval.setValue(1000)
        self.auto_send_interval.setSingleStep(100)
        self.auto_send_interval.setSuffix(" ms")

        self.line_by_line_cb = QCheckBox("Line by Line")

        self.manual_send_edit = QLineEdit()
        self.manual_send_edit.setPlaceholderText("手动发送，例如 #MODE,PC!")

        self.manual_send_btn = QPushButton("发送")
        self.manual_send_btn.clicked.connect(self.manual_send)

        self.auto_send_cb.stateChanged.connect(self.on_auto_send_changed)
        self.auto_send_interval.valueChanged.connect(self.on_auto_send_interval_changed)

        send_layout.addWidget(self.send_ascii_radio)
        send_layout.addWidget(self.send_hex_radio)
        send_layout.addWidget(self.auto_send_cb)
        send_layout.addWidget(self.auto_send_interval)
        send_layout.addWidget(self.line_by_line_cb)
        send_layout.addWidget(self.manual_send_edit)
        send_layout.addWidget(self.manual_send_btn)

        send_group.setLayout(send_layout)

        # 模式控制区
        mode_group = QGroupBox("控制模式")
        mode_layout = QHBoxLayout()

        self.pc_mode_btn = QPushButton("进入 PC 控制模式")
        self.manual_mode_btn = QPushButton("回到手动模式")
        self.mid_btn = QPushButton("回中位")

        self.pc_mode_btn.clicked.connect(self.enter_pc_mode)
        self.manual_mode_btn.clicked.connect(self.enter_manual_mode)
        self.mid_btn.clicked.connect(self.go_mid)

        mode_layout.addWidget(self.pc_mode_btn)
        mode_layout.addWidget(self.manual_mode_btn)
        mode_layout.addWidget(self.mid_btn)

        mode_group.setLayout(mode_layout)

        # 关节滑条区
        joint_group = QGroupBox("关节 PWM 控制")
        joint_layout = QGridLayout()

        self.sliders = []
        self.value_labels = []

        for i in range(5):
            name_label = QLabel(f"J{i + 1}")
            value_label = QLabel(str(SERVO_MID))
            value_label.setAlignment(Qt.AlignCenter)

            slider = QSlider(Qt.Horizontal)
            slider.setMinimum(SERVO_MIN)
            slider.setMaximum(SERVO_MAX)
            slider.setValue(SERVO_MID)
            slider.setTickInterval(5)
            slider.setSingleStep(1)

            slider.valueChanged.connect(lambda value, idx=i: self.on_slider_changed(idx, value))

            self.sliders.append(slider)
            self.value_labels.append(value_label)

            joint_layout.addWidget(name_label, i, 0)
            joint_layout.addWidget(slider, i, 1)
            joint_layout.addWidget(value_label, i, 2)

        joint_group.setLayout(joint_layout)

        # 夹爪控制区
        gripper_group = QGroupBox("夹爪控制")
        gripper_layout = QHBoxLayout()

        self.grip_open_btn = QPushButton("夹爪打开")
        self.grip_close_btn = QPushButton("夹爪闭合")

        self.grip_open_btn.clicked.connect(lambda: self.send_command("#G,0!"))
        self.grip_close_btn.clicked.connect(lambda: self.send_command("#G,1!"))

        gripper_layout.addWidget(self.grip_open_btn)
        gripper_layout.addWidget(self.grip_close_btn)

        gripper_group.setLayout(gripper_layout)

        main_layout.addWidget(serial_group)
        main_layout.addWidget(recv_group)
        main_layout.addWidget(send_group)
        main_layout.addWidget(mode_group)

        # 日志区
        log_group = QGroupBox("串口日志")
        log_layout = QVBoxLayout()

        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)

        log_layout.addWidget(self.log_text)
        log_group.setLayout(log_layout)

        # 动作控制区
        action_group = QGroupBox("动作控制")
        action_layout = QHBoxLayout()

        self.record_btn = QPushButton("记录当前点")
        self.play_btn = QPushButton("回放动作")
        self.clear_btn = QPushButton("清空记录")

        self.record_btn.clicked.connect(lambda: self.send_command("#REC!"))
        self.play_btn.clicked.connect(lambda: self.send_command("#PLAY!"))
        self.clear_btn.clicked.connect(lambda: self.send_command("#CLR!"))

        action_layout.addWidget(self.record_btn)
        action_layout.addWidget(self.play_btn)
        action_layout.addWidget(self.clear_btn)

        action_group.setLayout(action_layout)

        main_layout.addWidget(action_group)
        main_layout.addWidget(joint_group)
        main_layout.addWidget(gripper_group)
        main_layout.addWidget(log_group)

        central.setLayout(main_layout)

    def log(self, text):
        if hasattr(self, "recv_show_time_cb") and self.recv_show_time_cb.isChecked():
            now = time.strftime("%H:%M:%S")
            self.log_text.append(f"[{now}] {text}")
        else:
            self.log_text.append(text)

    def refresh_ports(self):
        self.port_combo.clear()

        ports = serial.tools.list_ports.comports()
        for port in ports:
            self.port_combo.addItem(port.device)

        if len(ports) == 0:
            self.log("未检测到串口设备")
        else:
            self.log("串口列表已刷新")

    def connect_serial(self):
        if self.ser is not None and self.ser.is_open:
            self.log("串口已经连接")
            return

        port = self.port_combo.currentText()
        if not port:
            QMessageBox.warning(self, "提示", "未选择串口")
            return

        baud = int(self.baud_combo.currentText())

        data_bits_map = {
            "5": serial.FIVEBITS,
            "6": serial.SIXBITS,
            "7": serial.SEVENBITS,
            "8": serial.EIGHTBITS,
        }

        parity_map = {
            "None": serial.PARITY_NONE,
            "Even": serial.PARITY_EVEN,
            "Odd": serial.PARITY_ODD,
        }

        stop_bits_map = {
            "1": serial.STOPBITS_ONE,
            "1.5": serial.STOPBITS_ONE_POINT_FIVE,
            "2": serial.STOPBITS_TWO,
        }

        flow = self.flow_combo.currentText()

        try:
            self.ser = serial.Serial(
                port=port,
                baudrate=baud,
                bytesize=data_bits_map[self.data_bits_combo.currentText()],
                parity=parity_map[self.parity_combo.currentText()],
                stopbits=stop_bits_map[self.stop_bits_combo.currentText()],
                timeout=0.05,
                write_timeout=0.2,
                rtscts=(flow == "RTS/CTS"),
                xonxoff=(flow == "XON/XOFF"),
                dsrdtr=False,
            )

            self.ser.setDTR(False)
            self.ser.setRTS(False)
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()

            self.log(
                f"已连接 {port}, "
                f"{baud}, "
                f"{self.data_bits_combo.currentText()}, "
                f"{self.parity_combo.currentText()}, "
                f"{self.stop_bits_combo.currentText()}, "
                f"{flow}"
            )

        except Exception as e:
            QMessageBox.critical(self, "串口连接失败", str(e))
            self.log(f"串口连接失败: {e}")

    def disconnect_serial(self):
        if self.ser is not None:
            try:
                if self.ser.is_open:
                    self.ser.close()
                    self.log("串口已断开")
            except Exception as e:
                self.log(f"断开串口异常: {e}")

        self.ser = None
        self.pc_mode_enabled = False

    def send_command(self, cmd):
        if self.ser is None or not self.ser.is_open:
            self.log("发送失败：串口未连接")
            return False

        try:
            data = (cmd + "\r\n").encode("ascii")
            self.ser.write(data)
            self.ser.flush()

            if not hasattr(self, "recv_show_tx_cb") or self.recv_show_tx_cb.isChecked():
                self.log(f"TX: {cmd}")

            return True

        except Exception as e:
            self.log(f"发送失败: {e}")
            return False

    def send_raw_bytes(self, data, display_text=""):
        if self.ser is None or not self.ser.is_open:
            self.log("发送失败：串口未连接")
            return False

        try:
            self.ser.write(data)
            self.ser.flush()

            if not hasattr(self, "recv_show_tx_cb") or self.recv_show_tx_cb.isChecked():
                if display_text:
                    self.log(f"TX: {display_text}")
                else:
                    self.log(f"TX: {data}")

            return True

        except Exception as e:
            self.log(f"发送失败: {e}")
            return False

    def manual_send(self):
        text = self.manual_send_edit.text()

        if not text:
            return

        if self.line_by_line_cb.isChecked():
            lines = text.splitlines()
        else:
            lines = [text]

        for line in lines:
            if not line:
                continue

            if self.send_hex_radio.isChecked():
                try:
                    clean = line.replace(",", " ").replace("0x", "").replace("0X", "")
                    data = bytes.fromhex(clean)
                    self.send_raw_bytes(data, line)
                except ValueError:
                    self.log("Hex 发送失败：格式错误，例如 23 4D 4F 44")
            else:
                data = (line + "\r\n").encode("ascii", errors="ignore")
                self.send_raw_bytes(data, line)

    def auto_send_tick(self):
        self.manual_send()

    def on_auto_send_changed(self):
        if self.auto_send_cb.isChecked():
            interval = self.auto_send_interval.value()
            self.auto_send_timer.start(interval)
            self.log(f"自动重发开启，间隔 {interval} ms")
        else:
            self.auto_send_timer.stop()
            self.log("自动重发关闭")


    def on_auto_send_interval_changed(self, value):
        if self.auto_send_cb.isChecked():
            self.auto_send_timer.start(value)

    def read_serial_data(self):
        if self.ser is None or not self.ser.is_open:
            return

        try:
            data = self.ser.read(256)
            if not data:
                return

            if self.recv_hex_radio.isChecked():
                text = " ".join(f"{b:02X}" for b in data)
            else:
                text = data.decode(errors="ignore")

            if self.recv_auto_newline_cb.isChecked():
                text = text.replace("\r", "").replace("\n", "\nRX: ")

            self.log(f"RX: {text}")

        except Exception as e:
            self.log(f"读取串口失败: {e}")

    def enter_pc_mode(self):
        if self.send_command("#MODE,PC!"):
            self.pc_mode_enabled = True

    def enter_manual_mode(self):
        if self.send_command("#MODE,MANUAL!"):
            self.pc_mode_enabled = False

    def go_mid(self):
        self.updating_sliders = True

        for i in range(5):
            self.sliders[i].setValue(SERVO_MID)
            self.joint_values[i] = SERVO_MID
            self.value_labels[i].setText(str(SERVO_MID))

        self.updating_sliders = False

        self.send_command("#MID!")

    def on_slider_changed(self, idx, value):
        self.joint_values[idx] = value
        self.value_labels[idx].setText(str(value))

        if self.updating_sliders:
            return

        self.joint_dirty = True

    def send_joint_if_dirty(self):
        if not self.joint_dirty:
            return

        if not self.pc_mode_enabled:
            return

        if self.ser is None or not self.ser.is_open:
            return

        cmd = "#J,{},{},{},{},{}!".format(
            self.joint_values[0],
            self.joint_values[1],
            self.joint_values[2],
            self.joint_values[3],
            self.joint_values[4],
        )

        self.send_command(cmd)
        self.joint_dirty = False

    def closeEvent(self, event):
        self.disconnect_serial()
        event.accept()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = RobotArmUpper()
    window.show()
    sys.exit(app.exec_())