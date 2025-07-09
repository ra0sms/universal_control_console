#!venv/bin/python3

import sys
from PyQt5.QtWidgets import (QApplication, QMainWindow, QVBoxLayout, QHBoxLayout,
                             QPushButton, QWidget, QLabel, QComboBox,
                             QTextEdit, QGroupBox, QCheckBox)
from PyQt5.QtCore import Qt
import serial
import serial.tools.list_ports

class MicrocontrollerGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.serial = None
        self.setWindowTitle("Universal Control Console")
        self.setGeometry(100, 100, 400, 600)
        
        self.initUI()
        self.initSerial()

    def initUI(self):
        main_widget = QWidget()
        layout = QVBoxLayout()
        
        # COM Port Settings
        port_group = QGroupBox("COM Port Settings")
        port_layout = QHBoxLayout()
        
        self.port_combo = QComboBox()
        self.refresh_ports()
        port_layout.addWidget(QLabel("Port:"))
        port_layout.addWidget(self.port_combo)
        
        port_layout.addWidget(QLabel("Baud: 9600"))
        
        self.connect_btn = QPushButton("Connect")
        self.connect_btn.clicked.connect(self.toggle_connection)
        port_layout.addWidget(self.connect_btn)
        
        port_group.setLayout(port_layout)
        layout.addWidget(port_group)
        
        # Direction Buttons Configuration
        control_group = QGroupBox("Outputs Settings")
        control_layout = QVBoxLayout()
        
        # Directions in order: N, NE, E, SE, S, SW, W, NW
        directions = ['N', 'NE', 'E', 'SE', 'S', 'SW', 'W', 'NW']
        self.button_groups = []
        
        for btn_num, direction in enumerate(directions):
            hbox = QHBoxLayout()
            
            # Direction label
            label = QLabel(direction)
            label.setFixedWidth(40)
            font = label.font()
            font.setPointSize(12)
            font.setBold(True)
            label.setFont(font)
            label.setStyleSheet("color: #3366FF;")
            hbox.addWidget(label)
            
            # Checkboxes and bit numbers
            checkboxes = []
            for bit in range(8):
                # Create vertical layout for checkbox + bit number
                vbox = QVBoxLayout()
                vbox.setSpacing(0)
                
                # Checkbox
                checkbox = QCheckBox()
                checkbox.setFixedWidth(25)
                checkboxes.append(checkbox)
                vbox.addWidget(checkbox)
                
                # Bit number label
                bit_label = QLabel(f"{8-bit}")
                bit_label.setFixedWidth(25)
                bit_label.setAlignment(Qt.AlignCenter)
                vbox.addWidget(bit_label)
                
                hbox.addLayout(vbox)
            
            control_layout.addLayout(hbox)
            
            self.button_groups.append({
                'checkboxes': checkboxes,
                'direction': direction
            })
            
            # Connect signals
            for checkbox in checkboxes:
                checkbox.stateChanged.connect(lambda _, bn=btn_num: self.update_button_value(bn))
        
        # Control buttons
        btn_layout = QHBoxLayout()
        self.read_btn = QPushButton("Read Settings")
        self.read_btn.clicked.connect(self.read_settings)
        btn_layout.addWidget(self.read_btn)
        
        self.write_btn = QPushButton("Write Settings")
        self.write_btn.clicked.connect(self.write_settings)
        btn_layout.addWidget(self.write_btn)
        
        control_layout.addLayout(btn_layout)
        control_group.setLayout(control_layout)
        layout.addWidget(control_group)
        
        # Log
        self.log = QTextEdit()
        self.log.setReadOnly(True)
        layout.addWidget(QLabel("Log:"))
        layout.addWidget(self.log)
        
        main_widget.setLayout(layout)
        self.setCentralWidget(main_widget)
        
        self.update_ui_state(False)

    def initSerial(self):
        self.serial = None

    def refresh_ports(self):
        self.port_combo.clear()
        ports = serial.tools.list_ports.comports()
        for port in ports:
            self.port_combo.addItem(port.device)

    def toggle_connection(self):
        if self.serial and self.serial.is_open:
            self.serial.close()
            self.serial = None
            self.log_message("Disconnected")
            self.update_ui_state(False)
        else:
            try:
                port = self.port_combo.currentText()
                self.serial = serial.Serial(port, 9600, timeout=1)
                self.log_message(f"Connected to {port} at 9600 baud")
                self.update_ui_state(True)
            except Exception as e:
                self.log_message(f"Connection error: {str(e)}")

    def update_ui_state(self, connected):
        self.connect_btn.setText("Disconnect" if connected else "Connect")
        self.read_btn.setEnabled(connected)
        self.write_btn.setEnabled(connected)
        for group in self.button_groups:
            for checkbox in group['checkboxes']:
                checkbox.setEnabled(connected)

    def log_message(self, message):
        self.log.append(message)
        
    def send_command(self, command):
        if self.serial and self.serial.is_open:
            try:
                self.serial.write((command + "\n").encode('ascii'))
                return True
            except Exception as e:
                self.log_message(f"Send error: {str(e)}")
                return False
        return False

    def read_response(self):
        if self.serial and self.serial.is_open:
            try:
                response = self.serial.readline().decode('ascii').strip()
                if response:
                    return response
            except Exception as e:
                self.log_message(f"Read error: {str(e)}")
        return None

    def get_button_value(self, button_num):
        """Calculate decimal value for specific direction from its checkboxes"""
        value = 0
        for i, checkbox in enumerate(self.button_groups[button_num]['checkboxes']):
            if checkbox.isChecked():
                value += 1 << (7 - i)  # Bit position (MSB first)
        return value

    def update_button_value(self, button_num):
        """Update log when checkboxes change"""
        value = self.get_button_value(button_num)
        direction = self.button_groups[button_num]['direction']
        self.log_message(f"{direction} value updated to: {value} (0b{bin(value)[2:].zfill(8)})")

    def read_settings(self):
        if self.send_command("G"):
            response = self.read_response()
            if response and response.startswith("Values:"):
                values = response.split()[1:] 
                if len(values) == 8:
                    try:
                        for btn_num in range(8):
                            decimal_value = int(values[btn_num])
                            for bit in range(8):
                                bit_value = (decimal_value >> (7 - bit)) & 1
                                self.button_groups[btn_num]['checkboxes'][bit].setChecked(bit_value)
                        self.log_message("All directions loaded successfully")
                    except ValueError:
                        self.log_message("Invalid response format")
                else:
                    self.log_message("Invalid number of values received")
            else:
                self.log_message("Failed to read settings")

    def write_settings(self):
        values = [str(self.get_button_value(btn_num)) for btn_num in range(8)]
        command = "F " + " ".join(values)
        if self.send_command(command):
            response = self.read_response()
            if response and "OK" in response:
                self.log_message("All directions saved successfully")
            else:
                self.log_message("Failed to save settings")

    def closeEvent(self, event):
        if self.serial and self.serial.is_open:
            self.serial.close()
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MicrocontrollerGUI()
    window.show()
    sys.exit(app.exec_())