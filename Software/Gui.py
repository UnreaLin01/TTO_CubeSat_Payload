import sys
import serial
import serial.tools.list_ports
import struct
import time
import csv
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QLabel, QWidget,
    QPushButton, QComboBox, QMessageBox, QFileDialog,
    QGridLayout, QHBoxLayout, QSizePolicy
)
from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtGui import QFont
from threading import Thread
import pyqtgraph as pg
import pyqtgraph.opengl as gl  # Import OpenGL module
import numpy as np
import os
from datetime import datetime

EXPECTED_LENGTH = 1 + 12 * 4  # Start byte + 12 floats
FIFO_SIZE = 2048  # Increased buffer size for safety


class CircularFIFO:
    def __init__(self, size):
        self.buffer = [0] * size
        self.size = size
        self.head = 0
        self.tail = 0
        self.full = False

    def append(self, byte):
        self.buffer[self.head] = byte
        self.head = (self.head + 1) % self.size
        if self.head == self.tail:  # Overwrite old data when buffer is full
            self.full = True
            self.tail = (self.tail + 1) % self.size

    def get_data(self):
        if self.full or self.head != self.tail:
            if self.head > self.tail:
                return self.buffer[self.tail:self.head]
            else:
                return self.buffer[self.tail:] + self.buffer[:self.head]
        return []

    def clear(self):
        self.head = 0
        self.tail = 0
        self.full = False


class SensorApp(QMainWindow):
    data_received = pyqtSignal(list)

    def __init__(self):
        super().__init__()
        self.setWindowTitle("TTO-CSAT GUI")
        self.labels = []
        self.update_rate = 0.25  # Default update rate is 0.25 seconds
        self.data_duration = 5 * 60  # Store data for 5 minutes
        self.display_duration = 1 * 60  # Display last 1 minute of data when connected
        self.max_data_points = int(self.data_duration / self.update_rate)
        self.display_data_points = int(self.display_duration / self.update_rate)
        self.init_ui()
        self.fifo = CircularFIFO(FIFO_SIZE)
        self.serial_port = None
        self.serial_thread = None
        self.running = False
        self.connected = False  # Track connection status
        self.logging = False  # Track logging status
        self.log_directory = None  # Directory for log files
        self.log_file = None  # File object for the log file
        self.log_writer = None  # CSV writer object

        # Initialize variables for user stamps
        self.user_stamp_count = 0  # Counter for user stamps
        self.user_stamp_pending = False  # Flag to indicate stamp pending

        # Initialize data for plots
        self.bme_temp_data = []
        self.diode_temp_data = []
        self.bme_humidity_data = []
        self.bme_pressure_data = []
        self.time_data = []
        self.initial_time = None  # Will store the initial time when data collection starts

        # Connect the data_received signal to the update_labels slot
        self.data_received.connect(self.update_labels)

    def init_ui(self):
        # Main window layout
        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        # Use a grid layout to organize widgets
        self.grid_layout = QGridLayout(central_widget)
        central_widget.setLayout(self.grid_layout)

        # Control area
        self.port_combobox = QComboBox()
        self.port_combobox.setFixedSize(100, 30)
        self.port_combobox.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)
        self.refresh_ports()

        self.update_rate_combobox = QComboBox()
        self.update_rate_combobox.setFixedSize(100, 30)
        self.update_rate_combobox.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)
        self.update_rate_combobox.addItems(["0.25s", "1s", "2.5s", "5s"])
        self.update_rate_combobox.setCurrentIndex(0)
        self.update_rate_combobox.currentTextChanged.connect(self.set_update_rate)

        self.connect_button = QPushButton("Connect")
        self.connect_button.setFixedSize(100, 30)
        self.connect_button.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)
        self.connect_button.clicked.connect(self.connect_serial)

        self.disconnect_button = QPushButton("Disconnect")
        self.disconnect_button.setFixedSize(100, 30)
        self.disconnect_button.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)
        self.disconnect_button.setEnabled(False)
        self.disconnect_button.clicked.connect(self.disconnect_serial)

        self.select_log_button = QPushButton("Save Location")
        self.select_log_button.setFixedSize(120, 30)
        self.select_log_button.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)
        self.select_log_button.clicked.connect(self.select_log_directory)

        self.start_log_button = QPushButton("Start Log")
        self.start_log_button.setFixedSize(100, 30)
        self.start_log_button.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)
        self.start_log_button.setEnabled(False)
        self.start_log_button.clicked.connect(self.start_logging)

        self.stop_log_button = QPushButton("Stop Log")
        self.stop_log_button.setFixedSize(100, 30)
        self.stop_log_button.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)
        self.stop_log_button.setEnabled(False)
        self.stop_log_button.clicked.connect(self.stop_logging)

        # Create the "User Stamp" button
        self.user_stamp_button = QPushButton("User Stamp")
        self.user_stamp_button.setFixedSize(100, 30)
        self.user_stamp_button.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)
        self.user_stamp_button.setEnabled(False)
        self.user_stamp_button.clicked.connect(self.add_user_stamp)

        # Create a horizontal layout for control widgets
        control_layout = QHBoxLayout()
        control_layout.setSpacing(10)  # Adjust spacing between widgets
        control_layout.addWidget(self.port_combobox)
        control_layout.addWidget(self.update_rate_combobox)
        control_layout.addWidget(self.connect_button)
        control_layout.addWidget(self.disconnect_button)
        control_layout.addWidget(self.select_log_button)
        control_layout.addWidget(self.start_log_button)
        control_layout.addWidget(self.stop_log_button)
        # Add the "User Stamp" button to the control layout
        control_layout.addWidget(self.user_stamp_button)

        # Create a QWidget to hold the layout
        control_widget = QWidget()
        control_widget.setLayout(control_layout)
        control_widget.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)

        # Add the control_widget to the grid layout at a fixed position
        self.grid_layout.addWidget(control_widget, 0, 0, 1, 4, alignment=Qt.AlignLeft)

        # Set stretch factors to fix control widget position
        self.grid_layout.setRowStretch(0, 0)  # Control widgets
        self.grid_layout.setRowStretch(1, 1)  # Main content
        self.grid_layout.setColumnStretch(0, 0)  # Labels
        self.grid_layout.setColumnStretch(1, 1)  # Plots
        self.grid_layout.setColumnStretch(2, 1)  # Plots
        self.grid_layout.setColumnStretch(2, 1)  # 3D plot

        # Labels area
        labels_widget = QWidget()
        labels_layout = QGridLayout(labels_widget)
        labels_widget.setLayout(labels_layout)

        labels_text = [
            "BME Temperature", "BME Pressure", "BME Altitude", "BME Humidity",
            "MPU Gyro X", "MPU Gyro Y", "MPU Gyro Z",
            "MPU Acc X", "MPU Acc Y", "MPU Acc Z",
            "Diode Temp", "Voltage"
        ]

        for i, text in enumerate(labels_text):
            label = QLabel(f"{text}: 0.00")
            label.setFixedSize(200, 20)
            labels_layout.addWidget(label, i, 0)
            self.labels.append(label)

        self.grid_layout.addWidget(labels_widget, 1, 0)

        # First row of plots
        self.bme_temp_plot = pg.PlotWidget(title="BME Temperature")
        self.bme_temp_curve = self.bme_temp_plot.plot(pen='r')

        self.bme_pressure_plot = pg.PlotWidget(title="BME Pressure")
        self.bme_pressure_curve = self.bme_pressure_plot.plot(pen='m')

        # Second row of plots
        self.diode_temp_plot = pg.PlotWidget(title="Diode Temperature")
        self.diode_temp_curve = self.diode_temp_plot.plot(pen='b')

        self.bme_humidity_plot = pg.PlotWidget(title="BME Humidity")
        self.bme_humidity_curve = self.bme_humidity_plot.plot(pen='g')

        # Add plots to layout
        self.grid_layout.addWidget(self.bme_temp_plot, 1, 1)
        self.grid_layout.addWidget(self.bme_pressure_plot, 1, 2)
        self.grid_layout.addWidget(self.diode_temp_plot, 2, 1)
        self.grid_layout.addWidget(self.bme_humidity_plot, 2, 2)

        try:
            # 3D Plot for acceleration
            self.acc_3d_plot = gl.GLViewWidget()
            self.acc_3d_plot.setCameraPosition(distance=5)

            # Add a unit sphere
            md = gl.MeshData.sphere(rows=10, cols=20, radius=1)
            self.unit_sphere = gl.GLMeshItem(
                meshdata=md, smooth=True, color=(0.5, 0.5, 0.5, 0.3),
                shader='shaded', drawFaces=True
            )
            self.acc_3d_plot.addItem(self.unit_sphere)

            # Add XYZ axes
            axis_length = 1.5
            x_axis = gl.GLLinePlotItem(pos=np.array([[0, 0, 0], [-axis_length, 0, 0]]), color=(1, 0, 0, 1), width=2)
            y_axis = gl.GLLinePlotItem(pos=np.array([[0, 0, 0], [0, -axis_length, 0]]), color=(0, 1, 0, 1), width=2)
            z_axis = gl.GLLinePlotItem(pos=np.array([[0, 0, 0], [0, 0, axis_length]]), color=(0, 0, 1, 1), width=2)
            self.acc_3d_plot.addItem(x_axis)
            self.acc_3d_plot.addItem(y_axis)
            self.acc_3d_plot.addItem(z_axis)

            # Add acceleration vector
            self.acc_vector = gl.GLLinePlotItem()
            self.acc_3d_plot.addItem(self.acc_vector)

            # Add the 3D plot to the layout next to the plots
            self.grid_layout.addWidget(self.acc_3d_plot, 1, 3, 2, 50)

            # Set fixed window size
            self.setFixedSize(1600, 800)
        except Exception as e:
            print(f"Error setting up 3D plot: {e}")

    def add_user_stamp(self):
        if self.logging:
            self.user_stamp_count += 1
            self.user_stamp_pending = True
            print(f"User Stamp {self.user_stamp_count} added.")
        else:
            QMessageBox.warning(self, "Warning", "Logging is not active.")

    def refresh_ports(self):
        # Refresh available COM Ports
        self.port_combobox.clear()
        ports = serial.tools.list_ports.comports()
        for port in ports:
            self.port_combobox.addItem(port.device)

    def set_update_rate(self, value):
        # Update the update rate
        rate_mapping = {"0.25s": 0.25, "1s": 1.0, "2.5s": 2.5, "5s": 5.0}
        self.update_rate = rate_mapping.get(value, 0.25)
        self.max_data_points = int(self.data_duration / self.update_rate)
        self.display_data_points = int(self.display_duration / self.update_rate)
        print(f"Update rate set to {self.update_rate} seconds")

    def set_plot_interaction(self, enabled):
        # Enable or disable plot interaction
        self.bme_temp_plot.setMouseEnabled(x=enabled, y=enabled)
        self.diode_temp_plot.setMouseEnabled(x=enabled, y=enabled)
        self.bme_humidity_plot.setMouseEnabled(x=enabled, y=enabled)
        self.bme_pressure_plot.setMouseEnabled(x=enabled, y=enabled)

    def update_plots(self):
        # Decide what data to display
        if self.connected:
            # Display last 1 minute of data
            if len(self.time_data) < self.display_data_points:
                # Not enough data for scrolling; display all available data
                display_time_data = self.time_data
                display_bme_temp_data = self.bme_temp_data
                display_diode_temp_data = self.diode_temp_data
                display_bme_humidity_data = self.bme_humidity_data
                display_bme_pressure_data = self.bme_pressure_data

                x_min = 0
                x_max = self.display_duration  # Fixed range
            else:
                # Display the last 1 minute of data
                display_time_data = self.time_data[-self.display_data_points:]
                display_bme_temp_data = self.bme_temp_data[-self.display_data_points:]
                display_diode_temp_data = self.diode_temp_data[-self.display_data_points:]
                display_bme_humidity_data = self.bme_humidity_data[-self.display_data_points:]
                display_bme_pressure_data = self.bme_pressure_data[-self.display_data_points:]

                x_min = display_time_data[0]
                x_max = display_time_data[-1]

            # Update plot curves
            self.bme_temp_curve.setData(display_time_data, display_bme_temp_data)
            self.diode_temp_curve.setData(display_time_data, display_diode_temp_data)
            self.bme_humidity_curve.setData(display_time_data, display_bme_humidity_data)
            self.bme_pressure_curve.setData(display_time_data, display_bme_pressure_data)

            # Adjust x-range
            self.bme_temp_plot.setXRange(x_min, x_max, padding=0)
            self.diode_temp_plot.setXRange(x_min, x_max, padding=0)
            self.bme_humidity_plot.setXRange(x_min, x_max, padding=0)
            self.bme_pressure_plot.setXRange(x_min, x_max, padding=0)
        else:
            # When disconnected, display all stored data
            display_time_data = self.time_data
            display_bme_temp_data = self.bme_temp_data
            display_diode_temp_data = self.diode_temp_data
            display_bme_humidity_data = self.bme_humidity_data
            display_bme_pressure_data = self.bme_pressure_data

            # Update plot curves
            self.bme_temp_curve.setData(display_time_data, display_bme_temp_data)
            self.diode_temp_curve.setData(display_time_data, display_diode_temp_data)
            self.bme_humidity_curve.setData(display_time_data, display_bme_humidity_data)
            self.bme_pressure_curve.setData(display_time_data, display_bme_pressure_data)

            # Do not adjust x-range; allow user to pan and zoom

    def update_labels(self, values):
        # Update data on the interface
        if values:
            for label, value in zip(self.labels, values):
                label.setText(f"{label.text().split(':')[0]}: {value:.2f}")

            # Get current time
            current_time = time.time()

            # Initialize initial_time if it's None
            if self.initial_time is None:
                self.initial_time = current_time

            # Calculate elapsed time
            elapsed_time = current_time - self.initial_time

            # Add data
            self.time_data.append(elapsed_time)
            self.bme_temp_data.append(values[0])
            self.diode_temp_data.append(values[10])
            self.bme_humidity_data.append(values[3])
            self.bme_pressure_data.append(values[1])

            # Keep only the most recent data up to max_data_points
            if len(self.time_data) > self.max_data_points:
                self.time_data = self.time_data[-self.max_data_points:]
                self.bme_temp_data = self.bme_temp_data[-self.max_data_points:]
                self.diode_temp_data = self.diode_temp_data[-self.max_data_points:]
                self.bme_humidity_data = self.bme_humidity_data[-self.max_data_points:]
                self.bme_pressure_data = self.bme_pressure_data[-self.max_data_points:]

            # Update acceleration vector
            acc_x = values[7]
            acc_y = values[8]
            acc_z = values[9]
            acc_vector = np.array([acc_x, acc_y, acc_z])

            # Normalize the vector
            norm = np.linalg.norm(acc_vector)
            if norm != 0:
                acc_unit_vector = acc_vector / norm
            else:
                acc_unit_vector = acc_vector

            # Update the 3D plot
            line_data = np.array([[0, 0, 0], acc_unit_vector])
            self.acc_vector.setData(
                pos=line_data, color=(1, 1, 0, 1), width=3, antialias=True
            )

            # Log data if logging is enabled
            if self.logging and self.log_writer:
                # Prepare the data row
                timestamp = datetime.now().strftime('%H:%M:%S.%f')[:-3]  # Time with milliseconds
                data_row = [timestamp] + values

                # Include the "User Stamp" if pending
                if self.user_stamp_pending:
                    data_row.append(self.user_stamp_count)
                    self.user_stamp_pending = False
                else:
                    data_row.append('')  # Empty value for "User Stamp"

                try:
                    self.log_writer.writerow(data_row)
                    self.log_file.flush()
                except Exception as e:
                    print(f"Error writing to log file: {e}")
                    self.stop_logging()

        # Update plots
        self.update_plots()

    def process_fifo(self):
        # Process data in the FIFO
        data = self.fifo.get_data()
        buffer_len = len(data)

        i = 0
        while i < buffer_len:
            if data[i] == 0xFF:
                if i + EXPECTED_LENGTH < buffer_len and data[i + EXPECTED_LENGTH] == ord('\n'):
                    packet = data[i + 1:i + EXPECTED_LENGTH]  # Remove start byte
                    try:
                        values = list(struct.unpack('<12f', bytes(packet)))
                        self.data_received.emit(values)  # Emit signal to update UI
                        self.fifo.clear()
                        return
                    except struct.error as e:
                        print(f"Error decoding data: {e}. Skipping.")
                    i += EXPECTED_LENGTH + 1  # Move past the packet
                else:
                    break  # Wait for more data
            else:
                i += 1

    def serial_worker(self, port):
        try:
            self.serial_port = serial.Serial(port, baudrate=115200, timeout=1)
            print(f"Connected to {port}")
        except serial.SerialException as e:
            QMessageBox.critical(self, "Error", f"Failed to connect to {port}: {e}")
            self.running = False
            return

        next_call = time.time()
        while self.running:
            try:
                self.serial_port.write(b'?')  # Send data request command
                time.sleep(0.05)  # Give the device some time to respond
                incoming_data = self.serial_port.read(self.serial_port.in_waiting or 1)
                for byte in incoming_data:
                    self.fifo.append(byte)

                self.process_fifo()

                # Sleep until the next update time
                next_call += self.update_rate
                sleep_time = max(0, next_call - time.time())
                time.sleep(sleep_time)
            except Exception as e:
                print(f"Error during serial read: {e}")
                break

        if self.serial_port:
            self.serial_port.close()
            print(f"Disconnected from {port}")

    def connect_serial(self):
        # Connect to serial port
        if self.running:
            return

        port = self.port_combobox.currentText()
        if not port:
            QMessageBox.warning(self, "Warning", "No serial port selected.")
            return

        # Reset data lists and initial time
        self.time_data.clear()
        self.bme_temp_data.clear()
        self.diode_temp_data.clear()
        self.bme_humidity_data.clear()
        self.bme_pressure_data.clear()
        self.initial_time = None

        self.running = True
        self.connected = True  # Set connected status here
        self.set_plot_interaction(False)  # Disable plot interaction

        self.serial_thread = Thread(target=self.serial_worker, args=(port,))
        self.serial_thread.daemon = True
        self.serial_thread.start()

        self.connect_button.setEnabled(False)
        self.disconnect_button.setEnabled(True)

        # Disable update rate combobox and port combobox after connecting
        self.update_rate_combobox.setEnabled(False)
        self.port_combobox.setEnabled(False)

        # Enable logging buttons
        if self.log_directory:
            self.start_log_button.setEnabled(True)
        else:
            self.start_log_button.setEnabled(False)
        self.stop_log_button.setEnabled(False)

    def disconnect_serial(self):
        # Disconnect from serial port
        self.running = False
        if self.serial_thread and self.serial_thread.is_alive():
            self.serial_thread.join()

        self.connected = False  # Update connection status
        self.set_plot_interaction(True)  # Enable plot interaction

        self.connect_button.setEnabled(True)
        self.disconnect_button.setEnabled(False)

        # Enable update rate combobox and port combobox after disconnecting
        self.update_rate_combobox.setEnabled(True)
        self.port_combobox.setEnabled(True)

        # Disable logging buttons
        self.start_log_button.setEnabled(False)
        self.stop_log_button.setEnabled(False)

        # Reset user stamp variables
        self.user_stamp_count = 0
        self.user_stamp_pending = False

        # Stop logging if it's active
        if self.logging:
            self.stop_logging()

        # Update plots to show all data
        self.update_plots()

    def select_log_directory(self):
        # Open a directory selection dialog
        options = QFileDialog.Options()
        directory = QFileDialog.getExistingDirectory(
            self,
            "Select Save Location",
            options=options
        )
        if directory:
            self.log_directory = directory
            print(f"Log directory selected: {self.log_directory}")
            # Enable Start Log button if connected
            if self.connected:
                self.start_log_button.setEnabled(True)
            else:
                self.start_log_button.setEnabled(False)
        else:
            self.log_directory = None
            self.start_log_button.setEnabled(False)

    def start_logging(self):
        if not self.log_directory:
            QMessageBox.warning(self, "Warning", "Please select a save location first.")
            return
        try:
            # Generate a filename with the current date and time
            filename = datetime.now().strftime('%Y-%m-%d_%H-%M-%S') + '.csv'
            log_file_path = os.path.join(self.log_directory, filename)
            # Open the log file in write mode
            self.log_file = open(log_file_path, 'w', newline='')
            self.log_writer = csv.writer(self.log_file)
            # Write header
            header = ['Time'] + [label.text().split(':')[0] for label in self.labels] + ['User Stamp']
            self.log_writer.writerow(header)
            self.logging = True
            self.start_log_button.setEnabled(False)
            self.stop_log_button.setEnabled(True)
            self.user_stamp_button.setEnabled(True)
            print(f"Logging started. Saving to {log_file_path}")
            
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to start logging: {e}")

    def stop_logging(self):
        if self.logging:
            try:
                self.log_file.close()
                print("Logging stopped.")
            except Exception as e:
                print(f"Error closing log file: {e}")
            finally:

                self.logging = False
                self.user_stamp_button.setEnabled(False)
                self.start_log_button.setEnabled(True)
                self.stop_log_button.setEnabled(False)
                self.log_file = None
                self.log_writer = None

                # Reset user stamp variables
                self.user_stamp_count = 0
                self.user_stamp_pending = False

    def closeEvent(self, event):
        # Safe exit handling
        self.running = False
        if self.serial_thread and self.serial_thread.is_alive():
            self.serial_thread.join()
        if self.logging:
            self.stop_logging()
        event.accept()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    main_window = SensorApp()
    main_window.show()
    sys.exit(app.exec_())
