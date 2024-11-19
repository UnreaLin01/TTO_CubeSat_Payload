import serial
import struct
import time

def read_sensor_data(serial_port):
    try:
        # 打開 Serial 連接
        ser = serial.Serial(serial_port, baudrate=115200, timeout=1)
        print(f"Connected to {serial_port}")
    except serial.SerialException as e:
        print(f"Error opening serial port: {e}")
        return

    try:
        while True:
            # 傳送 '?' 字符
            ser.write(b'?')
            
            # 讀取起始標誌
            start_byte = ser.read(1)
            if not start_byte or start_byte[0] != 0xFF:
                print("No start byte received or invalid start byte.")
                continue

            # 讀取 13 個 float 的數據
            data = ser.read(13 * 4)  # 每個 float 4 bytes，共 13 個
            if len(data) == 13 * 4:
                # 解碼數據
                values = struct.unpack('<13f', data)  # '<' 表示小端，'13f' 表示 13 個 float
                print_sensor_data(values)
            else:
                print("Incomplete data received.")

            # 等待 0.5 秒
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        ser.close()

def print_sensor_data(values):
    labels = [
        "BME Temperature", "BME Pressure", "BME Altitude", "BME Humidity",
        "MPU Gyro X", "MPU Gyro Y", "MPU Gyro Z",
        "MPU Acc X", "MPU Acc Y", "MPU Acc Z",
        "Diode Temp", "Voltage"
    ]

    print("\nSensor Data:")
    for label, value in zip(labels, values):
        print(f"{label}: {value:.2f}")

if __name__ == "__main__":
    serial_port = input("Enter the serial port (e.g., COM3 or /dev/ttyUSB0): ")
    read_sensor_data(serial_port)
