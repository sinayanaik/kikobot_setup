from st3215 import ST3215
import time
import serial.tools.list_ports

# Function to autodetect the first USB serial port
def autodetect_usb_port():
    ports = list(serial.tools.list_ports.comports())
    for port in ports:
        if 'USB' in port.description:
            return port.device
    return None

# Autodetect the USB port
usb_port = autodetect_usb_port()
if usb_port:
    print(f"Detected USB port: {usb_port}")
    servo = ST3215(usb_port)
else:
    print("No USB port detected. Falling back to /dev/serial0")
    servo = ST3215('/dev/serial0')

ids = servo.ListServos()
print("Hello")
print(ids)
servo.MoveTo(1, 1000)
time.sleep(2)
if ids:
    servo.MoveTo(ids[0], 2048)
print(servo.PingServo(1))