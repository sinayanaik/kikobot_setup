from st3215 import ST3215
import time

servo = ST3215('/dev/serial0')
ids = servo.ListServos()
print("Hello")
print(ids)
servo.MoveTo(1,1000)
time.sleep(2)
if ids:
    servo.MoveTo(ids[0], 2048)
print(servo.PingServo(1))