from ultralytics import YOLO
import cv2

model = YOLO('../Yolo-Weights/yolov8l.pt')
results = model('Images/basketball_in_hoop.png', show=True)
cv2.waitKey(0)

'''
First PSDR: Display data on LCD via SPI, also include that the other end is the microcontroller
Another PSDR: The ability to interface between the microcontroller and a PC using UART

USB can give 5V and 500 mA. Might as well do this from laptop since we are tethering to the PC anyway.
Consider using linear regulator to go from 5V to 3.3V. Or use a level shifter. Ultrasonic communication
can be one-way.

Three Hardware PSDRs:
MCU to computer
MCU to IMU
MCU to Ultrasonic
'''