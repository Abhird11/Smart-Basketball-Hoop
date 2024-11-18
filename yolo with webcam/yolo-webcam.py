from ultralytics import YOLO
import cv2
import cvzone
import math
import numpy as np
import time
import threading
import math
import serial

# Set constants for camera
cap = cv2.VideoCapture(0)
cap.set(3, 1920)
cap.set(4, 1080)
FRAME_WIDTH = int(cap.get(3))
FRAME_HEIGHT = int(cap.get(4))

# Set thresholds for sensors
ULTRASONIC_THRESH = 5
ACCEL_RIM_THRESH = 100
ACCEL_BACK_THRESH = 100

# Constants
G = 32.174
BALL_W = 0.5

# Get information about model
model = YOLO("../Yolo-Weights/basketball_30_epochs.pt")
classNames = ['ball', 'hoop', 'player']

# Object class for ball, player, and hoop locations
class Obj:
    # Set all variables.
    # Positions start at -1, and widths and heights start at 0
    def __init__(self, className):
        self.className = className
        self.x1 = -1
        self.y1 = -1
        self.x2 = -1
        self.y2 = -1
        self.centerx = -1
        self.centery = -1
        self.w = 0
        self.h = 0
        self.x1_pred = -1
        self.y1_pred = -1
        self.x2_pred = -1
        self.y2_pred = -1
        self.w_pred = 0
        self.h_pred = 0
        self.x1_curr = -1
        self.y1_curr = -1
        self.x2_curr = -1
        self.y2_curr = -1
        self.conf_curr = 0
        self.conf = 0
        self.maxconf = 0

    # Set positions
    def set(self, n_x1, n_y1, n_x2, n_y2, n_conf):
        self.x1 = n_x1
        self.y1 = n_y1
        self.x2 = n_x2
        self.y2 = n_y2
        self.w = n_x2 - n_x1
        self.h = n_y2 - n_y1
        self.x1_pred = self.x1
        self.y1_pred = self.y1
        self.x2_pred = self.x2
        self.y2_pred = self.y2
        self.w_pred = self.w
        self.h_pred = self.h
        self.centerx = int(self.x1 + self.w / 2)
        self.centery = int(self.y1 + self.h / 2)
        self.conf = n_conf

    # Set predicted position
    def set_pred(self, n_x1, n_y1, n_x2, n_y2):
        self.x1_pred = n_x1
        self.y1_pred = n_y1
        self.x2_pred = n_x2
        self.y2_pred = n_y2
        self.w_pred = self.x2_pred - self.x1_pred
        self.h_pred = self.y2_pred - self.y1_pred

    # Set current position
    def set_curr(self, n_x1, n_y1, n_x2, n_y2, n_conf):
        self.x1_curr = n_x1
        self.y1_curr = n_y1
        self.x2_curr = n_x2
        self.y2_curr = n_y2
        self.conf_curr = n_conf


# Create objects
ball = Obj("ball")
hoop = Obj("hoop")
player = Obj("player")
ball_timer = 0
player_timer = 0

# Initialize arc points arrays
arc_points = []
ideal_arc_points = []
loading_arc_point = (0, 0)

# Phase variables
in_play = 0
shot_direction = 0
loaded = 1
phase = "Setup"
prev_phase = "Setup"

# Statistics
shots_taken = 0
shots_scored = 0
shots_missed = 0
shot_percentage = 0
rim_shots = 0
back_shots = 0
airballs = 0
swishes = 0



def updateStatistics():
    global shots_taken, shots_scored, shots_missed, shot_percentage, rim_shots, back_shots, airballs, swishes
    global phase, ball, player, hoop
    [shot_flag, rim_flag, back_flag] = [0, 0, 0]
    shot_time = 0
    #ser = serial.Serial(port="COM4", baudrate=115200)
    phase_transitioned = 0
    while True:
        while phase == "Loading" or phase == "Setup":
            if (phase_transitioned == 1 and time.time() - shot_time > 0.5):
                shots_taken += 1
                shots_scored += shot_flag
                if (shot_flag == 0):
                    shots_missed += 1
                shot_percentage = math.floor(10000 * shots_scored / shots_taken) / 100
                rim_shots += rim_flag
                back_shots = time.time() - shot_time
                if (rim_flag == 0 and back_flag == 0):
                    if (shot_flag == 1):
                        swishes += 1
                    else:
                        airballs += 1
            phase_transitioned = 0
            shot_flag = 0
            rim_flag = 0
            back_flag = 0
            time.sleep(0.1)
        while phase == "Shooting" or phase == "Landing" or phase == "Returning":
            if (phase_transitioned == 0):
                phase_transitioned = 1
                shot_time = time.time()
            '''
            if (not shot_flag and ser.read(1) == 1):
                shot_flag = 1
            if (not rim_flag and ser.read(1) == 1):
                rim_flag = 1
            if (not back_flag and ser.read(1) == 1):
                back_flag = 1
            '''
            time.sleep(0.2)

threading.Thread(target=updateStatistics).start()

# Calculate the ideal arc
# x - current x position
# (x0, y0) - coordinates of starting position
# (x2, y2) - coordinates of ending position (hoop)
# theta - angle of shot (set to 45)
# width - ball width in ft
def calc_y(x, x0, y0, x2, y2, theta, width):
    g = G * width/BALL_W
    y0 = FRAME_HEIGHT - y0
    y2 = FRAME_HEIGHT - y2
    v0 = np.sqrt((g*((x2-x0)**2)) / ((x2-x0)*np.sin(2*theta) - 2*(y2-y0)*(np.cos(theta)**2)))
    y = FRAME_HEIGHT - (y0 + np.tan(theta)*(x-x0) - (g*((x-x0)**2))/(2*(v0**2)*(np.cos(theta)**2)))
    return int(FRAME_HEIGHT) if np.isnan(y) else int(y)

# Keep reading images
while True:
    # Read image
    success, img = cap.read()

    # Get all objects in frame
    results = model(img, stream=True)

    # Decrease confidence as frames go forward for ball and player,
    # since those locations are likely to be changes
    if not phase == "Loading" or phase == "Setup":
        if ball.conf > 0.3:
            ball.conf /= 2
        if player.conf > 0.3:
            player.conf /= 2

    for r in results:

        # Get all bounding boxes
        boxes = r.boxes

        # Max confidences found out of current detected objects
        max_player_conf = 0
        max_ball_conf = 0
        max_hoop_conf = 0
        ball.conf_curr = 0
        hoop.conf_curr = 0
        player.conf_curr = 0

        # Loop through each bounding box in frame
        for box in boxes:

            # Get coordinates of box
            x1, y1, x2, y2 = box.xyxy[0]
            x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
            w, h = x2 - x1, y2 - y1
            conf = math.ceil(box.conf[0] * 100) / 100
            cls = int(box.cls[0])

            # Display box
            cvzone.cornerRect(img, (x1, y1, w, h))
            cvzone.putTextRect(img, f'{classNames[cls]} {conf}', (max(0, x1), max(40, min(FRAME_HEIGHT - 120, y1 - 20))))

            # ADJUST OBJECT CONFIDENCE VALUES
            # Ball confidence adjustment
            if classNames[cls] == 'ball':
                if phase == "Shooting":
                    if x1 - ball.x1 < ball.w * 2 and x1 > ball.x1:
                        conf *= 50
                    elif x1 - ball.x1 < ball.w * 6 and x1 > ball.x1:
                        conf *= 3
                    elif x1 > player.x2:
                        conf = 0
                elif phase == "Loading" or phase == "Setup":
                    if math.sqrt((x1 - ball.x1) ** 2 + (y1 - ball.y1) ** 2) < ball.w * 2:
                        conf *= 50
                    elif math.sqrt((x1 - ball.x1) ** 2 + (y1 - ball.y1) ** 2) < ball.w * 5:
                        conf *= 3
                    else:
                        if phase == "Loading" and (x1 > player.x2 or x2 < player.x1):
                            conf = 0
                elif phase == "Landing" or phase == "Returning":
                    if ball.x1 - x1 < ball.w * 2 and x1 < ball.x1:
                        conf *= 50
                    elif ball.x1 - x1 < ball.w * 6 and x1 < ball.x1:
                        conf *= 3
                    elif abs(x1 - hoop.x1) > abs(hoop.x1 - player.x1) or abs(x1 - player.x1) > abs(hoop.x1 - player.x1):
                        conf = 0
                if conf > max_ball_conf and conf > 0 and conf > ball.conf:
                    ball.set_curr(x1, y1, x2, y2, ball.conf)
                    max_ball_conf = conf

            # Hoop confidence adjustment
            elif classNames[cls] == 'hoop' and conf > max_hoop_conf:
                hoop.set_curr(x1, y1, x2, y2, conf)
                max_hoop_conf = conf

            # Player confidence adjustment
            elif classNames[cls] == 'player':
                if abs(x1 - player.x1) > player.w:
                    conf /= 2
                if conf > max_player_conf and conf > 0 and conf > player.conf:
                    player.set_curr(x1, y1, x2, y2, player.conf)
                    max_player_conf = conf

        # Done looping through all boxes in frame

        # TRACKING LOGIC - Set actual coordinates of objects based on current position and last position
        # Ball tracking logic
        if phase == "Shooting":
            if abs(ball.x1_curr - player.x1) > abs(ball.x1 - player.x1) or ball.x1_curr <= player.x2:
                ball.set(ball.x1_curr, ball.y1_curr, ball.x2_curr, ball.y2_curr, max_ball_conf)
        elif phase == "Returning" or phase == "Landing":
            if abs(ball.x1_curr - hoop.x1) > abs(ball.x1 - hoop.x1):
                ball.set(ball.x1_curr, ball.y1_curr, ball.x2_curr, ball.y2_curr, max_ball_conf)
        else:
            # Predict using threshold of distance
            if abs(ball.x1_curr - ball.x1) < abs(hoop.x1 - player.x1) / 2 or abs(ball.x1_curr - ball.x1_pred) < abs(hoop.x1 - player.x1) / 2:
                ball.set(ball.x1_curr, ball.y1_curr, ball.x2_curr, ball.y2_curr, max_ball_conf)
            else:
                ball.set_pred(ball.x1_curr, ball.y1_curr, ball.x2_curr, ball.y2_curr)


        if ball_timer == 2:
            phase = "Setup"
        if ball.conf == 0:
            ball_timer += 1

        # Hoop tracking logic
        if max_hoop_conf > 0.4:
            hoop.set(hoop.x1_curr, hoop.y1_curr, hoop.x2_curr, hoop.y2_curr, hoop.conf_curr)
        '''
        if hoop.x1_curr < hoop.x2 or hoop.x2_curr > hoop.x1 or hoop.x1_curr < hoop.x2_pred or hoop.x2_curr > hoop.x1_pred:
            hoop.set(hoop.x1_curr, hoop.y1_curr, hoop.x2_curr, hoop.y2_curr, max_hoop_conf)
        else:
            hoop.set_pred(hoop.x1_curr, hoop.y1_curr, hoop.x2_curr, hoop.y2_curr)
        '''

        # Player tracking logic
        if player.x1_curr < player.x2 or player.x2_curr > player.x1 or player.x1_curr < player.x2_pred or player.x2_curr > player.x1_pred:
            player.set(player.x1_curr, player.y1_curr, player.x2_curr, player.y2_curr, max_player_conf)
        else:
            player.set_pred(player.x1_curr, player.y1_curr, player.x2_curr, player.y2_curr)

        '''
        ball.set(ball.x1_curr, ball.y1_curr, ball.x2_curr, ball.y2_curr, ball.conf_curr)
        hoop.set(hoop.x1_curr, hoop.y1_curr, hoop.x2_curr, hoop.y2_curr, hoop.conf_curr)
        player.set(player.x1_curr, player.y1_curr, player.x2_curr, player.y2_curr, player.conf_curr)
        '''

        # Display final object boxes
        #if ball.conf > 0:
        cvzone.cornerRect(img, (ball.x1, ball.y1, ball.w, ball.h), rt = 10, colorR = (255, 0, 0), colorC = (0, 0, 255))
        cvzone.putTextRect(img, f'{ball.className} {ball.conf}', (max(0, ball.x1), max(40, min(FRAME_HEIGHT - 120, ball.y1 - 20))), colorR=(255,0,0))
        if phase == "Shooting":
            arc_points.append((ball.centerx, ball.centery))
        if hoop.conf > 0:
            cvzone.cornerRect(img, (hoop.x1, hoop.y1, hoop.w, hoop.h), rt = 10, colorR = (255, 0, 0), colorC = (0, 0, 255))
            cvzone.putTextRect(img, f'{hoop.className} {hoop.conf}', (max(0, hoop.x1), max(40, min(FRAME_HEIGHT - 120, hoop.y1 - 20))), colorR=(255,0,0))
        if player.conf > 0:
            cvzone.cornerRect(img, (player.x1, player.y1, player.w, player.h), rt = 10, colorR = (255, 0, 0), colorC = (0, 0, 255))
            cvzone.putTextRect(img, f'{player.className} {player.conf}', (max(0, player.x1), max(40, min(FRAME_HEIGHT - 120, player.y1 - 20))), colorR=(255,0,0))

        # In play begins when all objects have been detected
        if player.conf > 0 and hoop.conf > 0 and ball.conf > 0:
            in_play = 1
            if player.centerx < hoop.centerx:
                shot_direction = 1
            else:
                shot_direction = -1


        if in_play == 1:
            if shot_direction == 1:
                if ball.x1 <= player.x2: # Loading phase
                    ideal_arc_points = []
                    for i in range(ball.centerx, hoop.centerx, 20):
                        ideal_arc_points.append((i, calc_y(i, ball.centerx, ball.centery, hoop.centerx, hoop.y1, 45, int((ball.w+ball.h)/2))))
                    phase = "Loading"

                    loaded = 1
                    loading_arc_point = (ball.centerx, ball.centery)
                elif ball.x2 >= hoop.x1 - ball.w: # Landing Phase
                    if phase == "Shooting":
                        phase = "Landing"
                else: # Ball in between player and hoop
                    if loaded == 1: # Shooting phase
                        if phase == "Loading":
                            phase = "Shooting"
                            arc_points = [loading_arc_point, (ball.centerx, ball.centery)]
                        loaded = 0
                    else:
                        if phase == "Landing":
                            phase = "Returning"
            else:
                if ball.x2 >= player.x1:
                    phase = "Loading"
                    loaded = 1
                    loading_arc_point = (ball.centerx, ball.centery)
                elif ball.x1 >= hoop.x2 + ball.w:
                    phase = "Landing"
                    loaded = 0
                else:
                    if loaded == 1:
                        if phase == "Loading":
                            arc_points = [loading_arc_point, (ball.centerx, ball.centery)]
                        phase = "Shooting"
                    else:
                        phase = "Returning"


            # Draw arc
            for i in range(len(arc_points) - 1):
                cv2.line(img, arc_points[i], arc_points[i+1], (255,0,0), 10)
            for i in range(len(ideal_arc_points) - 1):
                cv2.line(img, ideal_arc_points[i], ideal_arc_points[i+1], (0,0,0), 10)
            cvzone.putTextRect(img, f'{phase}', (500, 500), 5, 5)

        # Display statistics on bottom of frame
        font_size = 50
        font_offset_v = 15
        font_offset_h = 15
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.rectangle(img, (0, FRAME_HEIGHT - 2*font_size), (FRAME_WIDTH, FRAME_HEIGHT), (0, 0, 0), -1)
        cv2.putText(img, text=f'Scored: {shots_scored}', org=(font_offset_h, FRAME_HEIGHT - font_size - font_offset_v),
                    fontFace=font, fontScale=1, color=(255, 255, 255), thickness=2)
        cv2.putText(img, text=f'Missed: {shots_missed}', org=(font_offset_h, FRAME_HEIGHT - font_offset_v),
                    fontFace=font, fontScale=1, color=(255, 255, 255), thickness=2)
        cv2.putText(img, text=f'Total: {shots_taken}', org=(font_offset_h + int(FRAME_WIDTH * 1/4), FRAME_HEIGHT - font_size - font_offset_v),
                    fontFace=font, fontScale=1, color=(255, 255, 255), thickness=2)
        cv2.putText(img, text=f'Percent: {shot_percentage}%', org=(font_offset_h + int(FRAME_WIDTH * 1/4), FRAME_HEIGHT - font_offset_v),
                    fontFace=font, fontScale=1, color=(255, 255, 255), thickness=2)
        cv2.putText(img, text=f'Rim: {rim_shots}', org=(font_offset_h + int(FRAME_WIDTH * 2/4), FRAME_HEIGHT - font_size - font_offset_v),
                    fontFace=font, fontScale=1, color=(255, 255, 255), thickness=2)
        cv2.putText(img, text=f'Backboard: {back_shots}', org=(font_offset_h + int(FRAME_WIDTH * 2/4), FRAME_HEIGHT - font_offset_v),
                    fontFace=font, fontScale=1, color=(255, 255, 255), thickness=2)
        cv2.putText(img, text=f'Airballs: {airballs}', org=(font_offset_h + int(FRAME_WIDTH * 3/4), FRAME_HEIGHT - font_size - font_offset_v),
                    fontFace=font, fontScale=1, color=(255, 255, 255), thickness=2)
        cv2.putText(img, text=f'Swishes: {swishes}', org=(font_offset_h + int(FRAME_WIDTH * 3/4), FRAME_HEIGHT - font_offset_v),
                    fontFace=font, fontScale=1, color=(255, 255, 255), thickness=2)

    cv2.imshow("Image", img)
    cv2.waitKey(1)
