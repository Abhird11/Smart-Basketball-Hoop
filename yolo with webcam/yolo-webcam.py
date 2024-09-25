from ultralytics import YOLO
import cv2
import cvzone
import math
import numpy as np

cap = cv2.VideoCapture(0)
cap.set(3, 1920)
cap.set(4, 1080)
FRAME_HEIGHT = cap.get(4)

G = 32.174
BALL_W = 0.4167

model = YOLO("../Yolo-Weights/basketball.pt")

classNames = ['ball', 'hoop', 'player']

class Obj:
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
        self.conf = 0
        self.maxconf = 0

    def set(self, n_x1, n_y1, n_x2, n_y2, n_conf):
        self.x1 = n_x1
        self.y1 = n_y1
        self.x2 = n_x2
        self.y2 = n_y2
        self.w = n_x2 - n_x1
        self.h = n_y2 - n_y1
        self.centerx = int(self.x1 + self.w / 2)
        self.centery = int(self.y1 + self.h / 2)
        self.conf = n_conf

ball = Obj("ball")
hoop = Obj("hoop")
player = Obj("player")
arc_points = []
ideal_arc_points = []
loading_arc_point = (0, 0)
in_play = 0
shot_direction = 0
loaded = 1
phase = "Setup"
prev_phase = "Setup"


def calc_y(x, x0, y0, x2, y2, theta, width):
    g = G * width/BALL_W
    y0 = FRAME_HEIGHT - y0
    y2 = FRAME_HEIGHT - y2
    v0 = np.sqrt((g*((x2-x0)**2)) / ((x2-x0)*np.sin(2*theta) - 2*(y2-y0)*(np.cos(theta)**2)))
    y = FRAME_HEIGHT - (y0 + np.tan(theta)*(x-x0) - (g*((x-x0)**2))/(2*(v0**2)*(np.cos(theta)**2)))
    return int(FRAME_HEIGHT) if np.isnan(y) else int(y)

while True:
    success, img = cap.read()
    results = model(img, stream=True)
    for r in results:
        boxes = r.boxes
        max_player_conf = 0
        max_ball_conf = 0
        max_hoop_conf = 0

        for box in boxes:
            x1, y1, x2, y2 = box.xyxy[0]
            x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)

            w, h = x2 - x1, y2 - y1

            conf = math.ceil(box.conf[0] * 100) / 100
            cls = int(box.cls[0])

            if classNames[cls] == 'ball' and conf > max_ball_conf:
                ball.set(x1, y1, x2, y2, conf)
                max_ball_conf = conf
            elif classNames[cls] == 'hoop' and conf > max_hoop_conf:
                hoop.set(x1, y1, x2, y2, conf)
                max_hoop_conf = conf
            elif classNames[cls] == 'player' and conf > max_player_conf:
                player.set(x1, y1, x2, y2, conf)
                max_player_conf = conf

        if ball.conf > 0:
            cvzone.cornerRect(img, (ball.x1, ball.y1, ball.w, ball.h))
            cvzone.putTextRect(img, f'{ball.className} {ball.conf}', (max(0, ball.x1), max(40, ball.y1 - 20)))
            if phase == "Shooting":
                arc_points.append((ball.centerx, ball.centery))
        if hoop.conf > 0:
            cvzone.cornerRect(img, (hoop.x1, hoop.y1, hoop.w, hoop.h))
            cvzone.putTextRect(img, f'{hoop.className} {hoop.conf}', (max(0, hoop.x1), max(40, hoop.y1 - 20)))
        if player.conf > 0:
            cvzone.cornerRect(img, (player.x1, player.y1, player.w, player.h))
            cvzone.putTextRect(img, f'{player.className} {player.conf}', (max(0, player.x1), max(40, player.y1 - 20)))

        if player.conf > 0 and hoop.conf > 0 and ball.conf > 0:
            in_play = 1
            if player.centerx < hoop.centerx:
                shot_direction = 1
            else:
                shot_direction = -1

        if in_play == 1:
            if shot_direction == 1:

                if ball.x1 <= player.x2:
                    ideal_arc_points = []
                    for i in range(ball.centerx, hoop.centerx, 20):
                        ideal_arc_points.append((i, calc_y(i, ball.centerx, ball.centery, hoop.centerx, hoop.y1, 45, int((ball.w+ball.h)/2))))
                    phase = "Loading"
                    loaded = 1
                    loading_arc_point = (ball.centerx, ball.centery)
                elif ball.x2 >= hoop.x1 - ball.w:
                    phase = "Landing"
                    loaded = 0
                else:
                    if loaded == 1:
                        if phase == "Loading":
                            arc_points = [loading_arc_point, (ball.centerx, ball.centery)]
                        phase = "Shooting"
                    else:
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

            for i in range(len(arc_points) - 1):
                cv2.line(img, arc_points[i], arc_points[i+1], (255,0,0), 10)
            for i in range(len(ideal_arc_points) - 1):
                cv2.line(img, ideal_arc_points[i], ideal_arc_points[i+1], (0,0,0), 10)
            cvzone.putTextRect(img, f'{phase}', (500, 500))


    cv2.imshow("Image", img)
    cv2.waitKey(1)
