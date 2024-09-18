from ultralytics import YOLO
import cv2
import cvzone
import math

cap = cv2.VideoCapture(0)
cap.set(3, 1920)
cap.set(4, 1080)

model = YOLO("../Yolo-Weights/basketball.pt")

classNames = ['ball', 'hoop', 'player']

ball_center = (-1, -1)
ball_conf = 0
hoop_center = (-1, -1)
hoop_conf = 0
player_center = (-1, -1)
player_conf = 0

while True:
    success, img = cap.read()
    results = model(img, stream=True)
    for r in results:
        boxes = r.boxes
        max_player_conf = 0
        max_ball_conf = 0
        max_hoop_conf = 0

        for box in boxes:
            x1,y1,x2,y2 = box.xyxy[0]
            x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
            #cv2.rectangle(img, (x1, y1), (x2, y2), (255, 0, 255), 3)

            w,h = x2-x1, y2-y1
            cvzone.cornerRect(img, (x1,y1,w,h))

            conf = math.ceil(box.conf[0]*100) / 100
            cls = int(box.cls[0])
            cvzone.putTextRect(img, f'{classNames[cls]} {conf}', (max(0, x1),max(40, y1-20)))

            if classNames[cls] == 'ball' and conf > max_ball_conf:
                ball_center = (int(x1 + w/2), int(y1+h/2))
                ball_conf = conf
            elif classNames[cls] == 'hoop' and conf > max_hoop_conf:
                hoop_center = (int(x1 + w/2), int(y1+h/2))
                hoop_conf = conf
            elif classNames[cls] == 'player' and conf > max_player_conf:
                player_center = (int(x1 + w/2), int(y1 + h/2))

        
        if ball_center[0] >= 0 and hoop_center[0] > 0:
            cv2.line(img, ball_center, hoop_center, (0, 0, 0), 10)

    cv2.imshow("Image", img)
    cv2.waitKey(1)