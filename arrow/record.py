import cv2
import time

cap = cv2.VideoCapture(2)

width = cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
height = cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

writer = cv2.VideoWriter(f'{round(time.time())}.mp4', cv2.VideoWriter_fourcc(*'DIVX'), 20, (640, 480))


while True:
    ret, frame= cap.read()

    writer.write(frame)
    cv2.line(frame, (0, 200), (640, 200), (0, 0, 255), 3)
    cv2.imshow('frame', frame)

    if cv2.waitKey(1) & 0xFF == 27:
        break


cap.release()
writer.release()
cv2.destroyAllWindows()
