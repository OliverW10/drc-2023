import glob
import cv2
import numpy as np
import shutil
import os
import random
import time

images = glob.glob("./images/dataset/*.png")
print(f"found {len(images)} original images")
new_dir = "./images/labeled/"
try:
    os.mkdir(new_dir)
except FileExistsError:
    pass

for i, image_file in enumerate(images):
    img = cv2.imread(image_file)
    cv2.imshow("image", img)
    label = ""
    while label == "":
        ch = cv2.waitKey(-1)
        if ch == 27: # esc
            quit()
        if ch == ord('a'):
            label = "L"
        elif ch == ord('d'):
            label = "R"
        elif ch == ord('s'):
            label = "N"

    shutil.copyfile(image_file, f"{new_dir}{label}{str(i)}.png")
