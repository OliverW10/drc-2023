import glob
import cv2
import shutil
import os

images = glob.glob("./set2-rights-raw/*.png")
print(f"found {len(images)} original images")
new_dir = "./set2-rights-labeled-whole/"
try:
    os.mkdir(new_dir)
except FileExistsError:
    pass

for i, image_file in enumerate(images):
    img = cv2.imread(image_file)
    cv2.imshow("image", cv2.resize(img, (40*4, 40*4)))
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
