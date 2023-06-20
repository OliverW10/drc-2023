import glob
import cv2
import numpy as np
import shutil
import os
import random

def rotate_image(image, angle, scale):
    if random.random() > 0.7:
        img1 = cv2.dilate(image, np.ones((3, 3)))
    else:
        img1 = image
    image_center = tuple(np.array(image.shape[1::-1]) / 2)
    rot_mat = cv2.getRotationMatrix2D(image_center, angle, scale)
    result = cv2.warpAffine(img1, rot_mat, image.shape[1::-1], flags=cv2.INTER_NEAREST)
    return result

images = glob.glob("./set3-labeled/*.png")
print(f"found {len(images)} original images")
new_dir = "./set3-labeled-augmented/"
try:
    os.mkdir(new_dir)
except FileExistsError:
    pass

for image_file in images:
    img_name = image_file.split("/")[-1].split("\\")[1].split(".")[0]
    shutil.copyfile(image_file, f"{new_dir}{img_name}-0.png")
    img = cv2.imread(image_file)
    img_flip = cv2.flip(img, 1)
    flipped_img_name = img_name
    if img_name[0] == "R":
        flipped_img_name = "L" + img_name[1:]
    elif img_name[0] == "L":
        flipped_img_name = "R" + img_name[1:]
    cv2.imwrite(f"{new_dir}{flipped_img_name}-1.png", img_flip)
    
    x = 2
    if img_name[0] == "R" or img_name[0] == "L":
        max_rot = 20
        min_zoom = 0.8
        max_zoom = 1.2
    else:
        max_rot = 180
        min_zoom = 0.5
        max_zoom = 1.4
    for i in range(2):
        for _img_name, _img in [(img_name, img), (flipped_img_name, img_flip)]:
            new_img = rotate_image(_img, random.randint(-max_rot, max_rot), random.uniform(min_zoom, max_zoom))
            cv2.imwrite(f"{new_dir}{_img_name}-{str(x)}.png", new_img)
            x += 1

print(f"created {len(glob.glob(new_dir+'*.png'))} images")
ratio = len(glob.glob(new_dir+'*.png')) / len(images)
print(f"ratio: {int(ratio)}x")