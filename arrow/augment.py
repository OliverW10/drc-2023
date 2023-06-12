import glob
import cv2
import numpy as np
import shutil
import os
import random

def rotate_image(image, angle, scale):
    image_center = tuple(np.array(image.shape[1::-1]) / 2)
    rot_mat = cv2.getRotationMatrix2D(image_center, angle, scale)
    result = cv2.warpAffine(image, rot_mat, image.shape[1::-1], flags=cv2.INTER_NEAREST)
    return result

images = glob.glob("./custom-base/negative/*.png")
print(f"found {len(images)} original images")
new_dir = "./custom-base/negative-augmented/"
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
    for i in range(15):
        new_img = rotate_image(img, random.randint(-180, 180), random.uniform(0.5, 1.2))
        cv2.imwrite(f"{new_dir}{img_name}-{str(x)}.png", new_img)
        x += 1

        new_img = rotate_image(img_flip, random.randint(-180, 180), random.uniform(0.5, 1.2))
        cv2.imwrite(f"{new_dir}{flipped_img_name}-{str(x)}.png", new_img)
        x += 1

print(f"created {len(glob.glob(new_dir+'*.png'))} images")
ratio = len(glob.glob(new_dir+'*.png')) / len(images)
print(f"ratio: {int(ratio)}x")