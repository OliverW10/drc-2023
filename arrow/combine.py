import os
import random
import shutil

# dirname should include trailing slash
def collect_images_from_dir(dirname, percent = 1):
    imgs = os.listdir(dirname)
    return list([dirname + i for i in imgs if random.random() < percent])

all_images = [
    *collect_images_from_dir("custom-base/negative-augmented/"),
    *collect_images_from_dir("custom-base/positive-augmented/"),
    *collect_images_from_dir("set1/"),
    *collect_images_from_dir("set2-lefts-labeled-augmented/"),
    *collect_images_from_dir("set2-rights-labeled-augmented/"),
    *collect_images_from_dir("set3-labeled-augmented/"),
]

random.shuffle(all_images)

print("total:", len(all_images))

m = {"R": "right/", "L": "left/", "N": "negative/"}
for idx, imgname in enumerate(all_images):
    label = imgname.split("/")[-1][0]
    outname = "combined/" + m[label] + str(idx) + ".png"
    # print(imgname, " \t", outname)
    shutil.copy(imgname, outname)