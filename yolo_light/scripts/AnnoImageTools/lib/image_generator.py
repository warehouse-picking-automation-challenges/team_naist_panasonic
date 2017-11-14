#
# Version:  2017.07.31
# Authors:  Members of the Team NAIST-Panasonic at the Amazon Robotics Challenge 2017:
#           Gustavo A. Garcia R. <garcia-g at is.naist.jp> (Captain), 
#           Lotfi El Hafi, Felix von Drigalski, Wataru Yamazaki, Viktor Hoerig, Arnaud Delmotte, 
#           Akishige Yuguchi, Marcus Gall, Chika Shiogama, Kenta Toyoshima, Pedro Uriguen, 
#           Rodrigo Elizalde, Masaki Yamamoto, Yasunao Okazaki, Kazuo Inoue, Katsuhiko Asai, 
#           Ryutaro Futakuchi, Seigo Okada, Yusuke Kato, and Pin-Chu Yang
#####################
# Copyright 2017 Team NAIST-Panasonic 
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at 
#     http://www.apache.org/licenses/LICENSE-2.0 
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#####################

import cv2
import os
import glob
import numpy as np
from PIL import Image
from lib.utils import *
from IPython.terminal.debugger import set_trace as keyboard

# src_imageの背景画像に対して、overlay_imageのalpha画像を貼り付ける。pos_xとpos_yは貼り付け時の左上の座標
def overlay(src_image, overlay_image, pos_x, pos_y):
    # オーバレイ画像のサイズを取得
    ol_height, ol_width = overlay_image.shape[:2]

    # OpenCVの画像データをPILに変換
    # BGRAからRGBAへ変換
    src_image_RGBA = cv2.cvtColor(src_image, cv2.COLOR_BGR2RGB)
    overlay_image_RGBA = cv2.cvtColor(overlay_image, cv2.COLOR_BGRA2RGBA)

    #　PILに変換
    src_image_PIL=Image.fromarray(src_image_RGBA)
    overlay_image_PIL=Image.fromarray(overlay_image_RGBA)

    # 合成のため、RGBAモードに変更
    src_image_PIL = src_image_PIL.convert('RGBA')
    overlay_image_PIL = overlay_image_PIL.convert('RGBA')

    # 同じ大きさの透過キャンパスを用意
    tmp = Image.new('RGBA', src_image_PIL.size, (255, 255,255, 0))
    # 用意したキャンパスに上書き
    tmp.paste(overlay_image_PIL, (pos_x, pos_y), overlay_image_PIL)
    # オリジナルとキャンパスを合成して保存
    result = Image.alpha_composite(src_image_PIL, tmp)

    return  cv2.cvtColor(np.asarray(result), cv2.COLOR_RGBA2BGRA)

# 画像周辺のパディングを削除
def delete_pad(image):
    orig_h, orig_w = image.shape[:2]
    mask = np.argwhere(image[:, :, 3] > 128) # alphaチャンネルの条件、!= 0 や == 255に調整できる
    (min_y, min_x) = (max(min(mask[:, 0])-1, 0), max(min(mask[:, 1])-1, 0))
    (max_y, max_x) = (min(max(mask[:, 0])+1, orig_h), min(max(mask[:, 1])+1, orig_w))
    return image[min_y:max_y, min_x:max_x]

# 画像を指定した角度だけ回転させる
def rotate_image(image, angle):
    orig_h, orig_w = image.shape[:2]
    matrix = cv2.getRotationMatrix2D((orig_h/2, orig_w/2), angle, 1)
    return cv2.warpAffine(image, matrix, (orig_h, orig_w))

# 画像をスケーリングする
def scale_image(image, scale):
    orig_h, orig_w = image.shape[:2]
    return cv2.resize(image, (int(orig_w*scale), int(orig_h*scale)))

# 背景画像から、指定したhとwの大きさの領域をランダムで切り抜く
def random_sampling(image, h, w):
    orig_h, orig_w = image.shape[:2]
    y = np.random.randint(orig_h-h+1)
    x = np.random.randint(orig_w-w+1)
    return image[y:y+h, x:x+w]

# 画像をランダムに回転、スケールしてから返す
def random_rotate_scale_image(image, min_scale, max_scale, rand_angle):
    image = rotate_image(image, np.random.randint(rand_angle*2)-rand_angle)
    image = scale_image(image, min_scale + np.random.rand() * (max_scale-min_scale)) # 1 ~ 3倍
    return delete_pad(image)

# overlay_imageを、src_imageのランダムな場所に合成して、そこのground_truthを返す。
def random_overlay_image(src_image, overlay_image, minimum_crop):
    src_h, src_w = src_image.shape[:2]
    overlay_h, overlay_w = overlay_image.shape[:2]
    shift_item_h, shift_item_w = overlay_h * (1-minimum_crop), overlay_w * (1-minimum_crop)
    scale_item_h, scale_item_w = overlay_h * (minimum_crop*2-1), overlay_w * (minimum_crop*2-1)
    y_max = src_h-scale_item_h
    x_max = src_w-scale_item_w
    if y_max > 0:
        y = int(np.random.randint(src_h-scale_item_h) - shift_item_h)
    else:
        y = int(- shift_item_h)

    if x_max > 0:
        x = int(np.random.randint(src_w-scale_item_w) - shift_item_w)
    else:
        x = int(-shift_item_w)
    image = overlay(src_image, overlay_image, x, y)
    bbox = ((np.maximum(x, 0), np.maximum(y, 0)), (np.minimum(x+overlay_w, src_w-1), np.minimum(y+overlay_h, src_h-1)))

    return image, bbox


# Not finish yet!
def random_overlay_image_inrange(src_image, overlay_image, minimum_crop, range_of_overlay):
    src_h, src_w = src_image.shape[:2]
    overlay_h, overlay_w = overlay_image.shape[:2]
    shift_item_h, shift_item_w = overlay_h * (1-minimum_crop), overlay_w * (1-minimum_crop)
    scale_item_h, scale_item_w = overlay_h * (minimum_crop*2-1), overlay_w * (minimum_crop*2-1)
    y_max = src_h-scale_item_h
    x_max = src_w-scale_item_w
    range_xmin=range_of_overlay[0][0]
    range_xmax=range_of_overlay[0][1]
    range_ymin=range_of_overlay[1][0]
    range_ymax=range_of_overlay[1][1]
    try_count=0
    while(True):
        if y_max > 0:
            y = int(np.random.randint(src_h-scale_item_h) - shift_item_h)
        else:
            y = int(- shift_item_h)
        if x_max > 0:
            x = int(np.random.randint(src_w-scale_item_w) - shift_item_w)
        else:
            x = int(-shift_item_w)
        #print("x:"+str(x)+" y:"+str(y))

        if try_count<300:
            if x > range_xmin and (x+overlay_w) < range_xmax:
                if y > range_ymin and (y+overlay_h) < range_ymax:
                    break
        else:
            print("Some object is too big that cannot fit in the range. It will be random placed in the image.")
            break
        try_count+=1
    image = overlay(src_image, overlay_image, x, y)
    bbox = ((np.maximum(x, 0), np.maximum(y, 0)), (np.minimum(x+overlay_w, src_w-1), np.minimum(y+overlay_h, src_h-1)))
    return image, bbox

# 4点座標のbboxをyoloフォーマットに変換
def yolo_format_bbox(image, bbox):
    orig_h, orig_w = image.shape[:2]
    center_x = (bbox[1][0] + bbox[0][0]) / 2 / orig_w
    center_y = (bbox[1][1] + bbox[0][1]) / 2 / orig_h
    w = (bbox[1][0] - bbox[0][0]) / orig_w
    h = (bbox[1][1] - bbox[0][1]) / orig_h
    return(center_x, center_y, w, h)

def maximum_iou(box, boxes):
    max_iou = 0
    for src_box in boxes:
        iou = box_iou(box, src_box)
        if iou > max_iou:
            max_iou = iou
    return max_iou

# Calculate the old_box be covered rate
def BeOverlaid(box_old,box_new):
    box_old_area = box_area(box_old)
    intersection_area = box_intersection(box_new,box_old)
    percentage = intersection_area/box_old_area
    return percentage

def maximum_BeOverlaid(new_box,old_boxes):
    max_beoverlaid = 0.0
    for src_old_box in old_boxes:
        beoverlaid = BeOverlaid(src_old_box,new_box)
        if beoverlaid > max_beoverlaid:
            max_beoverlaid = beoverlaid
    return max_beoverlaid


class ImageGenerator():
    def __init__(self, item_path, background_path,split_class=""):
        self.bg_files = glob.glob(background_path + "/*")
        self.item_files = glob.glob(item_path + "/*")
        self.items = []
        self.labels = []
        self.class_lst=[]
        self.bgs = []
        for item_file in self.item_files:
            image = cv2.imread(item_file, cv2.IMREAD_UNCHANGED)
            center = np.maximum(image.shape[0], image.shape[1])
            pixels = np.zeros((center*2, center*2, image.shape[2]))
            y = int(center - image.shape[0]/2)
            x = int(center - image.shape[1]/2)
            pixels[y:y+image.shape[0], x:x+image.shape[1], :] = image
            if split_class == "":
                self.labels.append([])
                self.labels[0].append(item_file.split("/")[-1].split(".")[0])
                self.items.append([])
                self.items[0].append(pixels.astype(np.uint8))
            else:
                item_class = (item_file.split("/")[-1].split(".")[0]).split(split_class)[0]
                if item_class not in self.class_lst:
                    self.class_lst.append(item_class)
                    self.labels.append([])
                    self.items.append([])
                id_in_class_lst=self.class_lst.index(item_class)
                self.labels[id_in_class_lst].append(item_file.split("/")[-1].split(".")[0])
                self.items[id_in_class_lst].append(pixels.astype(np.uint8))
        print("Labels:"+str(self.class_lst))
        print("Labels Amount:"+str(len(self.class_lst)))

        for bg_file in self.bg_files:
            self.bgs.append(cv2.imread(bg_file))

    def generate_samples(self, n_samples, n_items, crop_width, crop_height, min_item_scale, max_item_scale, rand_angle, minimum_crop, delta_hue, delta_sat_scale, delta_val_scale,max_BeOverlaid,generate_once=False,range_of_overlay=None):
        x = []
        t = []
        for i in range(n_samples):
            bg = self.bgs[np.random.randint(len(self.bgs))]
            sample_image = random_sampling(bg, crop_height, crop_width)
            ground_truths = []
            boxes = []
            used_class = []
            for j in range(np.random.randint(n_items)+1):
                if generate_once:
                    if n_items > len(self.class_lst):
                        print("Error! If you activate generate_once=True, your classes must much more than n_items")
                        print("n_items:"+str(n_items)+" Item classes:"+str(len(self.class_lst)))
                        exit()
                    while(True):
                        class_id = np.random.randint(len(self.class_lst))
                        if not self.class_lst[class_id] in used_class:
                            used_class.append(self.class_lst[class_id])

                            break
                    file_id = np.random.randint(len(self.labels[class_id]))
                else:
                    class_id = np.random.randint(len(self.class_lst))
                    file_id = np.random.randint(len(self.labels[class_id]))
                item = self.items[class_id][file_id]
                item = random_rotate_scale_image(item, min_item_scale, max_item_scale, rand_angle)
                if range_of_overlay == None:
                    tmp_image, bbox = random_overlay_image(sample_image, item, minimum_crop)
                else:
                    tmp_image, bbox = random_overlay_image_inrange(sample_image, item, minimum_crop, range_of_overlay)
                yolo_bbox = yolo_format_bbox(tmp_image, bbox)
                box = Box(yolo_bbox[0], yolo_bbox[1], yolo_bbox[2], yolo_bbox[3])
                if (maximum_iou(box, boxes) < 0.3) and (maximum_BeOverlaid(box,boxes) < max_BeOverlaid):
                    boxes.append(box)
                    one_hot_label = np.zeros(len(self.labels))
                    one_hot_label[class_id] = 1
                    ground_truths.append({
                        "x": yolo_bbox[0],
                        "y": yolo_bbox[1],
                        "w": yolo_bbox[2],
                        "h": yolo_bbox[3],
                        "classname": self.class_lst[class_id],
                        "label": class_id,
                        "one_hot_label": one_hot_label
                    })
                    sample_image = tmp_image[:, :, :3]
            t.append(ground_truths)

            sample_image = random_hsv_image(sample_image, delta_hue, delta_sat_scale, delta_val_scale)

            sample_image = sample_image.transpose(2, 0, 1)
            x.append(sample_image)
        x = np.array(x)
        return x, t

