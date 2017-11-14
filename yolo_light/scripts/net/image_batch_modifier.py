import cv2
import numpy as np
from IPython.core.debugger import Tracer; keyboard = Tracer()
from scipy.interpolate import UnivariateSpline


def create_LUT_8UC1(x, y):
    spl = UnivariateSpline(x, y,k=2)
    return spl(xrange(256))

def _get_images_from_batches(batch):
    batch_size = batch.shape[0]
    img_width = batch.shape[1]
    img_height = batch.shape[2]
    img_channel = batch.shape[3]
    imgs = np.split(batch,batch_size)
    reshaped_imgs = []
    for img in imgs:
        img = img.reshape(img_width,img_height,img_channel)
        reshaped_imgs.append(img)
    return reshaped_imgs,img_width,img_height,img_channel

def trans2uint(batch):
    batch = np.interp(batch,[0,1],[0,255])
    batch = np.ndarray.astype(batch,'uint8')
    return batch

def trans2float(batch):
    batch = np.interp(batch,[0,255],[0,1])
    batch = np.ndarray.astype(batch,'float64')
    return batch

def add_noise_batch(batch,level):
    noiselevel = np.sqrt(level)
    gaussian_noise = np.random.normal(0,noiselevel,size = np.shape(batch))
    noisy_batch = batch+gaussian_noise
    noisy_batch = np.clip(noisy_batch,0.0,1.0)
    return noisy_batch

def adjust_gamma_batch(batch,gammalevel=(1,3)):
    imgs,_,_,_ = _get_images_from_batches(batch)
    gammaed_imgs=[]
    for img in imgs:
        gamma = np.random.uniform(gammalevel[0],gammalevel[1])
        gammaed_imgs.append(_adjusting_gamma(img,gamma))
    batch = np.array(gammaed_imgs)
    return batch

def apply_blur_batch(batch,kernelmax):
    imgs,_,_,_ = _get_images_from_batches(batch)
    blur_imgs = []
    for img in imgs:
        kernel = np.random.randint(int(kernelmax))
        if kernel == 0:
            blur_imgs.append(img)
        else:
            blur_imgs.append(_apply_blur(img,kernel))
    batch = np.array(blur_imgs)
    return batch

def adjust_saturation_batch(batch,valuelevel=0.2):
    imgs,_,_,_ = _get_images_from_batches(batch)
    saturated_imgs = []
    for img in imgs:
        value = np.random.uniform((1/(1+valuelevel)),1+valuelevel)
        saturated_imgs.append(_adjusting_saturation(img,value))
    batch = np.array(saturated_imgs)
    return batch

def adjust_exposure_batch(batch,valuelevel=0.2):
    imgs,_,_,_ = _get_images_from_batches(batch)
    exposure_imgs = []
    for img in imgs:
        value = np.random.uniform((1/(1+valuelevel)),1+valuelevel)
        exposure_imgs.append(_adjusting_exposure(img,value))
    batch = np.array(exposure_imgs)
    return batch


def apply_filter_batch(batch):
    batch = trans2uint(batch)
    imgs,_,_,_ = _get_images_from_batches(batch)
    filted_img=[]
    for img in imgs:
        option = np.random.randint(2)
        if option == 0:
            filted_img.append(img)
        elif option == 1:
            filted_img.append(_apply_filter(img_bgr_in,"warming"))
        elif option == 2:
            filted_img.append(_apply_filter(img_bgr_in,"cold"))
    batch = np.array(filted_img)
    return batch

def _adjusting_gamma(image,gamma):
    invGamma = 1.0 / gamma
    table = np.array([((i / 255.0) ** invGamma) * 255
       for i in np.arange(0, 256)]).astype("uint8")
    gammaed_image = cv2.LUT(image, table)
    #apply gamma correction using the lookup table
    return gammaed_image

def _apply_shifting(img,x,y):
    rows,cols,chs = img.shape
    M = np.float32([[1,0,x],[0,1,y]])
    dst = cv2.warpAffine(img,M,(cols,rows))
    return dst

def _apply_blur(img,kernellevel):
    img_blur = cv2.blur(img,(kernellevel,kernellevel))
    return img_blur

def _apply_filter(img_bgr_in,filter):
    img_gray = cv2.cvtColor(img_rgb_in, cv2.COLOR_RGB2GRAY)
    anchor_x = [0, 128, 255]
    anchor_y = [0, 192, 255]
    myLUT = create_LUT_8UC1(anchor_x, anchor_y)
    img_curved = cv2.LUT(img_gray, myLUT).astype(np.uint8)
    incr_ch_lut = create_LUT_8UC1([0, 64, 128, 192, 256],
                [0, 70, 140, 210, 256])
    decr_ch_lut = create_LUT_8UC1([0, 64, 128, 192, 256],
                [0, 30, 80, 120, 192])
    if filter == "warming":
        c_b, c_g, c_r = cv2.split(img_bgr_in)
        c_r = cv2.LUT(c_r, incr_ch_lut).astype(np.uint8)
        c_b = cv2.LUT(c_b, decr_ch_lut).astype(np.uint8)
        img_bgr_warm = cv2.merge((c_b, c_g, c_r))
        c_b = cv2.LUT(c_b, decr_ch_lut).astype(np.uint8)

        # increase color saturation
        c_h, c_s, c_v = cv2.split(cv2.cvtColor(img_bgr_warm,
            cv2.COLOR_BGR2HSV))
        c_s = cv2.LUT(c_s, incr_ch_lut).astype(np.uint8)
        img_bgr_warm = cv2.cvtColor(cv2.merge(
            (c_h, c_s, c_v)),
            cv2.COLOR_HSV2BGR)
        return img_bgr_warm

    elif filter == "cold":
        c_b, c_g, c_r = cv2.split(img_bgr_in)
        c_r = cv2.LUT(c_r, decr_ch_lut).astype(np.uint8)
        c_b = cv2.LUT(c_b, incr_ch_lut).astype(np.uint8)
        img_bgr_cold = cv2.merge((c_b, c_g, c_r))

        # decrease color saturation
        c_h, c_s, c_v = cv2.split(cv2.cvtColor(img_bgr_cold,
            cv2.COLOR_BGR2HSV))
        c_s = cv2.LUT(c_s, decr_ch_lut).astype(np.uint8)
        img_bgr_cold = cv2.cvtColor(cv2.merge(
            (c_h, c_s, c_v)),
            cv2.COLOR_HSV2BGR)
        return img_bgr_cold


def _adjusting_saturation(img,value):
    hsv = cv2.cvtColor(img,cv2.COLOR_RGB2HSV)
    hsv = hsv.astype(np.float64)
    hsv[:,:,1] = hsv[:,:,1]*value
    hsv[:,:,1] = np.clip(hsv[:,:,1],0.0,255.0)
    hsv = hsv.astype(np.uint8)
    image = cv2.cvtColor(hsv,cv2.COLOR_HSV2RGB)
    return image

def _adjusting_exposure(img,value):
    hsv = cv2.cvtColor(img,cv2.COLOR_RGB2HSV)
    hsv = hsv.astype(np.float64)
    hsv[:,:,2] = hsv[:,:,2]*value
    hsv[:,:,2] = np.clip(hsv[:,:,2],0.0,255.0)
    hsv = hsv.astype(np.uint8)
    image = cv2.cvtColor(hsv,cv2.COLOR_HSV2RGB)
    return image



def apply_flip_x(img):
    return cv2.flip(img, 0)

def apply_flip_y(img):
    return cv2.flip(img, 1)

def apply_flip_xy(img):
    return cv2.flip(img, -1)

def apply_resize(img):
    LinerImg = cv2.resize(img, size, interpolation = cv2.INTER_LINER)
    return LinerImg
