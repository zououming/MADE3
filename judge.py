#coding=utf-8
import cv2
import numpy as np

def get_svm(path):
    svm = cv2.ml_SVM.load(path)
    return svm

def find_arrow(red_contours, frame):
    arrow_box = []
    contour_area = 0

    for red_contour in red_contours:
        if cv2.contourArea(red_contour) < 1000:
            break
        ellipse = cv2.fitEllipse(red_contour)
        rect = cv2.minAreaRect(cv2.boxPoints(ellipse))
        box = np.int0(cv2.boxPoints(rect))

        min_index = box.min(0)
        max_index = box.max(0)
        img = frame[min_index[1]:max_index[1], min_index[0]:max_index[0]]
        shape = np.shape(img)

        if shape[0] == 0 or shape[1] == 0: # or arrow_judge(img, arrow_svm) is False:
            continue

        rate = float(shape[0]) / shape[1]
        # if rate > 1.3 or rate < 0.7:
        #     continue

        arrow_box = cv2.boundingRect(box)
        contour_area = cv2.contourArea(red_contour)
        break

    return arrow_box, contour_area

def arrow_judge(img, svm):
    return True
    img = cv2.resize(img, (64, 64))
    gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    _, gray_img = cv2.threshold(gray_img, 0, 255, cv2.THRESH_OTSU)
    vector = np.ravel(gray_img).astype(np.float32)
    vector = np.resize(vector, (1, 64*64))
    _, result = svm.predict(vector)
    if result[0] > 0:
        return True
    else:
        return False
