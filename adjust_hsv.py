#coding=utf-8
import cv2
import os
import numpy as np

def write_xml(green_lower, green_upper, red_lower, red_upper, xml_path):
    file_write = cv2.FileStorage(xml_path, cv2.FileStorage_WRITE)
    file_write.write('green_lower', np.ravel(green_lower))
    file_write.write('green_upper', np.ravel(green_upper))
    file_write.write('red_lower', np.ravel(red_lower))
    file_write.write('red_upper', np.ravel(red_upper))

    print('Saved to', xml_path)

def read_xml(xml_path):
    file_read = cv2.FileStorage(xml_path, cv2.FileStorage_READ)
    green_lower = np.ravel(file_read.getNode('green_lower').mat())
    green_upper = np.ravel(file_read.getNode('green_upper').mat())
    red_lower = np.ravel(file_read.getNode('red_lower').mat())
    red_upper = np.ravel(file_read.getNode('red_upper').mat())
    return green_lower, green_upper, red_lower, red_upper

def to_list(green_lower, green_upper, red_lower, red_upper):
    green_lower = green_lower.tolist()
    green_upper = green_upper.tolist()
    red_lower = red_lower.tolist()
    red_upper = red_upper.tolist()
    return green_lower, green_upper, red_lower, red_upper

def change_trackbar(self):
    pass

def adjust_hsv():
    global green_lower, green_upper, red_lower, red_upper
    adjust_window = "adjust HSV"
    cv2.namedWindow(adjust_window)

    cv2.createTrackbar("green lower H", adjust_window, green_lower[0], 180, change_trackbar)
    cv2.createTrackbar("green lower S", adjust_window, green_lower[1], 255, change_trackbar)
    cv2.createTrackbar("green lower V", adjust_window, green_lower[2], 255, change_trackbar)
    cv2.createTrackbar("green upper H", adjust_window, green_upper[0], 180, change_trackbar)
    cv2.createTrackbar("green upper S", adjust_window, green_upper[1], 255, change_trackbar)
    cv2.createTrackbar("green upper V", adjust_window, green_upper[2], 255, change_trackbar)

    cv2.createTrackbar("red lower H1", adjust_window, red_lower[0], 180, change_trackbar)
    cv2.createTrackbar("red lower H2", adjust_window, red_lower[1], 180, change_trackbar)
    cv2.createTrackbar("red lower S", adjust_window, red_lower[2], 255, change_trackbar)
    cv2.createTrackbar("red lower V", adjust_window, red_lower[3], 255, change_trackbar)
    cv2.createTrackbar("red upper H1", adjust_window, red_upper[0], 180, change_trackbar)
    cv2.createTrackbar("red upper H2", adjust_window, red_upper[1], 180, change_trackbar)
    cv2.createTrackbar("red upper S", adjust_window, red_upper[2], 255, change_trackbar)
    cv2.createTrackbar("red upper V", adjust_window, red_upper[3], 255, change_trackbar)

    cap = cv2.VideoCapture(0)
    while cap.isOpened():
        _, img = cap.read()
        hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        cv2.namedWindow("1")
        cv2.imshow("1", img)
        cv2.waitKey(1)

        shape = np.shape(img)
        print(shape, float(shape[0]) / shape[1])
        green_lower[0] = cv2.getTrackbarPos("green lower H", adjust_window)
        green_lower[1] = cv2.getTrackbarPos("green lower S", adjust_window)
        green_lower[2] = cv2.getTrackbarPos("green lower V", adjust_window)
        green_upper[0] = cv2.getTrackbarPos("green upper H", adjust_window)
        green_upper[1] = cv2.getTrackbarPos("green upper S", adjust_window)
        green_upper[2] = cv2.getTrackbarPos("green upper V", adjust_window)

        red_lower[0] = cv2.getTrackbarPos("red lower H1", adjust_window)
        red_lower[1] = cv2.getTrackbarPos("red lower H2", adjust_window)
        red_lower[2] = cv2.getTrackbarPos("red lower S", adjust_window)
        red_lower[3] = cv2.getTrackbarPos("red lower V", adjust_window)
        red_upper[0] = cv2.getTrackbarPos("red upper H1", adjust_window)
        red_upper[1] = cv2.getTrackbarPos("red upper H2", adjust_window)
        red_upper[2] = cv2.getTrackbarPos("red upper S", adjust_window)
        red_upper[3] = cv2.getTrackbarPos("red upper V", adjust_window)

        hsv_green = cv2.inRange(hsv_img, (green_lower[0], green_lower[1], green_lower[2]),
                                (green_upper[0], green_upper[1], green_upper[2]))

        hsv_red1 = cv2.inRange(hsv_img, (red_lower[0], red_lower[2], red_lower[3]),
                               (red_upper[0], red_upper[2], red_upper[3]))
        hsv_red2 = cv2.inRange(hsv_img, (red_lower[1], red_lower[2], red_lower[3]),
                               (red_upper[1], red_upper[2], red_upper[3]))
        hsv_red = cv2.bitwise_or(hsv_red1, hsv_red2)

        cv2.imshow("green", hsv_green)
        cv2.imshow("red", hsv_red)

        key = cv2.waitKey(1)
        if key == 32 or key == 27:
            cv2.destroyWindow("1")
            print('green lower:', green_lower)
            print('green upper:', green_upper)
            print('red lower:', red_lower)
            print('red upper:', red_upper)
            break
        if key == 27:
            break

    return green_lower, green_upper, red_lower, red_upper

green_lower = [43, 108, 55]
green_upper = [82, 255, 255]

red_lower = [0, 156, 91, 41]
red_upper = [10, 180, 255, 255]

if __name__ == '__main__':
    green_lower, green_upper, red_lower, red_upper = read_xml('setting/hsv.xml')
    green_lower, green_upper, red_lower, red_upper = to_list(green_lower, green_upper, red_lower, red_upper)
    green_lower, green_upper, red_lower, red_upper = adjust_hsv()
    save = input('save it? y/n: ')
    if save == 'y':
        write_xml(green_lower, green_upper, red_lower, red_upper, 'setting/hsv.xml')
