#coding=utf-8
import cv2
import adjust_hsv
import judge
import action
import time
import threading

def collect():
    global frame, hsv_red
    while 1:
        thread_lock.acquire()

        _, frame = capture.read()
        frame = cv2.resize(frame, (240, 160))
        hsv_img = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        hsv_red1 = cv2.inRange(hsv_img, (red_lower[0], red_lower[2], red_lower[3]),
                               (red_upper[0], red_upper[2], red_upper[3]))
        hsv_red2 = cv2.inRange(hsv_img, (red_lower[1], red_lower[2], red_lower[3]),
                               (red_upper[1], red_upper[2], red_upper[3]))
        hsv_red = cv2.bitwise_or(hsv_red1, hsv_red2)

        thread_lock.release()

        cv2.imshow("1", frame)
        cv2.imshow("red", hsv_red)
        cv2.waitKey(10)

def get_cnt_area(contour):
    return cv2.contourArea(contour)

def find_arrow(arrow_svm, direction):
    global frame, hsv_red
    model = "find"
    print("start find")

    while model != "quit":
        thread_lock.acquire()
        red_contours, _ = cv2.findContours(hsv_red, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        thread_lock.release()
        red_contours.sort(key=get_cnt_area, reverse=True)

        box, _ = judge.find_arrow(red_contours, frame, arrow_svm)

        sum_time = 0
        if model == "find":
            if len(box) > 0:
                print("find!!!")
                model = "adjust"
                time.sleep(1)

            if direction == "right":
                act.turn_right(act.find_pwm_duty_cycle)
                time.sleep(act.find_time)
                act.stop()
                sum_time += act.find_time
            elif direction == "left":
                act.turn_left(act.find_pwm_duty_cycle)
                time.sleep(act.find_time)
                act.stop()
                sum_time -= act.find_time

            if sum_time >= act.find_max_time:
                direction = "left"
                act.turn_left(act.find_pwm_duty_cycle)
                time.sleep(sum_time)
                sum_time = 0
            elif sum_time <= -act.find_max_time:
                direction = "right"
                act.turn_right(act.find_pwm_duty_cycle)
                time.sleep(sum_time)
                sum_time = 0

            time.sleep(act.find_time)

        if model == "adjust":
            if len(box) == 0:
                model = "find"
                continue
            print("adj")
            center = (box[0] + box[2]) / 2
            if center < act.center_min:
                act.turn_left(act.adj_pwm_duty_cycle)
                time.sleep(0.1)
                act.stop()
            elif center > act.center_max:
                act.turn_right(act.adj_pwm_duty_cycle)
                time.sleep(0.1)
                act.stop()
            else:
                print("quit")
                model = "quit"
            time.sleep(0.2)

def is_destination(arrow_svm):
    global frame, hsv_red

    thread_lock.acquire()
    red_contours, _ = cv2.findContours(hsv_red, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    thread_lock.release()

    red_contours.sort(key=get_cnt_area, reverse=True)

    box, contour = judge.find_arrow(red_contours, frame, arrow_svm)

    if len(box) > 0 and cv2.contourArea(contour) > act.destination_area:
        (x, y, w, h) = [int(v) for v in box]
        center = (x + w) / 2  # 320 240

        if center >= act.center_min and center <= act.center_max:
            return True

    return False

def start():
    global green_lower, green_upper, red_lower, red_upper, capture, frame
    arrow_svm = judge.get_svm("setting/arrow.xml")
    act.stop()
    print("start")
    find_arrow(arrow_svm, "left")
    print("find arrow!")
    adj_time = 0
    last_time = 0
    try:
        while 1:
            act.go_straight(act.straight_pwm_duty_cycle)
            act.update_distance()

            if (act.distance[0] < act.max_distance or act.distance[1] < act.max_distance) and is_destination(arrow_svm) is False:
                last_time = time.time()
                adj_time += act.avoid_return()
            elif act.distance[0] < act.stop_distance and act.distance[1] < act.stop_distance:
                act.stop()
                print("reach destination!")
                break

            if time.time() - last_time > act.adj_time and time.time() - last_time < act.find_destination_time:
                if adj_time > 0:
                    act.turn_right(act.find_pwm_duty_cycle)
                    time.sleep(adj_time)
                    last_time = time.time()
                elif adj_time < 0:
                    act.turn_left(act.find_pwm_duty_cycle)
                    time.sleep(-adj_time)
                    last_time = time.time()
                adj_time = 0
            elif time.time() - last_time > act.find_destination_time:
                find_arrow(arrow_svm, "right")
                last_time = time.time()

    except KeyboardInterrupt:
        act.end()

capture = cv2.VideoCapture(0)
act = action.Action()
thread_lock = threading.Lock()

if __name__ == "__main__":
    xml_path = "setting/hsv.xml"
    print("read xml file:", xml_path)
    green_lower, green_upper, red_lower, red_upper = adjust_hsv.read_xml(xml_path)
    green_lower, green_upper, red_lower, red_upper = adjust_hsv.to_list(green_lower, green_upper, red_lower, red_upper)

    print("green lower:", green_lower)
    print("green upper:", green_upper)
    print("red lower:", red_lower)
    print("red upper:", red_upper)

    try:
        collect_thread = threading.Thread(target=collect)
        car_thread = threading.Thread(target=start)

        collect_thread.start()
        car_thread.start()

        collect_thread.join()
        car_thread.join()

    except:
        print("error")

    while 1:
        pass

