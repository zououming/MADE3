#coding=utf-8
import time
import cv2
import RPi.GPIO as GPIO

class Action():
    distance = [float(0) ,float(0)]

    left_trig = 16
    left_echo = 18

    right_trig = 36
    right_echo = 38

    right_input1 = 5
    right_input2 = 7
    right_enable = 3
    right_pwm = []

    left_input1 = 11
    left_input2 = 13
    left_enable = 15
    left_pwm = []

    find_pwm_duty_cycle = 50
    adj_pwm_duty_cycle = 30
    straight_pwm_duty_cycle = [25, 25]

    center_min = 90
    center_max = 150

    find_time = 0.2
    find_max_time = 6 * find_time

    turn_time = 0.1
    turn_right_max_time = 2

    adj_time = 2
    find_destination_time = 5

    max_distance = 15
    stop_distance = 10

    destination_area = 2000

    def __init__(self):
        self.GPIO_init()

    def GPIO_init(self):
        GPIO.setmode(GPIO.BOARD)

        GPIO.setup(self.left_trig, GPIO.OUT, initial=GPIO.LOW) #超声波
        GPIO.setup(self.left_echo, GPIO.IN)

        GPIO.setup(self.right_trig, GPIO.OUT, initial=GPIO.LOW) #超声波
        GPIO.setup(self.right_echo, GPIO.IN)

        GPIO.setup(self.right_input1, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.right_input2, GPIO.OUT, initial=GPIO.HIGH)
        GPIO.setup(self.right_enable, GPIO.OUT)

        GPIO.setup(self.left_input1, GPIO.OUT, initial=GPIO.HIGH)
        GPIO.setup(self.left_input2, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.left_enable, GPIO.OUT)
        self.left_pwm = GPIO.PWM(self.left_enable, 20)
        self.right_pwm = GPIO.PWM(self.right_enable, 20)

        self.left_pwm.start(0)
        self.right_pwm.start(0)
        print("init")

    def turn_right(self, pwm):
        GPIO.output(self.right_input1, GPIO.LOW)
        GPIO.output(self.right_input2, GPIO.HIGH)

        GPIO.output(self.left_input1, GPIO.HIGH)
        GPIO.output(self.left_input2, GPIO.LOW)

        self.left_pwm.ChangeDutyCycle(pwm)
        self.right_pwm.ChangeDutyCycle(pwm)

        print("turn right")

    def turn_left(self, pwm):
        GPIO.output(self.right_input1, GPIO.HIGH)
        GPIO.output(self.right_input2, GPIO.LOW)

        GPIO.output(self.left_input1, GPIO.LOW)
        GPIO.output(self.left_input2, GPIO.HIGH)

        self.left_pwm.ChangeDutyCycle(pwm)
        self.right_pwm.ChangeDutyCycle(pwm)

        print("turn left")

    def go_straight(self, pwm):
        GPIO.output(self.right_input1, GPIO.LOW)
        GPIO.output(self.right_input2, GPIO.HIGH)

        GPIO.output(self.left_input1, GPIO.LOW)
        GPIO.output(self.left_input2, GPIO.HIGH)

        self.left_pwm.ChangeDutyCycle(pwm[0])
        self.right_pwm.ChangeDutyCycle(pwm[1])
        print("go straight")

    def stop(self):
        GPIO.output(self.right_input1, GPIO.LOW)
        GPIO.output(self.right_input2, GPIO.LOW)

        GPIO.output(self.left_input1, GPIO.LOW)
        GPIO.output(self.left_input2, GPIO.LOW)

    def update_distance(self):
        self.distance[0] = self.get_distance("left")
        self.distance[1] = self.get_distance("right")
        print(self.distance)

    def get_distance(self, direction):
        if direction == "left":
            trig = self.left_trig
            echo = self.left_echo
        else:
            trig = self.right_trig
            echo = self.right_echo

        # GPIO.setup(echo, GPIO.OUT)
        # GPIO.output(echo, GPIO.HIGH)
        # time.sleep(5)

        GPIO.output(trig, GPIO.HIGH)
        time.sleep(0.000015)
        GPIO.output(trig, GPIO.LOW)

        while GPIO.input(echo) == 0:
            pass
        t1 = time.time()
        while GPIO.input(echo) == 1:
            pass
        t2 = time.time()

        return (t2 - t1) * 343 / 2 * 100

    def avoid_return(self):
        avoid_time = 0
        direction = ""
        print("avoid")
        while self.distance[0] < self.max_distance or self.distance[1] < self.max_distance:
            if self.distance[1] < self.max_distance:
                direction = "left"
                self.turn_left(self.find_pwm_duty_cycle)
            else:
                direction = "right"
                self.turn_right(self.find_pwm_duty_cycle)

            time.sleep(self.turn_time)
            self.stop()
            time.sleep(self.turn_time)

            avoid_time += self.turn_time
            self.update_distance()

        print("avoid end")

        if direction == "left":
            return avoid_time
        else:
            return -avoid_time

    def end(self):
        self.stop()
        GPIO.clearup()