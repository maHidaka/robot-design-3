import time
import urllib.request
import cv2
import numpy as np
import math

#
# マニピュレータ操作
#


class RV2AJ():
    def __init__(self, robot_ip="127.0.0.01"):
        self.robot_ip = robot_ip
        self.robot_angles = "0,0,0,0,0"
        self.robot_ev_status = "OFF"

    def _post(self, uri):
        req = urllib.request.Request(uri)
        print(uri)
        with urllib.request.urlopen(req) as res:
            body = res.read()

    def _send_request(self, target_angles="", target_ev_status=""):
        if target_ev_status == "":
            self._post(
                "http://{}/angles.py?angles={}".format(self.robot_ip, target_angles))
        elif target_angles == "":
            self._post(
                "http://{}/angles.py?ev={}".format(self.robot_ip, target_ev_status))
        else:
            self._post("http://{}/angles.py?angles={}?ev={}".format(self.robot_ip,
                                                                    target_angles, target_ev_status))

    def set_angles(self, angles):
        self._send_request(target_angles=angles)

    def set_ev(self, ev_status):
        if ev_status == "ON" or ev_status == "1" or ev_status == 1:
            self._send_request(target_ev_status="ON")
        elif ev_status == "OFF" or ev_status == "0" or ev_status == 0:
            self._send_request(target_ev_status="OFF")

#
# 射撃
#


def fire():
    print("fire")
    robot.set_ev("ON")
    time.sleep(0.5)
    robot.set_ev("OFF")

#
# 矩形認識
#


def find_rect_of_target_color(image, color_l, color_h):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV_FULL)
    h = hsv[:, :, 0]
    s = hsv[:, :, 1]
    mask = np.zeros(h.shape, dtype=np.uint8)
    color_l = color_l*(255/360)
    color_h = color_h*(255/360)
    mask[((h < color_h) & (h > color_l)) & (s > 110)] = 255
    contours, _ = cv2.findContours(
        mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    rects = []

    for contour in contours:
        approx = cv2.convexHull(contour)
        area = cv2.contourArea(approx)
        if area < 1e2 or 1e5 < area:  # 1e2以下、1e5以上の画像が着たらc中断してfor先頭にジャンプ
            continue
        rect = cv2.boundingRect(approx)
        rectl = list(rect)
        rectl.append(int(area))
        rects.append(np.array(rectl))
    return rects

# イメージの中心座標を返す


def frame_centor():
    width = capture.get(cv2.CAP_PROP_FRAME_WIDTH)
    height = capture.get(cv2.CAP_PROP_FRAME_HEIGHT)
    width = int(width) // 2
    height = int(height) // 2
    return width, height

# 与えられたx,y値から属する象限を返す


def orthant(x, y):
    if x*y < 0:
        if x < 0:
            orthant = 2
        else:
            orthant = 4
    elif x < 0:
        orthant = 3
    else:
        orthant = 1

    return orthant

# 与えられたx,y値から目標間接角を返す


def arg(x, y, argX, argY):

    sleeptime = 0.05
    movex = argX - (x * 0.025)
    movey = argY + (y * 0.02)
    spt = max([abs(movex - argX), abs(movey - argY)]) * sleeptime
    if spt < sleeptime:
        spt = sleeptime
    time.sleep(spt)

    argX = int(movex)
    argY = int(movey)
    return argX, argY

#########################################
# main
#########################################


if __name__ == "__main__":
    capture = cv2.VideoCapture(1)
    capture.set(3, 1080)  # Width
    capture.set(4, 960)  # Heigh
    centor_width, centor_height = frame_centor()
    robot = RV2AJ("10.232.169.213")
    robot.set_ev("OFF")
    print("ev=on")
    #
    robot.set_angles("0,0,0,0,0")
    #
    print("0,0,0,0,0")
    time.sleep(2.0)
    #
    defaultX = 0
    defaultY = 60
    robot.set_angles("0,-35,65,60,45")
    #
    time.sleep(1.0)

    ################stand by#########
    while 1:
        k = cv2.waitKey(10)
        if k == ord("f"):
            fire()
            time.sleep(0.5)
        if k == ord("s"):
            break

        _, frame = capture.read()
        rects = find_rect_of_target_color(frame, 200, 250)
        if len(rects) > 0:
            rect = max(rects, key=(lambda x: x[2] * x[3]))
            obj_centor_x = rect[0]+(rect[2] // 2)
            obj_centor_y = rect[1]+(rect[3] // 2)
            Area = rect[4]
            dist = (29700/float(Area)) * 2
            dx = obj_centor_x - centor_width
            dy = obj_centor_y - centor_height
            object_centor = "x="+str(obj_centor_x)+",y="+str(obj_centor_y)
            difference = "difference: x=" + \
                str(dx)+",y="+str(dy)+" ,distance"+str(dist)

            cv2.circle(frame, (obj_centor_x, obj_centor_y),
                       5, (0, 255, 0), thickness=-1)
            cv2.rectangle(frame, tuple(rect[0:2]), tuple(
                rect[0:2] + rect[2:4]), (0, 0, 255), thickness=2)
            cv2.putText(frame, object_centor, (rect[0], rect[1]-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), thickness=1)
            cv2.putText(frame, difference, (rect[0], rect[1]-30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), thickness=1)
            cv2.putText(frame, "Stand by......", (30, 450),
                        cv2.FONT_HERSHEY_SIMPLEX, 7, (0, 0, 255), thickness=15)
        else:
            cv2.putText(frame, "LOST", (30, 450),
                        cv2.FONT_HERSHEY_SIMPLEX, 13, (0, 0, 255), thickness=20)
        cv2.imshow('position', frame)

    # メインモード

    x = 0  # 初期姿勢定義
    y = 60  #
    dx = 0
    dy = 0

    while 1:
        _, frame = capture.read()
        k = cv2.waitKey(10)
        if k == ord("f"):
            cv2.putText(frame, "FIRE", (30, 450),
                        cv2.FONT_HERSHEY_SIMPLEX, 13, (0, 0, 255), thickness=20)
            fire()
            time.sleep(0.5)
        elif k == 27:
            break

        rects = find_rect_of_target_color(frame, 200, 250)
        if len(rects) > 0:
            rect = max(rects, key=(lambda x: x[2] * x[3]))
            obj_centor_x = rect[0]+(rect[2] // 2)
            obj_centor_y = rect[1]+(rect[3] // 2)
            Area = rect[4]
            dist = (29700/float(Area)) * 2
            dx = obj_centor_x - centor_width
            dy = obj_centor_y - centor_height

            object_centor = "x="+str(obj_centor_x)+",y="+str(obj_centor_y)
            difference = "difference: x=" + \
                str(dx)+",y="+str(dy)+" ,distance"+str(dist)

            cv2.circle(frame, (obj_centor_x, obj_centor_y),
                       5, (0, 255, 0), thickness=-1)
            cv2.rectangle(frame, tuple(rect[0:2]), tuple(
                rect[0:2] + rect[2:4]), (0, 0, 255), thickness=2)
            cv2.putText(frame, object_centor, (rect[0], rect[1]-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), thickness=1)
            cv2.putText(frame, difference, (rect[0], rect[1]-30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), thickness=1)
        else:
            cv2.putText(frame, "LOST", (30, 450),
                        cv2.FONT_HERSHEY_SIMPLEX, 13, (0, 0, 255), thickness=20)
        cv2.imshow('position', frame)

        x, y = arg(dx, dy, x, y)

        x_limm = -40
        x_limM = 60
        y_limm = 30
        y_limM = 75

        if x < x_limm:
            x = x_limm
        if x > x_limM:
            x = x_limM
        if y < y_limm:
            y = y_limm
        if y > y_limM:
            y = y_limM

        if (dx > -10 and dx < 10)and(dy > -10 and dy < 10):
            fire()
            time.sleep(1.5)
        #
        robot.set_angles(str(x)+",-35,65,"+str(y)+",45")
        #
        print(str(x)+",30,40,"+str(y)+",45")
        # time.sleep(0.05)
    capture.release()
    cv2.destroyAllWindows()
