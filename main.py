import cv2
import mediapipe as mp
import numpy as np
import time
import requests


import threading

robot_busy = False

URL = "http://192.168.1.1/cmd?"

def send(cmd):
    import urllib.parse
    encoded = urllib.parse.quote(cmd)

    try:
        requests.get(URL + encoded, timeout=0.2)
    except:
        pass

    time.sleep(0.12) 

def move_axis(axis, cur, tgt):
    while abs(cur - tgt) > 10:
        cur = cur + 10 if cur < tgt else cur - 10
        send(f"{axis} {int(cur)}")
        time.sleep(0.1)

    while abs(cur - tgt) > 0:
        cur = cur + 1 if cur < tgt else cur - 1
        send(f"{axis} {int(cur)}")
        time.sleep(0.1)

    return tgt

def smooth_move(current, target):
    for axis in ["Z","B","Y","X"]:
        current[axis] = move_axis(axis, current[axis], target[axis])
    return current

def grip(open=True):
    send("E 60" if open else "E 82")
    time.sleep(0.3)

    
def is_close(current, target, tol=3):
    for axis in ["Z", "B", "X", "Y"]:
        if abs(current[axis] - target[axis]) > tol:
            return False
    return True

# ================== 标定 ==================
POSES = {
    "bottom": {"Z":116,"B":90,"Y":160,"X":102},
    "bottom_left": {"Z":95,"B":96,"Y":176,"X":86},
    "bottom_right": {"Z":86,"B":98,"Y":180,"X":120},
    "top": {"Z":150,"B":136,"Y":116,"X":102},
    "top_left": {"Z":140,"B":134,"Y":130,"X":74},
    "top_right": {"Z":120,"B":148,"Y":140,"X":132},
}

current_pose = POSES["bottom"].copy()

slots = {
    "bottom": None,
    "bottom_left": "red",
    "bottom_right": None,
    "top": None,
    "top_left": "blue",
    "top_right": None,
}


def run_move(src, dst):
    global robot_busy

    robot_busy = True
    move_block(src, dst)
    robot_busy = False
    

def move_block(src, dst):
    global current_pose

    SAFE_Y = 110 

    grip(True)

    current_pose = smooth_move(current_pose, {
        "Z": POSES[src]["Z"],
        "B": POSES[src]["B"],
        "X": POSES[src]["X"],
        "Y": SAFE_Y
    })

    current_pose["Y"] = move_axis("Y", current_pose["Y"], POSES[src]["Y"])

    grip(False)

    current_pose["Y"] = move_axis("Y", current_pose["Y"], SAFE_Y)

    current_pose = smooth_move(current_pose, {
        "Z": POSES[dst]["Z"],
        "B": POSES[dst]["B"],
        "X": POSES[dst]["X"],
        "Y": SAFE_Y
    })

    while not is_close(current_pose, POSES[dst], tol=3):
        current_pose = smooth_move(current_pose, POSES[dst])

    target_Y = POSES[dst]["Y"]

    while current_pose["Y"] < target_Y:
        current_pose["Y"] += 1
        send(f"Y {int(current_pose['Y'])}")
        time.sleep(0.1)

    send(f"Y {target_Y + 2}") 
    time.sleep(0.2)
    grip(True)

    current_pose["Y"] = move_axis("Y", current_pose["Y"], SAFE_Y)

# ================== Vision ==================
cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)

object_history = {}
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(max_num_hands=1)
mp_draw = mp.solutions.drawing_utils

COLOR_RANGES = {
    "red": [
        (np.array([0, 100, 40]), np.array([10, 255, 200])),
        (np.array([170, 100, 40]), np.array([180, 255, 200]))
    ],
    "blue": [
        (np.array([90, 60, 40]), np.array([130, 255, 200]))
    ],
    "yellow": [
        (np.array([15, 50, 50]), np.array([40, 255, 255]))
    ]
}


def filter_stable_objects(objects):
    global object_history

    stable_objects = []
    now = time.time()

    new_history = {}

    for obj in objects:
        x, y = obj["pos"]
        key = (obj["color"], int(x/20), int(y/20)) 

        if key in object_history:
            start_time = object_history[key]
        else:
            start_time = now

        new_history[key] = start_time

        # ⭐ 超过0.5秒才算稳定
        if now - start_time > 0.5:
            stable_objects.append(obj)

    object_history = new_history
    return stable_objects


def detect_objects(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    objects = []

    for color, ranges in COLOR_RANGES.items():
        mask_total = None
        for lower, upper in ranges:
            mask = cv2.inRange(hsv, lower, upper)
            mask_total = mask if mask_total is None else mask_total + mask

        kernel = np.ones((5,5), np.uint8)
        mask_total = cv2.morphologyEx(mask_total, cv2.MORPH_OPEN, kernel)

        contours, _ = cv2.findContours(mask_total, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            if 300 < cv2.contourArea(cnt) < 3000:
                x,y,w,h = cv2.boundingRect(cnt)
                cx,cy = x+w//2, y+h//2
                objects.append({
                    "color": color,
                    "pos": (cx,cy),
                    "box": (x,y,w,h)
                })

    return objects

# ================== Slot ==================
def get_slot_positions(w, h):
    cx, cy = w//2, h//2
    dx = w//6
    dy = h//6

    return {
        "bottom": (cx, cy),
        "bottom_left": (cx-dx, cy),
        "bottom_right": (cx+dx, cy),
        "top": (cx, cy-dy),
        "top_left": (cx-dx, cy-dy),
        "top_right": (cx+dx, cy-dy),
    }

def get_slot(x, y, pts):
    best, dmin = None, 9999
    for k,(sx,sy) in pts.items():
        d = (x-sx)**2 + (y-sy)**2
        if d < dmin:
            dmin, best = d, k
    return best

# ================== UI ==================
def draw_points(frame, pts):
    overlay = frame.copy()
    for (x,y) in pts.values():
        cv2.circle(overlay, (x,y), 22, (180,180,180), 2) 
    return cv2.addWeighted(overlay, 0.3, frame, 0.7, 0)

def draw_info(frame, color, slot):
    cv2.putText(frame, f"Color: {color if color else '-'}", (10,30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255,255,255), 2)
    cv2.putText(frame, f"Position: {slot if slot else '-'}", (10,60),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255,255,255), 2)

# ================== status ==================
state = "idle"
selected_color = None
stable_start = None
last_pos = None

# ================== main loop ==================

while True:
    ret, frame = cap.read()
    if not ret:
        break

    h,w,_ = frame.shape

    raw_frame = frame.copy()
    raw_objects = detect_objects(raw_frame)
    objects = filter_stable_objects(raw_objects)


    pts = get_slot_positions(w,h)
    frame = draw_points(frame, pts)

    for obj in objects:
        x,y,w1,h1 = obj["box"]
        cv2.rectangle(frame, (x,y), (x+w1,y+h1), (0,255,0), 2)
        cv2.putText(frame, obj["color"], (x,y-10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)

    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    result = hands.process(rgb)

    finger = None

    if result.multi_hand_landmarks:
        for hand in result.multi_hand_landmarks:
            mp_draw.draw_landmarks(frame, hand, mp_hands.HAND_CONNECTIONS)
            cx = int(hand.landmark[8].x * w)
            cy = int(hand.landmark[8].y * h)
            finger = (cx,cy)
            cv2.circle(frame, finger, 10, (255,0,0), -1)

    # ===== reset =====
    if not result.multi_hand_landmarks:
        state = "idle"
        selected_color = None
        stable_start = None
        last_pos = None

    stable = False
    if finger:
        if last_pos:
            dist = np.linalg.norm(np.array(finger)-np.array(last_pos))
            if dist < 12:
                if stable_start is None:
                    stable_start = time.time()
                elif time.time()-stable_start > 1:
                    stable = True
            else:
                stable_start = None
        last_pos = finger

    current_slot = None
    if finger:
        current_slot = get_slot(finger[0], finger[1], pts)

    if finger:

        if state == "idle" and stable:
            for obj in objects:
                if np.linalg.norm(np.array(finger)-np.array(obj["pos"])) < 50:
                    selected_color = obj["color"]
                    print("Color:", selected_color)
                    state = "color_locked"
                    stable_start = None
                    break


        elif state == "color_locked":

            if last_pos is not None:
                dist = np.linalg.norm(np.array(finger)-np.array(last_pos))

                if dist < 12:
                    if stable_start is None:
                        stable_start = time.time()
                    elif time.time() - stable_start > 2:

                        target = current_slot
                        print("Execute:", selected_color, "->", target)

                        src = None
                        for k,v in slots.items():
                            if v == selected_color:
                                src = k

                        if src:
                            if slots[target] is not None:
                                temp = "bottom_left" if target != "bottom_left" else "bottom_right"
                                if not robot_busy:
                                    threading.Thread(target=run_move, args=(src, target)).start()
                                slots[temp] = slots[target]

                            if not robot_busy:
                                threading.Thread(target=run_move, args=(src, target)).start()

                            slots[target] = selected_color
                            slots[src] = None

                        state = "idle"
                        selected_color = None
                        stable_start = None

                else:
                    stable_start = None

            last_pos = finger

    draw_info(frame, selected_color, current_slot)

    cv2.imshow("FINAL SYSTEM", frame)
    if cv2.waitKey(1) & 0xFF == 27:
        break

cap.release()
cv2.destroyAllWindows()
