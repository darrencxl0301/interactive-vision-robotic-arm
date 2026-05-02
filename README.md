# Interactive Vision-Guided Robotic Arm

A real-time **human-robot interaction** system that connects computer vision with physical manipulation.  
No buttons, joysticks, or handheld controllers — the interface is your hand.

![demo](assets/demo.gif)

![demo2](assets/demo2.gif)

---

## Overview

This system processes live webcam input to detect colored objects and track fingertip gestures in real time.  
A finite-state machine interprets user intent from fingertip dwell time, then issues motor commands to a physical robotic arm over Wi-Fi.  
The vision loop remains live throughout execution, so perception and action run concurrently rather than sequentially.

```text
Webcam
  └─► Vision          HSV-based color segmentation + contour filtering + temporal stability
        └─► Gesture   MediaPipe fingertip tracking + dwell-time intent recognition
              └─► FSM  idle → color_locked → execute → idle
                    └─► Arm  HTTP → ESP8266 → servo control
```

---

## 🛠 Stack

| Layer | Tech |
|---|---|
| Vision | OpenCV — multi-range HSV segmentation, morphological filtering |
| Hand tracking | MediaPipe Hands — fingertip tracking via landmark #8 |
| Control logic | Event-driven finite-state machine + background threading |
| Transport | HTTP over Wi-Fi — `/cmd?<axis> <value>` |
| Firmware | ESP8266 — Arduino |
| Hardware | 6-servo robotic arm (X / Y / Z / B / T / E) |

---

## 🚀 Setup

```bash
# Create environment
conda create -n vision_env python=3.10 -y
conda activate vision_env

# Install dependencies
pip install -r requirements.txt
```

Connect the computer to the ESP8266 Wi-Fi network, then run:

```bash
python main.py
```

---

## ⚙️ Hardware Setup

This project spans the full stack from physical assembly to firmware to vision — not just software.

**1. Mechanical assembly**  
The arm is built from a 6-servo kit (base rotation, shoulder, elbow, wrist, wrist rotation, gripper). Each joint is connected to a dedicated servo horn and secured to the frame. Getting the torque balance right across joints matters for repeatability.

**2. Firmware (Arduino IDE + ESP8266)**  
The ESP8266 runs a lightweight HTTP server written in Arduino C++. On receiving a request like `/cmd?X 110`, it parses the axis and target value, then drives the corresponding servo via PWM. The firmware also handles request decoding, bounds checking per axis, and smooth stepping between positions to avoid servo strain.

**3. Wi-Fi bridge**  
The ESP8266 hosts its own access point. The Python host connects to this network and sends commands over plain HTTP — keeping the communication layer simple and latency low.

**4. Integration**  
Once the arm is calibrated and the firmware is flashed, the Python system treats the arm as a stateless HTTP endpoint. All sequencing, safety logic, and world-state tracking live on the software side.

---

## 🎯 Calibration

Workspace slots such as `bottom`, `top_left`, and `top_right` are defined in the `POSES` dictionary in `main.py` as per-axis servo values.  
These should be tuned to match the physical workspace of the arm before running the system.

---

## 🧩 Design Decisions

**Dwell-time interaction model**  
Intent is inferred from how long the fingertip remains stationary over a region — 1 second to select a color, 2 seconds to confirm a target. No hardware input required.

**Temporal stability filter**  
Raw detections are only surfaced after 0.5s of positional consistency, suppressing flicker and transient false detections without noticeable latency.

**Non-blocking execution**  
Robot motion runs in a background thread. The vision loop never waits on hardware commands, so the camera feed stays responsive throughout.

**Safe trajectory planning**  
Each pick-and-place lifts to a fixed clearance height before lateral movement. Descent is incremental with a small final overpress before release, reducing drop failures and collisions.

**HSV red wraparound**  
Red spans the hue boundary in HSV space. Detection uses two separate hue ranges merged into a single mask to handle this correctly.

---

## 👤 Author

**Darren Chai Xin Lun**  
[ddcxl0301@gmail.com](mailto:ddcxl0301@gmail.com) · [@darrencxl0301](https://github.com/darrencxl0301)

---

## 🔭 Possible Extensions

- Camera-based slot occupancy detection
- Multi-object task planning and sequencing
- Natural language instructions via LLM
