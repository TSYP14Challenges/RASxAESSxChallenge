"""
victim_fire_pi.py - low-latency version of the victim/fire vision stack, built for a Raspberry Pi 4.

WHY IT IS FASTER THAN v2
  1. Capture thread that always keeps only the NEWEST frame. Reading a network/USB stream slower
     than it arrives makes frames queue up = the lag you saw. Here old frames are simply dropped.
  2. The neural network runs in its own thread at a limited rate (DETECT_MIN_INTERVAL_S), so the
     movement/breathing analysis is never blocked by it. Between detections the last boxes are reused.
  3. Nano model, small input (320), NCNN format (the fast CPU format on ARM). No "small" models.
  4. Breathing analysis runs on the small torso crop only (locked while the person is still),
     not on the whole frame. Fire colour analysis runs on a 320-px-wide copy.
  5. Everything heavy runs at 10 Hz or less, whatever the camera frame rate is.
  6. Built-in profiler: the HUD/CSV show detector time, analysis time and end-to-end lag.

ONE-TIME SETUP ON THE PI (64-bit Raspberry Pi OS, 4 GB+ recommended, add a heatsink/fan)
    python3 -m venv venv --system-site-packages && source venv/bin/activate
    pip install torch --index-url https://download.pytorch.org/whl/cpu
    pip install ultralytics ncnn opencv-python-headless numpy
    yolo export model=yolov8n-pose.pt format=ncnn imgsz=320     # creates yolov8n-pose_ncnn_model/
    (optional, later)  yolo export model=fire.pt format=ncnn imgsz=320

RUN
    python victim_fire_pi.py --no-window                       # Pi camera (picamera2), headless
    python victim_fire_pi.py "http://PHONE-IP:4747/video"      # phone stream (also works on a laptop)
    python victim_fire_pi.py clip.mp4                          # video file (uses the video's own clock)
    add --no-window on the Pi (drawing windows costs CPU); add --live to replay a file like a live stream
Quit with q (window) or Ctrl+C.

LIVE PREVIEW OVER THE NETWORK (headless-friendly, no VLC/mjpg-streamer needed)
    python victim_fire_pi.py --no-window --mjpeg               # serves annotated frames at :8090/stream
    Then, from any laptop/phone browser on the same network:  http://<pi-hostname>.local:8090/stream
    Bounding boxes, status labels and the fire overlay are all drawn into the streamed frame -- this
    is the actual CV output, not just the raw camera feed. Uses only the stdlib http.server, no extra
    packages. Optional: --mjpeg-port N to change the port (default 8090).

CAMERA BACKEND
    Source "0" (the default / bare "python victim_fire_pi.py --no-window") now goes through
    picamera2 automatically -- this is the official libcamera-based API for the Pi's CSI camera
    module, and it's what actually delivers frames reliably (cv2.VideoCapture(0) opens the device
    node fine on this OS/camera combo but never returns real frames). Any other source (a URL, a
    video file, or an explicit USB device index like "1") still goes through cv2.VideoCapture as
    before. The venv must be created with --system-site-packages so it can see the system-wide
    python3-picamera2 package (pip installing picamera2 into an isolated venv does not work
    reliably -- it depends on system libcamera bindings).

Same limits as before: the camera must be stationary and the person visible for ~15 s for the
breathing verdict. "NO BREATHING" means unresponsive - a human must check.
"""
import csv
import sys
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import cv2
import numpy as np
from ultralytics import YOLO

# ---------------- config ----------------
USE_POSE = True                      # False = plain person detector (faster, no keypoints)
POSE_NCNN = "yolov8n-pose_ncnn_model" if USE_POSE else "yolov8n_ncnn_model"
POSE_PT = "yolov8n-pose.pt" if USE_POSE else "yolov8n.pt"      # fallback when the NCNN folder is missing
FIRE_NCNN = "fire_ncnn_model"
FIRE_PT = "fire.pt"
IMGSZ = 320                          # network input size (320 is ~4x cheaper than 640)
DETECT_MIN_INTERVAL_S = 0.35         # at most ~3 detections per second
FIRE_MODEL_INTERVAL_S = 1.0          # fire model at most once per second
FIRE_MODEL_IMGSZ = 480               # higher than pose's IMGSZ for tighter boxes (fire runs far less often)
FIRE_MODEL_CONF = 0.35               # a bit stricter than the default 0.25 -- fewer loose/low-confidence boxes
FIRE_MODEL_ONLY_CLASS = "fire"       # drop any other class this model predicts (e.g. "smoke") -- fire only
PERSON_TTL_S = 3.0                   # forget detections older than this
LOG_CSV = "detections_log.csv"

CAP_W, CAP_H, CAP_FPS = 640, 480, 15  # requested from a local camera (ignored for streams/files)
ANALYSIS_DT = 0.1                    # analysis tick: 10 Hz

# gross motion (computed on a DIFF_W-wide copy)
DIFF_W = 160
REF_DELAY_S = 0.5
MOTION_PIX_THR = 10
MOTION_RATIO_THR = 0.004
GROSS_RATIO_MULT = 2.5   # "moving" needs this many times MOTION_RATIO_THR (breathing alone should not trigger it)
NOISE_MULT = 2.5
MOVING_FRAMES = 6        # consecutive analysis ticks above threshold before we call it "moving" (~0.6s)
MOVING_HOLD_S = 3.0
CAMERA_MOVING_THR = 0.25
LYING_ANGLE_DEG = 55

# breathing (optical flow on the locked torso crop)
ROI_MAX_W = 160                      # torso crop is shrunk to at most this width before optical flow
ROI_SHIFT_FRAC = 0.30                # re-lock the torso ROI only if it moved more than this fraction
BG_PATCH = (96, 72)                  # background patch used to cancel camera shake
BREATH_WIN_S = 16
BREATH_MIN_S = 10
BREATH_BAND = (0.1, 0.7)
NOISE_BAND = (0.9, 3.0)
BREATH_SNR_THR = 20.0
BREATH_VOTES = (4, 5)
NO_SIGNAL_S = 20
SAMPLE_DT = 0.1

# fire
FIRE_W = 320                         # heuristic runs on this width
FIRE_HEURISTIC_DT = 0.2              # 5 Hz
USE_FIRE_HEURISTIC = True
FIRE_MIN_AREA = 0.0015
FIRE_CONFIRM_HEUR = (3, 6)           # 3 hits in the last 6 heuristic runs
FIRE_CONFIRM_MODEL = (2, 3)          # 2 hits in the last 3 model runs
FIRE_NEAR_FRAC = 0.10

EVENT_KEYS = {"ALIVE", "LIKELY", "NO", "STILL"}


# ---------------- capture: always the newest frame ----------------
class PiCameraSource:
    """Drop-in replacement for FrameSource, backed by picamera2 (CSI camera via libcamera).

    Same public surface as FrameSource: .opened, .read() -> (ok, frame_bgr, t), .close().
    Runs a background thread that always keeps only the newest captured frame, exactly like
    FrameSource's live mode, so old frames never queue up.
    """

    def __init__(self, width=CAP_W, height=CAP_H, fps=CAP_FPS):
        from picamera2 import Picamera2  # local import: only needed on this path

        self.live = True
        self.is_file = False
        self.opened = False
        try:
            self.picam2 = Picamera2()
            config = self.picam2.create_video_configuration(
                main={"size": (width, height), "format": "RGB888"},
                controls={"FrameRate": fps},
            )
            self.picam2.configure(config)
            self.picam2.start()
            self.opened = True
        except Exception as e:
            print(f"picamera2 failed to start: {e}")
            return

        self.cond = threading.Condition()
        self.frame, self.t = None, 0.0
        self.count = self.seen = 0
        self.eof = self.stop = False
        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self):
        while not self.stop:
            try:
                # RGB888 from picamera2 is actually laid out as BGR for our purposes once
                # captured this way in practice varies by version, so convert explicitly.
                arr = self.picam2.capture_array("main")
            except Exception:
                with self.cond:
                    self.eof = True
                    self.cond.notify_all()
                return
            # Note: picamera2's "RGB888" format is actually delivered in BGR byte order
            # despite the name -- it's already what OpenCV wants, so no conversion here.
            # (Converting it, as an earlier version of this file did, swaps R/B and turns
            # skin tones blue.) Just drop a possible alpha channel if present.
            frame_bgr = arr[:, :, :3] if arr.shape[2] == 4 else arr
            with self.cond:
                self.frame, self.t = frame_bgr, time.time()
                self.count += 1
                self.cond.notify_all()

    def read(self):
        with self.cond:
            while self.count == self.seen and not self.eof and not self.stop:
                self.cond.wait(timeout=1.0)
            if self.count == self.seen:
                return False, None, 0.0
            self.seen = self.count
            return True, self.frame, self.t

    def close(self):
        if self.opened:
            self.stop = True
            time.sleep(0.05)
            try:
                self.picam2.stop()
            except Exception:
                pass


class FrameSource:
    def __init__(self, source, live):
        self.is_file = isinstance(source, str) and not source.lower().startswith(("http://", "https://", "rtsp://"))
        self.live = live
        self.cap = cv2.VideoCapture(source)
        self.opened = self.cap.isOpened()
        if not self.opened:
            return
        if isinstance(source, int):
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAP_W)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAP_H)
            self.cap.set(cv2.CAP_PROP_FPS, CAP_FPS)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.cond = threading.Condition()
        self.frame, self.t = None, 0.0
        self.count = self.seen = 0
        self.eof = self.stop = False
        if live:
            threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self):
        fps = self.cap.get(cv2.CAP_PROP_FPS) or 30.0
        period = 1.0 / fps if self.is_file else 0.0       # pace a replayed file like a real camera
        while not self.stop:
            t0 = time.time()
            ok, f = self.cap.read()
            if not ok:
                with self.cond:
                    self.eof = True
                    self.cond.notify_all()
                return
            with self.cond:
                self.frame, self.t = f, time.time()
                self.count += 1
                self.cond.notify_all()
            if period:
                time.sleep(max(0.0, period - (time.time() - t0)))

    def read(self):
        """Returns (ok, frame, timestamp_seconds)."""
        if not self.live:
            ok, f = self.cap.read()
            return (True, f, self.cap.get(cv2.CAP_PROP_POS_MSEC) / 1000.0) if ok else (False, None, 0.0)
        with self.cond:
            while self.count == self.seen and not self.eof and not self.stop:
                self.cond.wait(timeout=1.0)
            if self.count == self.seen:
                return False, None, 0.0
            self.seen = self.count
            return True, self.frame, self.t

    def close(self):
        if self.opened:
            self.stop = True
            time.sleep(0.05)
            self.cap.release()


# ---------------- neural network worker ----------------
def load_model(ncnn_dir, pt_name, task):
    if Path(ncnn_dir).exists():
        print(f"Loading NCNN model {ncnn_dir}")
        return YOLO(ncnn_dir, task=task)
    print(f"{ncnn_dir} not found -> using {pt_name} (much slower on a Pi: export to NCNN, see header)")
    return YOLO(pt_name, task=task)


class Hysteresis:
    def __init__(self, need, window):
        self.need, self.h, self.last = need, deque(maxlen=window), []

    def update(self, boxes):
        self.h.append(bool(boxes))
        if boxes:
            self.last = list(boxes)
        return list(self.last) if sum(self.h) >= self.need else []


class Detector:
    """Runs person (pose) detection and, optionally, a fire model on the newest frame."""

    def __init__(self, threaded):
        self.pose = load_model(POSE_NCNN, POSE_PT, "pose" if USE_POSE else "detect")
        self.fire_model = None
        if Path(FIRE_NCNN).exists():
            self.fire_model = YOLO(FIRE_NCNN, task="detect")
        elif Path(FIRE_PT).exists():
            self.fire_model = YOLO(FIRE_PT)
        self.fire_hyst = Hysteresis(*FIRE_CONFIRM_MODEL)
        self.threaded = threaded
        self.lock = threading.Lock()
        self.pending = None
        self.persons, self.persons_t = [], -1e9
        self.model_fire = []
        self.last_run = self.last_fire_run = -1e9
        self.infer_ms = 0.0
        self.stop = False
        if threaded:
            threading.Thread(target=self._loop, daemon=True).start()

    def submit(self, frame, t):
        if self.threaded:
            with self.lock:
                self.pending = (t, frame)
        elif t - self.last_run >= DETECT_MIN_INTERVAL_S:
            self._infer(frame, t)

    def _loop(self):
        last = 0.0
        while not self.stop:
            wait = DETECT_MIN_INTERVAL_S - (time.time() - last)
            if wait > 0:
                time.sleep(wait)
            with self.lock:
                item, self.pending = self.pending, None
            if item is None:
                time.sleep(0.01)
                continue
            last = time.time()
            self._infer(item[1], item[0])

    def _infer(self, frame, t):
        t0 = time.time()
        r = self.pose.track(frame, persist=True, classes=[0], imgsz=IMGSZ, conf=0.3, verbose=False)[0]
        persons = []
        if r.boxes is not None and r.boxes.id is not None:
            ids = r.boxes.id.int().tolist()
            boxes = r.boxes.xyxy.int().tolist()
            kxy = kcf = None
            if r.keypoints is not None:
                kxy = r.keypoints.xy.cpu().numpy()
                kcf = r.keypoints.conf.cpu().numpy() if r.keypoints.conf is not None else None
            for i, tid in enumerate(ids):
                persons.append((tid, boxes[i],
                                None if kxy is None else kxy[i],
                                None if kcf is None else kcf[i]))
        fire = None
        if self.fire_model is not None and t - self.last_fire_run >= FIRE_MODEL_INTERVAL_S:
            self.last_fire_run = t
            fr = self.fire_model.predict(frame, imgsz=FIRE_MODEL_IMGSZ, conf=FIRE_MODEL_CONF, verbose=False)[0]
            boxes_f = []
            for b in fr.boxes:
                label = fr.names[int(b.cls[0])]
                if FIRE_MODEL_ONLY_CLASS is not None and label != FIRE_MODEL_ONLY_CLASS:
                    continue  # e.g. drop "smoke" -- only report actual fire
                x1, y1, x2, y2 = map(int, b.xyxy[0].tolist())
                boxes_f.append((x1, y1, x2, y2, label))
            fire = self.fire_hyst.update(boxes_f)
        with self.lock:
            self.persons, self.persons_t = persons, t
            if fire is not None:
                self.model_fire = fire
            self.last_run = t
            self.infer_ms = 0.8 * self.infer_ms + 0.2 * (time.time() - t0) * 1000 if self.infer_ms else (time.time() - t0) * 1000

    def latest(self):
        with self.lock:
            return self.persons, self.persons_t, self.model_fire, self.infer_ms


# ---------------- analysis helpers ----------------
def is_lying(xy, conf, box):
    if xy is not None and conf is not None and np.all(conf[[5, 6, 11, 12]] > 0.4):
        v = xy[[11, 12]].mean(axis=0) - xy[[5, 6]].mean(axis=0)
        return np.degrees(np.arctan2(abs(v[0]), abs(v[1]) + 1e-6)) > LYING_ANGLE_DEG
    x1, y1, x2, y2 = box
    return (x2 - x1) > 1.2 * (y2 - y1)


def torso_roi(xy, conf, box, size):
    """Torso rectangle in full-frame pixels (keypoints if visible, else the central half of the box)."""
    W, H = size
    if xy is not None and conf is not None and np.all(conf[[5, 6, 11, 12]] > 0.4):
        pts = xy[[5, 6, 11, 12]]
        x1, y1 = pts.min(axis=0)
        x2, y2 = pts.max(axis=0)
        px, py = 0.15 * max(x2 - x1, 10), 0.15 * max(y2 - y1, 10)
        x1, y1, x2, y2 = x1 - px, y1 - py, x2 + px, y2 + py
    else:
        bx1, by1, bx2, by2 = [float(v) for v in box]
        cx, cy, w, h = (bx1 + bx2) / 2, (by1 + by2) / 2, (bx2 - bx1) / 2, (by2 - by1) / 2
        x1, y1, x2, y2 = cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2
    x1, y1 = int(np.clip(x1, 0, W - 2)), int(np.clip(y1, 0, H - 2))
    x2, y2 = int(np.clip(x2, 0, W)), int(np.clip(y2, 0, H))
    x2, y2 = min(W, max(x2, x1 + 24)), min(H, max(y2, y1 + 24))
    return x1, y1, x2, y2


def roi_moved(a, b):
    aw, ah = max(1, a[2] - a[0]), max(1, a[3] - a[1])
    dx = abs((a[0] + a[2] - b[0] - b[2]) / 2.0)
    dy = abs((a[1] + a[3] - b[1] - b[3]) / 2.0)
    dw, dh = abs(aw - (b[2] - b[0])), abs(ah - (b[3] - b[1]))
    return dx > ROI_SHIFT_FRAC * aw or dy > ROI_SHIFT_FRAC * ah or dw > 0.5 * aw or dh > 0.5 * ah


def crop_flow_y(ref_gray, gray, rect, max_w):
    """Mean vertical optical flow inside rect, in full-resolution pixels."""
    x1, y1, x2, y2 = rect
    a, b = ref_gray[y1:y2, x1:x2], gray[y1:y2, x1:x2]
    k = 1.0
    if a.shape[1] > max_w:
        k = max_w / a.shape[1]
        a = cv2.resize(a, None, fx=k, fy=k, interpolation=cv2.INTER_AREA)
        b = cv2.resize(b, None, fx=k, fy=k, interpolation=cv2.INTER_AREA)
    a, b = cv2.GaussianBlur(a, (5, 5), 0), cv2.GaussianBlur(b, (5, 5), 0)
    flow = cv2.calcOpticalFlowFarneback(a, b, None, 0.5, 2, 11, 2, 5, 1.1, 0)
    return float(flow[..., 1].mean()) / k


def box_gap(a, b):
    dx = max(b[0] - a[2], a[0] - b[2], 0)
    dy = max(b[1] - a[3], a[1] - b[3], 0)
    return (dx * dx + dy * dy) ** 0.5


def pick_bg_patch(person_boxes, W, H):
    pw, ph = BG_PATCH
    if W < 3 * pw or H < 3 * ph:
        return None
    for x, y in ((0, 0), (W - pw, 0), (0, H - ph), (W - pw, H - ph)):
        r = (x, y, x + pw, y + ph)
        if all(box_gap(r, b) > 10 for b in person_boxes):
            return r
    return None


def breathing_snr(buf):
    if len(buf) < 20:
        return None
    ts = np.array([b[0] for b in buf])
    vs = np.array([b[1] for b in buf])
    if ts[-1] - ts[0] < BREATH_MIN_S:
        return None
    fs = 1.0 / SAMPLE_DT
    tg = np.arange(max(ts[0], ts[-1] - BREATH_WIN_S), ts[-1], 1.0 / fs)
    x = np.interp(tg, ts, vs)
    tt = tg - tg[0]
    x = x - np.polyval(np.polyfit(tt, x, 1), tt)
    x = x * np.hanning(len(x))
    n = 4 * len(x)
    P = np.abs(np.fft.rfft(x, n)) ** 2
    f = np.fft.rfftfreq(n, 1.0 / fs)
    band = (f >= BREATH_BAND[0]) & (f <= BREATH_BAND[1])
    ref = (f >= NOISE_BAND[0]) & (f <= NOISE_BAND[1])
    k = P[band].argmax()
    return float(P[band][k] / (np.median(P[ref]) + 1e-12)), float(f[band][k])


class PersonState:
    def __init__(self):
        self.still_since = None
        self.moving_count = 0
        self.last_move = -1e9
        self.last_seen = 0.0
        self.roi = None
        self.buf = deque(maxlen=400)
        self.votes = deque(maxlen=BREATH_VOTES[1])
        self.next_analysis = 0.0
        self.snr, self.bpm = 0.0, None
        self.lying = False
        self.status = "OBSERVING"
        self.near_fire = False

    def reset_breathing(self):
        self.buf.clear()
        self.votes.clear()
        self.snr, self.bpm, self.next_analysis = 0.0, None, 0.0


def fire_heuristic(frame, state):
    """Cheap colour + flicker fire detector on a FIRE_W-wide copy. Returns boxes in full-frame pixels."""
    H, W = frame.shape[:2]
    k = FIRE_W / W
    small = cv2.resize(frame, (FIRE_W, max(2, int(H * k))), interpolation=cv2.INTER_AREA)
    hsv = cv2.cvtColor(small, cv2.COLOR_BGR2HSV)
    Y, Cr, Cb = [c.astype(np.int16) for c in cv2.split(cv2.cvtColor(small, cv2.COLOR_BGR2YCrCb))]
    warm = cv2.inRange(hsv, (0, 130, 170), (35, 255, 255))
    chroma = ((Cr > 168) & ((Cr - Cb) > 100) & (Y > 100)).astype(np.uint8) * 255
    flame = cv2.bitwise_and(warm, chroma)
    core = cv2.inRange(hsv, (0, 0, 235), (179, 90, 255))
    near = cv2.dilate(flame, np.ones((7, 7), np.uint8))
    mask = cv2.bitwise_or(flame, cv2.bitwise_and(core, near))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    area = mask.mean() / 255.0
    flicker = 0.0 if state.get("prev") is None else cv2.absdiff(mask, state["prev"]).mean() / 255.0
    state["prev"] = mask
    if not (area > FIRE_MIN_AREA and flicker > 0.0003):
        return []
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return []
    x, y, w, h = cv2.boundingRect(max(contours, key=cv2.contourArea))
    return [(int(x / k), int(y / k), int((x + w) / k), int((y + h) / k), "fire?")]


# ---------------- optional MJPEG server (headless live preview) ----------------
class MjpegServer:
    """Serves the latest annotated frame at http://<host>:<port>/stream as multipart/x-mixed-replace.
    Zero extra dependencies -- just stdlib http.server + cv2.imencode. Call update(frame) every tick;
    any number of browsers/VLC/etc can connect to /stream at once and each gets the newest frame.
    """

    def __init__(self, port=8090, quality=80):
        self.port = port
        self.quality = quality
        self.lock = threading.Lock()
        self.jpg = None
        outer = self

        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                if self.path != "/stream":
                    self.send_response(404)
                    self.end_headers()
                    return
                self.send_response(200)
                self.send_header("Age", "0")
                self.send_header("Cache-Control", "no-cache, private")
                self.send_header("Pragma", "no-cache")
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=FRAME")
                self.end_headers()
                try:
                    while True:
                        with outer.lock:
                            jpg = outer.jpg
                        if jpg is None:
                            time.sleep(0.05)
                            continue
                        self.wfile.write(b"--FRAME\r\n")
                        self.send_header("Content-Type", "image/jpeg")
                        self.send_header("Content-Length", str(len(jpg)))
                        self.end_headers()
                        self.wfile.write(jpg)
                        self.wfile.write(b"\r\n")
                        time.sleep(0.03)
                except (BrokenPipeError, ConnectionResetError):
                    pass

            def log_message(self, fmt, *args):
                pass  # silence per-request logging

        self.server = ThreadingHTTPServer(("0.0.0.0", port), Handler)
        threading.Thread(target=self.server.serve_forever, daemon=True).start()
        print(f"MJPEG preview: http://<this-pi-hostname>.local:{port}/stream")

    def update(self, frame):
        ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, self.quality])
        if ok:
            with self.lock:
                self.jpg = buf.tobytes()

    def close(self):
        self.server.shutdown()


# ---------------- main loop ----------------
def main():
    raw = sys.argv[1:]
    mjpeg_port = 8090
    if "--mjpeg-port" in raw:
        i = raw.index("--mjpeg-port")
        if i + 1 < len(raw):
            mjpeg_port = int(raw[i + 1])
            raw = raw[:i] + raw[i + 2:]   # drop the flag and its value before positional parsing
    show = "--no-window" not in raw
    mjpeg = "--mjpeg" in raw
    args = [a for a in raw if not a.startswith("--")]
    arg = args[0] if args else "0"
    source = int(arg) if arg.isdigit() else arg
    is_file = isinstance(source, str) and not source.lower().startswith(("http://", "https://", "rtsp://"))
    live = (not is_file) or ("--live" in sys.argv)

    # Local Pi camera (default source "0") goes through picamera2; anything else
    # (URL, file, or an explicit non-zero device index) keeps using cv2.VideoCapture.
    if source == 0:
        print("Using picamera2 for local camera capture")
        src = PiCameraSource(CAP_W, CAP_H, CAP_FPS)
    else:
        src = FrameSource(source, live)
    if not src.opened:
        sys.exit("Cannot open video source")
    det = Detector(threaded=live)
    fire_h = Hysteresis(*FIRE_CONFIRM_HEUR)
    fire_state = {}
    heur_boxes, last_heur = [], -1e9

    mjpeg_srv = MjpegServer(port=mjpeg_port) if mjpeg else None
    draw = show or mjpeg   # need the annotated frame if either a local window or the MJPEG stream wants it

    people, hist = {}, deque()
    last_tick, last_sample, last_log = -1e9, -1e9, -1e9
    fire_reported = False
    tick_ms = 0.0

    logf = open(LOG_CSV, "w", newline="")
    log = csv.writer(logf)
    log.writerow(["time_s", "id", "status", "lying", "motion_ratio", "motion_thr", "breath_snr",
                  "breath_bpm", "near_fire", "fire_detected", "lag_ms", "det_ms", "tick_ms"])

    try:
        while True:
            ok, frame, now = src.read()
            if not ok:
                break
            det.submit(frame, now)
            if now - last_tick < ANALYSIS_DT:
                continue
            last_tick = now
            t_start = time.time()
            lag_ms = (t_start - now) * 1000.0 if live else 0.0
            H, W = frame.shape[:2]

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            sc = DIFF_W / W
            small = cv2.resize(gray, (DIFF_W, max(2, int(H * sc))), interpolation=cv2.INTER_AREA)
            sh, sw = small.shape
            hist.append((now, gray, small))
            while len(hist) > 1 and hist[1][0] <= now - REF_DELAY_S:
                hist.popleft()
            _, ref_gray, ref_small = hist[0]
            d = np.abs((small.astype(np.float32) - small.mean()) - (ref_small.astype(np.float32) - ref_small.mean()))
            diff = d > MOTION_PIX_THR
            camera_moving = float(diff.mean()) > CAMERA_MOVING_THR

            persons, p_t, model_fire, det_ms = det.latest()
            if now - p_t > PERSON_TTL_S:
                persons = []

            # fire: heuristic at 5 Hz, or the model's (already hysteresis-filtered) boxes
            if det.fire_model is not None:
                fire_boxes = model_fire
            elif USE_FIRE_HEURISTIC:
                if now - last_heur >= FIRE_HEURISTIC_DT:
                    last_heur = now
                    heur_boxes = fire_h.update(fire_heuristic(frame, fire_state))
                fire_boxes = heur_boxes
            else:
                fire_boxes = []

            # background = outside person boxes: noise estimate for the motion threshold
            bg = np.ones((sh, sw), bool)
            for _, box, _, _ in persons:
                x1, y1, x2, y2 = np.clip((np.array(box) * sc).astype(int), 0, [sw, sh, sw, sh])
                bg[y1:y2, x1:x2] = False
            bg_ok = bg.sum() > 0.1 * bg.size
            bg_ratio = float(diff[bg].mean()) if bg_ok else 0.0
            motion_thr = max(MOTION_RATIO_THR, NOISE_MULT * bg_ratio)

            # camera-shake reference: vertical flow of a person-free background patch
            do_sample = bool(persons) and not camera_moving and (now - last_sample) >= SAMPLE_DT
            gmed = 0.0
            if do_sample:
                last_sample = now
                patch = pick_bg_patch([p[1] for p in persons], W, H)
                if patch is not None:
                    gmed = crop_flow_y(ref_gray, gray, patch, ROI_MAX_W)

            for tid, box, xy, cf in persons:
                x1, y1, x2, y2 = [int(v) for v in box]
                x1, y1, x2, y2 = max(0, x1), max(0, y1), min(W, x2), min(H, y2)
                if x2 - x1 < 8 or y2 - y1 < 8:
                    continue
                st = people.setdefault(tid, PersonState())
                st.last_seen = now
                st.lying = is_lying(xy, cf, (x1, y1, x2, y2))

                sx1, sy1, sx2, sy2 = np.clip((np.array([x1, y1, x2, y2]) * sc).astype(int), 0, [sw, sh, sw, sh])
                roi_d = diff[sy1:sy2, sx1:sx2]
                ratio = float(roi_d.mean()) if roi_d.size else 0.0
                st.moving_count = st.moving_count + 1 if ratio > GROSS_RATIO_MULT * motion_thr else 0
                if st.moving_count >= MOVING_FRAMES:
                    st.last_move = now

                if camera_moving:
                    st.still_since = None
                    st.reset_breathing()
                    status = "CAMERA MOVING"
                elif now - st.last_move < MOVING_HOLD_S:
                    st.still_since = None
                    st.reset_breathing()
                    status = "ALIVE (moving)"
                else:
                    if st.still_since is None:
                        st.still_since = now
                    still_for = now - st.still_since

                    new_roi = torso_roi(xy, cf, (x1, y1, x2, y2), (W, H))
                    if st.roi is None or roi_moved(st.roi, new_roi):
                        st.roi = new_roi                     # (re)lock the torso crop
                        st.reset_breathing()
                    if do_sample:
                        st.buf.append((now, crop_flow_y(ref_gray, gray, st.roi, ROI_MAX_W) - gmed))

                    if still_for >= BREATH_MIN_S and now >= st.next_analysis:
                        st.next_analysis = now + 1.0
                        r = breathing_snr(st.buf)
                        if r is not None:
                            st.snr, hz = r
                            st.votes.append(st.snr > BREATH_SNR_THR)
                            st.bpm = hz * 60.0

                    breathing = len(st.votes) == BREATH_VOTES[1] and sum(st.votes) >= BREATH_VOTES[0]
                    if breathing:
                        status = f"LIKELY ALIVE (breathing ~{st.bpm:.0f}/min)"
                    elif still_for < BREATH_MIN_S:
                        status = f"OBSERVING {still_for:.0f}s"
                    elif still_for < NO_SIGNAL_S:
                        status = f"ANALYSING {still_for:.0f}s"
                    else:
                        status = "NO MOVEMENT / NO BREATHING (body?)" if st.lying else "STILL (upright)"

                key = status.split()[0]
                if key != st.status.split()[0] and key in EVENT_KEYS:
                    print(f"[EVENT] t={now:7.1f}s person#{tid}: {status}  lying={st.lying}", flush=True)
                st.status = status

                near = any(box_gap((x1, y1, x2, y2), fb[:4]) < FIRE_NEAR_FRAC * W for fb in fire_boxes)
                if near and not st.near_fire:
                    print(f"[EVENT] t={now:7.1f}s person#{tid}: NEAR FIRE", flush=True)
                st.near_fire = near

                if draw:
                    color = (0, 200, 0) if key in ("ALIVE", "LIKELY") else ((0, 0, 255) if key == "NO" else (0, 165, 255))
                    cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                    cv2.putText(frame, f"#{tid} {status}" + (" | NEAR FIRE" if near else ""),
                                (x1, max(15, y1 - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                    cv2.putText(frame, f"{'lying' if st.lying else 'upright'} motion={ratio:.3f} snr={st.snr:.0f}",
                                (x1, min(H - 5, y2 + 16)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)

                if now - last_log >= 1.0:
                    log.writerow([f"{now:.2f}", tid, status, int(st.lying), f"{ratio:.4f}", f"{motion_thr:.4f}",
                                  f"{st.snr:.1f}", "" if st.bpm is None else f"{st.bpm:.0f}", int(near),
                                  int(bool(fire_boxes)), f"{lag_ms:.0f}", f"{det_ms:.0f}", f"{tick_ms:.0f}"])
            if now - last_log >= 1.0:
                last_log = now
                logf.flush()

            for tid in [t for t, s in people.items() if now - s.last_seen > 10.0]:
                del people[tid]

            if fire_boxes and not fire_reported:
                print(f"[EVENT] t={now:7.1f}s FIRE detected", flush=True)
            fire_reported = bool(fire_boxes)

            tick_ms = 0.9 * tick_ms + 0.1 * (time.time() - t_start) * 1000.0 if tick_ms else (time.time() - t_start) * 1000.0

            if draw:
                for x1, y1, x2, y2, label in fire_boxes:
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
                    cv2.putText(frame, label, (x1, max(15, y1 - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                cv2.putText(frame, f"lag {lag_ms:.0f} ms | det {det_ms:.0f} ms | tick {tick_ms:.0f} ms",
                            (8, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                if mjpeg_srv is not None:
                    mjpeg_srv.update(frame)
                if show:
                    cv2.imshow("victim + fire (Pi)", frame)
                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        break
    except KeyboardInterrupt:
        pass
    finally:
        det.stop = True
        src.close()
        logf.close()
        if mjpeg_srv is not None:
            mjpeg_srv.close()
        if show:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()