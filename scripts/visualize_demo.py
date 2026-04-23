#!/usr/bin/python3
"""
Prism A/B Web Dashboard.
Two-container architecture: tracks legacy_container and accel_container CPU independently.
"""

import glob, json, logging, os, shutil, signal, subprocess, threading, time, warnings
import cv2, numpy as np, psutil, rclpy
from flask import Flask, Response
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.time import Time
from sensor_msgs.msg import Image

PORT = 8080
JQ = [cv2.IMWRITE_JPEG_QUALITY, 78]
SW, SH = 640, 480
HTML_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'dashboard.html')


def detect_hardware():
    if glob.glob('/dev/nvhost-*') or os.path.exists('/dev/nvmap'):
        return 'nvidia-jetson'
    if glob.glob('/dev/dri/renderD*') and shutil.which('gst-inspect-1.0'):
        try:
            out = subprocess.run(
                ['gst-inspect-1.0', 'vapostproc'],
                capture_output=True, timeout=2,
            )
            if out.returncode == 0:
                return 'intel-vaapi'
        except Exception:
            pass
    return 'cpu'


class DashNode(Node):
    def __init__(self):
        super().__init__('visualize_demo')
        # RELIABLE to match publishers (image_proc and prism_image_proc default).
        # depth=2 keeps the dashboard snappy without queueing old frames.
        qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(Image, '/legacy/image_processed', self._on_l, qos)
        self.create_subscription(Image, '/accelerated/image_processed', self._on_a, qos)

        self.lock = threading.Lock()
        blk = self._black()
        self.l_jpg = blk; self.a_jpg = blk
        self.l_evt = threading.Event(); self.a_evt = threading.Event()
        self.l_lat = 0.0; self.a_lat = 0.0
        self.l_fps = 0.0; self.a_fps = 0.0
        self.l_cpu = 0.0; self.a_cpu = 0.0
        self.l_ram = 0.0; self.a_ram = 0.0
        self._lc = 0; self._ac = 0; self._t = time.monotonic()
        self._lp = None; self._ap = None
        self.create_timer(1.0, self._stats)

    @staticmethod
    def _black():
        _, b = cv2.imencode('.jpg', np.zeros((SH, SW, 3), np.uint8), JQ)
        return b.tobytes()

    def _enc(self, msg):
        if msg.width == 0 or msg.height == 0 or len(msg.data) < msg.height * msg.step:
            return None
        if msg.encoding == 'bgr8':
            f = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3)
        elif msg.encoding == 'rgb8':
            f = cv2.cvtColor(np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3),
                             cv2.COLOR_RGB2BGR)
        else:
            return None
        out = f if (msg.width == SW and msg.height == SH) else cv2.resize(f, (SW, SH))
        _, b = cv2.imencode('.jpg', out, JQ)
        return b.tobytes()

    def _lat(self, m):
        return (self.get_clock().now() - Time.from_msg(m.header.stamp)).nanoseconds / 1e6

    def _on_l(self, m):
        try:
            j = self._enc(m)
            if j:
                with self.lock:
                    self.l_jpg = j; self.l_lat = self._lat(m); self._lc += 1
                self.l_evt.set()
        except Exception: pass

    def _on_a(self, m):
        try:
            j = self._enc(m)
            if j:
                with self.lock:
                    self.a_jpg = j; self.a_lat = self._lat(m); self._ac += 1
                self.a_evt.set()
        except Exception: pass

    def _find(self, name):
        for p in psutil.process_iter(['pid', 'cmdline']):
            try:
                cmd = ' '.join(p.info['cmdline'] or [])
                if 'component_container' in cmd and name in cmd:
                    p.cpu_percent(); return p
            except Exception: pass
        return None

    def _validate(self, p, name):
        """Ensure cached Process still exists and matches the expected container.
        Guards against PID reuse and silently-dead containers from prior launches."""
        if p is None:
            return None
        try:
            if not p.is_running():
                return None
            cmd = ' '.join(p.cmdline() or [])
            if 'component_container' not in cmd or name not in cmd:
                return None
            return p
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            return None

    def _read(self, p):
        try:
            return p.cpu_percent(), p.memory_info().rss / 1048576
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            return 0, 0

    def _stats(self):
        self._lp = self._validate(self._lp, 'legacy_container') or self._find('legacy_container')
        self._ap = self._validate(self._ap, 'accel_container') or self._find('accel_container')
        lc, lr = self._read(self._lp) if self._lp else (0, 0)
        ac, ar = self._read(self._ap) if self._ap else (0, 0)
        dt = time.monotonic() - self._t
        with self.lock:
            self.l_cpu = lc; self.l_ram = lr; self.a_cpu = ac; self.a_ram = ar
            self.l_fps = self._lc / dt if dt > 0 else 0
            self.a_fps = self._ac / dt if dt > 0 else 0
            self._lc = 0; self._ac = 0
        self._t = time.monotonic()

    def json(self):
        with self.lock:
            return {
                'll': round(self.l_lat, 1), 'al': round(self.a_lat, 1),
                'lf': round(self.l_fps, 1), 'af': round(self.a_fps, 1),
                'lc': round(self.l_cpu, 0), 'ac': round(self.a_cpu, 0),
                'lr': round(self.l_ram, 0), 'ar': round(self.a_ram, 0),
                'd': round(abs(self.l_lat - self.a_lat), 1),
            }


app = Flask(__name__)
nd = None

MJPEG_BOUNDARY = b'gstadaptframe'

def _mj(side):
    # Per-frame Content-Length lets the browser read exactly len bytes without
    # scanning for the boundary inside JPEG payload — JPEG is binary and can
    # contain CRLF+boundary-like byte sequences that would otherwise truncate
    # the frame and break the stream after a few seconds.
    ev = nd.l_evt if side == 'legacy' else nd.a_evt
    last_jpg = None
    try:
        while True:
            ev.wait(timeout=0.2); ev.clear()
            with nd.lock:
                j = nd.l_jpg if side == 'legacy' else nd.a_jpg
            if j is last_jpg:
                continue
            last_jpg = j
            header = (b'--' + MJPEG_BOUNDARY + b'\r\n'
                      b'Content-Type: image/jpeg\r\n'
                      b'Content-Length: ' + str(len(j)).encode() + b'\r\n\r\n')
            yield header + j + b'\r\n'
    except (GeneratorExit, BrokenPipeError, ConnectionResetError):
        return

@app.route('/stream/<side>')
def stream(side):
    if side not in ('legacy', 'accelerated'): return '', 404
    return Response(_mj(side),
                    mimetype='multipart/x-mixed-replace; boundary=' + MJPEG_BOUNDARY.decode())

@app.route('/api/stats')
def stats():
    return Response(json.dumps(nd.json()), mimetype='application/json')

@app.route('/api/system')
def system():
    return Response(json.dumps({'hw': detect_hardware()}), mimetype='application/json')

@app.route('/')
def index():
    try:
        with open(HTML_PATH, 'r', encoding='utf-8') as f:
            return f.read()
    except FileNotFoundError:
        return f'dashboard.html not found at {HTML_PATH}', 500



def main(args=None):
    global nd
    try:
        out = subprocess.check_output(['fuser', f'{PORT}/tcp'], stderr=subprocess.DEVNULL).decode()
        for p in out.split():
            if p.strip().isdigit(): os.kill(int(p.strip()), signal.SIGKILL)
        time.sleep(0.3)
    except Exception: pass

    rclpy.init(args=args)
    nd = DashNode()
    ex = SingleThreadedExecutor(); ex.add_node(nd)
    stop = threading.Event()
    def spin():
        while not stop.is_set():
            try: ex.spin_once(timeout_sec=0.05)
            except Exception: break
    threading.Thread(target=spin, daemon=True).start()
    nd.get_logger().info(f'Dashboard at http://localhost:{PORT}')

    logging.getLogger('werkzeug').setLevel(logging.ERROR)
    warnings.filterwarnings('ignore', message='.*development server.*')

    try: app.run(host='0.0.0.0', port=PORT, threaded=True, use_reloader=False)
    except KeyboardInterrupt: pass
    stop.set(); ex.shutdown(); nd.destroy_node(); rclpy.try_shutdown()

if __name__ == '__main__': main()
