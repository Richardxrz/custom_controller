"""本地 URDF 查看器服务：托管页面 + 模型 + /angles。

用法：
    python3 server.py --model /tmp/rebot/.../rebotarm_bringup \
        --urdf description/urdf/ReBot_Arm_DM.urdf --open
"""

from __future__ import annotations

import argparse
import json
import mimetypes
import os
import posixpath
import sys
import threading
import xml.etree.ElementTree as ET
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import unquote, urlparse

def _find_web_dir() -> str:
    here = os.path.dirname(os.path.abspath(__file__))
    candidates = [here]
    meipass = getattr(sys, "_MEIPASS", None)
    if meipass:
        candidates += [os.path.join(meipass, "web_viewer"),
                       os.path.join(os.path.dirname(meipass), "Resources", "web_viewer")]
    candidates.append(os.path.join(os.path.dirname(here), "Resources", "web_viewer"))
    for c in candidates:
        if os.path.isfile(os.path.join(c, "index.html")):
            return c
    return here


WEB_DIR = _find_web_dir()
MODEL_DIR = ""
URDF_REL = ""
ANGLES = [0.0] * 6
LOCK = threading.Lock()


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass

    def _send(self, code, body: bytes, ctype="application/octet-stream"):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _file(self, path, ctype=None):
        try:
            with open(path, "rb") as f:
                data = f.read()
        except OSError:
            self._send(404, b"not found", "text/plain")
            return
        ctype = ctype or mimetypes.guess_type(path)[0] or "application/octet-stream"
        self._send(200, data, ctype)

    def do_GET(self):
        path = unquote(urlparse(self.path).path)
        if path == "/angles":
            with LOCK:
                self._send(200, json.dumps(ANGLES).encode(), "application/json")
            return
        if path == "/model/urdf":
            p = os.path.join(MODEL_DIR, URDF_REL)
            try:
                text = open(p, "rb").read()
            except OSError:
                self._send(404, b"urdf not found", "text/plain")
                return
            try:
                root = ET.fromstring(text)
                base = posixpath.dirname(URDF_REL)
                for mesh in root.iter():
                    if mesh.tag.split("}")[-1].lower() != "mesh":
                        continue
                    fn = (mesh.get("filename") or "").replace("\\", "/")
                    if fn.startswith("package://"):
                        fn = fn[len("package://"):].split("/", 1)[-1]
                    elif fn.startswith("file://"):
                        fn = fn[len("file://"):]
                    elif fn.startswith("/"):
                        fn = fn.lstrip("/")
                    else:
                        fn = posixpath.normpath(posixpath.join(base, fn))
                    mesh.set("filename", fn)
                text = ET.tostring(root, encoding="utf-8", xml_declaration=True)
            except ET.ParseError:
                pass
            self._send(200, text, "text/xml")
            return
        if path.startswith("/model/"):
            rel = path[len("/model/"):]
            full = os.path.realpath(os.path.join(MODEL_DIR, rel))
            if not full.startswith(os.path.realpath(MODEL_DIR) + os.sep):
                self._send(403, b"forbidden", "text/plain")
                return
            self._file(full)
            return
        if path == "/":
            path = "/index.html"
        full = os.path.realpath(os.path.join(WEB_DIR, path.lstrip("/")))
        if not full.startswith(WEB_DIR + os.sep):
            self._send(403, b"forbidden", "text/plain")
            return
        self._file(full)

    def do_POST(self):
        global ANGLES
        if urlparse(self.path).path != "/angles":
            self._send(404, b"not found", "text/plain")
            return
        n = int(self.headers.get("Content-Length", "0"))
        try:
            data = json.loads(self.rfile.read(min(n, 4096)))
            vals = [float(x) for x in data][:6]
            if len(vals) != 6:
                raise ValueError
        except Exception:
            self._send(400, b"bad angles", "text/plain")
            return
        with LOCK:
            ANGLES = vals
        self._send(200, b"ok", "text/plain")


def find_urdf(model_dir: str) -> str:
    for root, _, files in os.walk(model_dir):
        for f in files:
            if f.lower().endswith(".urdf"):
                return os.path.relpath(os.path.join(root, f), model_dir)
    raise SystemExit("模型目录里找不到 .urdf，请用 --urdf 指定")


def main() -> int:
    global MODEL_DIR, URDF_REL
    ap = argparse.ArgumentParser()
    rebot_model = os.path.join(WEB_DIR, "models", "rebot")
    demo = os.path.join(WEB_DIR, "models", "demo")
    ap.add_argument("--model", default=rebot_model if os.path.isdir(rebot_model) else demo)
    ap.add_argument("--urdf")
    ap.add_argument("--port", type=int, default=8765)
    ap.add_argument("--open", action="store_true")
    args = ap.parse_args()
    MODEL_DIR = os.path.realpath(args.model)
    URDF_REL = args.urdf or find_urdf(MODEL_DIR)
    if not os.path.isfile(os.path.join(MODEL_DIR, URDF_REL)):
        raise SystemExit(f"URDF 不存在: {URDF_REL}")
    httpd = ThreadingHTTPServer(("127.0.0.1", args.port), Handler)
    url = f"http://127.0.0.1:{args.port}/"
    print(f"viewer: {url}\nmodel : {MODEL_DIR}\nurdf  : {URDF_REL}")
    if args.open:
        import webbrowser

        threading.Timer(0.5, lambda: webbrowser.open(url)).start()
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        httpd.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
