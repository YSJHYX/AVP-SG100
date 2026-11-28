"""Loopback-only HTTP workbench. Opening a page does not initialize devices."""
from __future__ import annotations

import argparse
import atexit
from pathlib import Path
import secrets
from urllib.parse import urlsplit

from flask import Flask, Response, abort, jsonify, render_template, request, send_file

from .runtime import ROOT, Workstation


def create_app(station=None):
    app = Flask(__name__)
    app.config["MAX_CONTENT_LENGTH"] = 16 * 1024
    station = station or Workstation(ROOT / "recording/ui/station.json")
    app.extensions["workstation"] = station
    token = secrets.token_urlsafe(32)

    @app.before_request
    def local_request():
        if request.host.split(":", 1)[0] not in ("127.0.0.1", "localhost"):
            abort(403)
        if request.method == "POST":
            origin = request.headers.get("Origin")
            if origin and urlsplit(origin).netloc != request.host:
                abort(403)
            if not secrets.compare_digest(request.headers.get("X-Workbench-Token", ""), token):
                abort(403)

    @app.after_request
    def headers(response):
        response.headers["Cache-Control"] = "no-store"
        response.headers["X-Content-Type-Options"] = "nosniff"
        response.headers["Content-Security-Policy"] = "default-src 'self'; img-src 'self' data:; style-src 'self'; script-src 'self'; frame-ancestors 'none'"
        return response

    @app.get("/")
    def index():
        return render_template("index.html", token=token, demo=station.demo)

    @app.get("/api/status")
    def status():
        return jsonify(station.snapshot())

    @app.post("/api/action")
    def action():
        body = request.get_json(silent=True)
        if not isinstance(body, dict):
            return jsonify(error="无效请求"), 400
        command = body.get("action")
        # A stop remains available while ordinary operations wait/flush.
        lock = command != "disarm"
        if lock and not station.operation_lock.acquire(blocking=False):
            return jsonify(error="另一项操作正在进行，请稍候"), 409
        try:
            if command == "configure":
                station.configure(body.get("config"))
            elif command == "start":
                station.start()
            elif command in ("calibrate", "arm", "disarm"):
                station.control(command)
            elif command == "mode":
                station.control(command, mode=body.get("mode"))
            elif command == "script":
                station.control(command, motion=body.get("motion"))
            elif command == "begin":
                station.begin(body.get("instruction", ""))
            elif command in ("finish", "off"):
                station.finish(stop_motion=command == "finish")
            elif command == "restart":
                station.restart()
            elif command == "reference":
                station.capture_reference()
            elif command == "confirm":
                station.confirm(body.get("decision"), dagger=body.get("dagger") is True)
            elif command == "close":
                station.close()
            else:
                raise ValueError("未知操作")
            return jsonify(station.snapshot())
        except (RuntimeError, ValueError, OSError) as exc:
            station.event(str(exc))
            return jsonify(error=str(exc), status=station.snapshot()), 400
        finally:
            if lock:
                station.operation_lock.release()

    @app.get("/stream/<role>")
    def stream(role):
        if role not in ("head", "right_wrist") or station.preview is None:
            abort(404)
        return Response(station.preview.stream(role), mimetype="multipart/x-mixed-replace; boundary=frame")

    @app.get("/api/media/<episode>/<role>/<int:index>.jpg")
    def media(episode, role, index):
        try:
            return send_file(station.media_path(episode, role, index), mimetype="image/jpeg")
        except (ValueError, FileNotFoundError):
            abort(404)

    @app.get("/api/reference.jpg")
    def reference():
        if not station.reference_path.is_file():
            abort(404)
        return send_file(station.reference_path, mimetype="image/jpeg")

    return app


def main():
    parser = argparse.ArgumentParser(description="AVP-SG100 本机数采工作台")
    parser.add_argument("--demo", action="store_true", help="离线演示，不打开真实硬件")
    parser.add_argument("--port", type=int, default=8090)
    parser.add_argument("--config", type=Path)
    args = parser.parse_args()
    config = args.config or ROOT / ("recording/ui/demo-station.json" if args.demo else "recording/ui/station.json")
    station = Workstation(config, demo=args.demo)
    atexit.register(station.emergency_cleanup)
    print(f"打开 http://127.0.0.1:{args.port}；准备设备前请核对工位配置。", flush=True)
    # No reloader: one process owns the cameras and the episode controller.
    create_app(station).run(host="127.0.0.1", port=args.port, threaded=True, use_reloader=False)


if __name__ == "__main__":
    main()
