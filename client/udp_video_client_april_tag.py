import cv2
import socket
import struct
import numpy as np
import time
import os
import configparser
import sys
import argparse
from collections import defaultdict
from pupil_apriltags import Detector

TIMEOUT = 0.5  # seconds before tossing half‑baked frame


def main():
    # --- Load Credentials ---
    parser = argparse.ArgumentParser(
        description="UDP Video Stream Client with AprilTag Detection"
    )
    parser.add_argument(
        "--wifi",
        action="store_true",
        help="Use WiFi IP/network settings instead of LAN.",
    )
    parser.add_argument("--timeout", type=float, default=TIMEOUT)
    args = parser.parse_args()

    mode = "wifi" if args.wifi else "lan"
    # The client needs to know the server's ports, so we read the server config file
    config_path = os.path.expanduser(".rov_server_creds")

    config = configparser.ConfigParser()
    if not os.path.exists(config_path) or not config.read(config_path):
        sys.exit(f"✗ ERROR: Config file not found or is empty at '{config_path}'")

    try:
        creds = config[mode]
        server_ip = creds["rov_ip"]
        # Uses the single video port
        port = config.getint("DEFAULT", "video_port")
        print(f"✓ Loaded '{mode}' settings from '{config_path}'")
    except (KeyError, configparser.NoSectionError) as e:
        sys.exit(f"✗ ERROR: Missing section or key in config file: {e}")
    # --- End Load Credentials ---

    # build tag detector once (cpu‑only, no fuss)
    detector = Detector(families="tag36h11")
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("", port))  # Bind to the unified video port
    sock.settimeout(1.0)
    print(f"Listening for video from {server_ip} on port {port} (with AprilTag)")

    buffers = defaultdict(lambda: {"total": None, "parts": {}, "ts": time.time()})

    while True:
        try:
            packet, addr = sock.recvfrom(65_535)
        except socket.timeout:
            continue

        # Filter packets to only accept from the configured ROV IP
        if server_ip != "any" and addr[0] != server_ip:
            continue

        fid, total, idx = struct.unpack("!HHH", packet[:6])
        payload = packet[6:]

        buf = buffers[fid]
        buf["ts"] = time.time()
        if buf["total"] is None:
            buf["total"] = total
        buf["parts"][idx] = payload

        # got all chunks?
        if len(buf["parts"]) == buf["total"]:
            parts = [buf["parts"][i] for i in range(buf["total"])]
            jpg = b"".join(parts)
            frame = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
            if frame is not None:
                # -------- AprilTag pass ----------
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                tags = detector.detect(gray)
                for t in tags:
                    pts = t.corners.astype(int)
                    for i in range(4):
                        p1 = (int(pts[i][0]), int(pts[i][1]))
                        p2 = (int(pts[(i + 1) % 4][0]), int(pts[(i + 1) % 4][1]))
                        cv2.line(frame, p1, p2, (0, 255, 0), 2)
                    cX, cY = int(t.center[0]), int(t.center[1])
                    cv2.circle(frame, (cX, cY), 4, (0, 0, 255), -1)
                    cv2.putText(
                        frame,
                        f"id {t.tag_id}",
                        (cX + 6, cY - 6),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (0, 0, 255),
                        2,
                    )
                # ---------------------------------

                cv2.imshow("UDP Stream", frame)
                if cv2.waitKey(1) == 27:  # ESC to quit
                    break
            del buffers[fid]

        # purge stale frames
        now = time.time()
        dead = [k for k, v in buffers.items() if now - v["ts"] > args.timeout]
        for k in dead:
            del buffers[k]

    sock.close()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
