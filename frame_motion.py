#!/usr/bin/env python3
"""Live stream from PMW3360 with optical flow motion vectors overlaid.
   Press ESC/Q to quit. This visualizes how the sensor tracks movement."""

import sys
import time
import serial
import numpy as np
import cv2
from collections import deque

FRAME_SIZE = 36
FRAME_PIXELS = FRAME_SIZE * FRAME_SIZE
HEADER = b"FRAME"
FOOTER = b"END"
DISPLAY_SCALE = 16
BLOCK_SIZE = 6  # motion vector grid: 36/6 = 6x6 grid of vectors

def read_frame(ser):
    buf = b""
    while True:
        byte = ser.read(1)
        if not byte:
            return None
        buf += byte
        if buf.endswith(HEADER):
            break
        if len(buf) > 256:
            buf = buf[-len(HEADER):]

    pixel_data = ser.read(FRAME_PIXELS)
    if len(pixel_data) != FRAME_PIXELS:
        return None
    ser.read(len(FOOTER))

    frame = np.frombuffer(pixel_data, dtype=np.uint8).reshape(FRAME_SIZE, FRAME_SIZE)
    frame = np.clip(frame.astype(np.uint16) * 2, 0, 255).astype(np.uint8)
    # sensor is rotated 90 degrees clockwise in the mouse
    frame = np.rot90(frame, k=1)
    return frame

def main():
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} <serial-port>")
        sys.exit(1)

    port = sys.argv[1]
    display_size = FRAME_SIZE * DISPLAY_SCALE

    ser = serial.Serial(port, 115200, timeout=10)
    ser.reset_input_buffer()
    ser.write(b"c")

    cv2.namedWindow("PMW3360 Motion", cv2.WINDOW_AUTOSIZE)

    prev_frame = None
    frame_count = 0
    fps_time = time.time()
    fps = 0

    # accumulated motion trail
    trail = np.zeros((display_size, display_size, 3), dtype=np.uint8)
    total_dx = 0.0
    total_dy = 0.0

    try:
        while True:
            frame = read_frame(ser)
            if frame is None:
                print("Timeout, retrying...")
                ser.reset_input_buffer()
                ser.write(b"c")
                continue

            frame_count += 1
            now = time.time()
            elapsed = now - fps_time
            if elapsed >= 1.0:
                fps = frame_count / elapsed
                frame_count = 0
                fps_time = now

            # compute optical flow
            flow = None
            if prev_frame is not None:
                flow = cv2.calcOpticalFlowFarneback(
                    prev_frame, frame,
                    None,
                    pyr_scale=0.5, levels=3, winsize=5,
                    iterations=3, poly_n=5, poly_sigma=1.1, flags=0
                )

            prev_frame = frame.copy()

            # build color display (grayscale -> BGR)
            display = cv2.resize(frame, (display_size, display_size),
                                 interpolation=cv2.INTER_NEAREST)
            display = cv2.cvtColor(display, cv2.COLOR_GRAY2BGR)

            if flow is not None:
                # compute overall motion (average flow)
                mean_flow = np.mean(flow, axis=(0, 1))
                total_dx += mean_flow[0]
                total_dy += mean_flow[1]

                # draw motion vectors on a grid
                half = BLOCK_SIZE // 2
                for gy in range(0, FRAME_SIZE, BLOCK_SIZE):
                    for gx in range(0, FRAME_SIZE, BLOCK_SIZE):
                        # average flow in this block
                        block_flow = flow[gy:gy+BLOCK_SIZE, gx:gx+BLOCK_SIZE]
                        dx = np.mean(block_flow[:, :, 0])
                        dy = np.mean(block_flow[:, :, 1])

                        # scale to display coordinates
                        cx = (gx + half) * DISPLAY_SCALE
                        cy = (gy + half) * DISPLAY_SCALE

                        # arrow length scaled up for visibility
                        arrow_scale = DISPLAY_SCALE * 3
                        ex = int(cx + dx * arrow_scale)
                        ey = int(cy + dy * arrow_scale)

                        # color based on magnitude
                        mag = np.sqrt(dx*dx + dy*dy)
                        if mag > 0.2:
                            # green to red based on speed
                            intensity = min(mag * 3, 1.0)
                            color = (0, int(255 * (1 - intensity)), int(255 * intensity))
                            cv2.arrowedLine(display, (cx, cy), (ex, ey),
                                            color, 2, tipLength=0.3)

                # draw accumulated motion trail dot
                trail_x = int(display_size // 2 + total_dx * DISPLAY_SCALE * 0.5)
                trail_y = int(display_size // 2 + total_dy * DISPLAY_SCALE * 0.5)

                # fade trail
                trail = (trail * 0.97).astype(np.uint8)

                # draw trail dot if in bounds
                if 0 <= trail_x < display_size and 0 <= trail_y < display_size:
                    cv2.circle(trail, (trail_x, trail_y), 3, (0, 255, 255), -1)

                # blend trail onto display
                display = cv2.add(display, trail)

                # draw crosshair at center
                cv2.drawMarker(display, (display_size // 2, display_size // 2),
                               (100, 100, 100), cv2.MARKER_CROSS, 20, 1)

            # overlay info
            cv2.putText(display, f"{fps:.1f} FPS", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.putText(display, f"dx:{total_dx:.2f} dy:{total_dy:.2f}", (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 200), 2)

            cv2.imshow("PMW3360 Motion", display)

            key = cv2.waitKey(1) & 0xFF
            if key == 27 or key == ord('q'):
                break
            elif key == ord('r'):
                # reset trail
                trail[:] = 0
                total_dx = 0.0
                total_dy = 0.0

    except KeyboardInterrupt:
        pass
    finally:
        ser.write(b"x")
        time.sleep(0.1)
        ser.close()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
