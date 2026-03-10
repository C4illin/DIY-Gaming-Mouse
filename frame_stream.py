#!/usr/bin/env python3
"""Live stream frames from the PMW3360 sensor. Press ESC or Q to quit."""

import sys
import time
import serial
import numpy as np
import cv2
from collections import deque

FRAME_SIZE = 36
FRAME_PIXELS = FRAME_SIZE * FRAME_SIZE  # 1296
HEADER = b"FRAME"
FOOTER = b"END"
DISPLAY_SCALE = 16  # 36 * 16 = 576px

def read_frame(ser):
    """Read one frame from serial. Returns 36x36 numpy array or None on timeout."""
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
    # scale 7-bit (0-127) to 8-bit (0-255)
    frame = np.clip(frame.astype(np.uint16) * 2, 0, 255).astype(np.uint8)
    frame = np.rot90(frame, k=1)
    return frame

def main():
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} <serial-port> [avg-frames]")
        print(f"  e.g. {sys.argv[0]} /dev/ttyACM0")
        print(f"       {sys.argv[0]} /dev/ttyACM0 4")
        sys.exit(1)

    port = sys.argv[1]
    avg_count = int(sys.argv[2]) if len(sys.argv) > 2 else 1
    display_size = FRAME_SIZE * DISPLAY_SCALE

    ser = serial.Serial(port, 115200, timeout=10)
    ser.reset_input_buffer()

    cv2.namedWindow("PMW3360", cv2.WINDOW_AUTOSIZE)

    # start streaming mode
    ser.write(b"c")

    frame_buf = deque(maxlen=max(avg_count, 1))

    frame_count = 0
    fps_time = time.time()
    fps = 0

    try:
        while True:
            frame = read_frame(ser)
            if frame is None:
                print("Timeout waiting for frame, retrying...")
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

            if avg_count > 1:
                frame_buf.append(frame.astype(np.float32))
                display_frame = np.mean(frame_buf, axis=0).astype(np.uint8)
            else:
                display_frame = frame

            display = cv2.resize(display_frame, (display_size, display_size),
                                 interpolation=cv2.INTER_NEAREST)

            info = f"{fps:.1f} FPS"
            if avg_count > 1:
                info += f" | avg: {avg_count}"
            cv2.putText(display, info, (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, 255, 2)

            cv2.imshow("PMW3360", display)

            key = cv2.waitKey(1) & 0xFF
            if key == 27 or key == ord('q'):
                break

    except KeyboardInterrupt:
        pass
    finally:
        ser.write(b"x")
        time.sleep(0.1)
        ser.close()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
