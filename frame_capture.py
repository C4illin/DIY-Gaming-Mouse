#!/usr/bin/env python3
"""Capture and display a frame from the PMW3360 sensor via serial."""

import sys
import serial
import numpy as np
from PIL import Image

FRAME_SIZE = 36
FRAME_PIXELS = FRAME_SIZE * FRAME_SIZE  # 1296
HEADER = b"FRAME"
FOOTER = b"END"

def capture_frame(port, baudrate=115200, timeout=10):
    with serial.Serial(port, baudrate, timeout=timeout) as ser:
        # flush any stale data
        ser.reset_input_buffer()

        # send frame capture command
        ser.write(b"f")

        # wait for FRAME header
        buf = b""
        while True:
            byte = ser.read(1)
            if not byte:
                raise TimeoutError("Timed out waiting for FRAME header")
            buf += byte
            if buf.endswith(HEADER):
                break
            # keep only last few bytes to avoid unbounded buffer
            if len(buf) > 64:
                buf = buf[-len(HEADER):]

        # read 1296 pixel bytes
        pixel_data = ser.read(FRAME_PIXELS)
        if len(pixel_data) != FRAME_PIXELS:
            raise IOError(f"Expected {FRAME_PIXELS} bytes, got {len(pixel_data)}")

        # read END footer
        footer = ser.read(len(FOOTER))
        if footer != FOOTER:
            print(f"Warning: expected footer {FOOTER!r}, got {footer!r}")

    return pixel_data

def main():
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} <serial-port> [output.png]")
        print(f"  e.g. {sys.argv[0]} /dev/ttyACM0 frame.png")
        sys.exit(1)

    port = sys.argv[1]
    output = sys.argv[2] if len(sys.argv) > 2 else None

    print(f"Requesting frame capture from {port}...")
    pixel_data = capture_frame(port)

    # convert to numpy array and reshape to 36x36
    # pixel values are 0-127 (7-bit), scale to 0-255
    frame = np.frombuffer(pixel_data, dtype=np.uint8).reshape(FRAME_SIZE, FRAME_SIZE)
    frame = np.clip(frame * 2, 0, 255).astype(np.uint8)
    frame = np.rot90(frame, k=1)

    print(f"Frame captured: {FRAME_SIZE}x{FRAME_SIZE}, "
          f"min={frame.min()}, max={frame.max()}, mean={frame.mean():.1f}")

    if output:
        img = Image.fromarray(frame, mode="L")
        # save a scaled-up version so it's not tiny
        img_scaled = img.resize((360, 360), Image.NEAREST)
        img_scaled.save(output)
        print(f"Saved to {output}")
    else:
        # display with pillow
        img = Image.fromarray(frame, mode="L")
        img_scaled = img.resize((360, 360), Image.NEAREST)
        img_scaled.show()

if __name__ == "__main__":
    main()
