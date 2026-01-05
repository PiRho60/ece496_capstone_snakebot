# camera_feed_click.py
# Click on the feed to print pixel coordinates

import cv2

WINDOW_NAME = "Camera Feed (click to print coords, ESC to quit)"
last_click = None  # (x, y)

# ---------- Optional digital zoom ----------
def digital_zoom(frame, zoom_factor):
    if zoom_factor <= 1.0:
        return frame
    h, w = frame.shape[:2]
    new_w = int(w / zoom_factor)
    new_h = int(h / zoom_factor)
    x1 = (w - new_w) // 2
    y1 = (h - new_h) // 2
    cropped = frame[y1:y1 + new_h, x1:x1 + new_w]
    return cv2.resize(cropped, (w, h), interpolation=cv2.INTER_LINEAR)

# ---------- Mouse callback ----------
def on_mouse(event, x, y, flags, param):
    global last_click
    if event == cv2.EVENT_LBUTTONDOWN:
        last_click = (x, y)
        print(f"Clicked pixel: (x={x}, y={y})")

def draw_click_overlay(frame):
    if last_click is None:
        return

    x, y = last_click
    h, w = frame.shape[:2]

    size = 8
    cv2.line(frame, (max(0, x - size), y), (min(w - 1, x + size), y), (0, 255, 0), 1)
    cv2.line(frame, (x, max(0, y - size)), (x, min(h - 1, y + size)), (0, 255, 0), 1)

    cv2.putText(
        frame,
        f"({x}, {y})",
        (min(w - 1, x + 10), min(h - 1, y + 20)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (0, 255, 0),
        2,
    )

def main():
    global last_click  # ✅ declared once at function top

    cam_index = 0  # adjust if needed
    cap = cv2.VideoCapture(cam_index)

    if not cap.isOpened():
        raise RuntimeError(f"Could not open camera index {cam_index}")

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    cv2.setMouseCallback(WINDOW_NAME, on_mouse)

    zoom = 1.0

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        frame = digital_zoom(frame, zoom)
        draw_click_overlay(frame)

        cv2.imshow(WINDOW_NAME, frame)

        key = cv2.waitKey(1) & 0xFF
        if key == 27:  # ESC
            break
        elif key in (ord('+'), ord('=')):
            zoom = min(zoom + 0.1, 3.0)
            print(f"Zoom: {zoom:.1f}x")
        elif key == ord('-'):
            zoom = max(zoom - 0.1, 1.0)
            print(f"Zoom: {zoom:.1f}x")
        elif key == ord('c'):
            last_click = None
            print("Cleared click marker")

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
