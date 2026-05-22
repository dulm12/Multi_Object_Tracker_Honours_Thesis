import cv2

video_path = r"C:\Users\Dulmith Pitigalage\Thesis_C\Data_Sessions\Session_2026-03-17_17-11-01\cam1_raw.mp4"

cap = cv2.VideoCapture(video_path)
ret, frame = cap.read()
cap.release()

def mouse_callback(event, x, y, flags, param):
    if event == cv2.EVENT_MOUSEMOVE:
        display = frame.copy()
        cv2.putText(display, f"u={x}, v={y}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        # Draw horizontal line across full width at current v
        cv2.line(display, (0, y), (1280, y), (0, 255, 0), 1)
        cv2.imshow("Find Roofline", display)

cv2.imshow("Find Roofline", frame)
cv2.setMouseCallback("Find Roofline", mouse_callback)
cv2.waitKey(0)
cv2.destroyAllWindows()