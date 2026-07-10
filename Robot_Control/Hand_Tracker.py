import cv2
import mediapipe as mp
import numpy as np

mp_drawing = mp.solutions.drawing_utils
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(min_detection_confidence=0.7, min_tracking_confidence=0.7)

# NOTE: Hand_Tracker no longer opens its own serial connection or packs its
# own packet. It used to send struct.pack('HHHHH', ...) for a 5-servo rig,
# which doesn't match Robot_Control.ino (3 servos: wrist/claw/elbow, framed
# with a leading 255 byte). Instead it just reports servo targets through
# apply_servo_update(), which Robot_Control.py owns, so there is exactly one
# place that talks to the Arduino and exactly one packet format in use.

cap = None
_running = False


def is_fist(landmarks):
    def curled(tip, pip):
        return tip.y > pip.y

    index_tip = landmarks[mp_hands.HandLandmark.INDEX_FINGER_TIP]
    index_pip = landmarks[mp_hands.HandLandmark.INDEX_FINGER_PIP]
    middle_tip = landmarks[mp_hands.HandLandmark.MIDDLE_FINGER_TIP]
    middle_pip = landmarks[mp_hands.HandLandmark.MIDDLE_FINGER_PIP]
    ring_tip = landmarks[mp_hands.HandLandmark.RING_FINGER_TIP]
    ring_pip = landmarks[mp_hands.HandLandmark.RING_FINGER_PIP]
    pinky_tip = landmarks[mp_hands.HandLandmark.PINKY_TIP]
    pinky_pip = landmarks[mp_hands.HandLandmark.PINKY_PIP]

    curled_count = 0
    for tip, pip in [
        (index_tip, index_pip),
        (middle_tip, middle_pip),
        (ring_tip, ring_pip),
        (pinky_tip, pinky_pip),
    ]:
        if curled(tip, pip):
            curled_count += 1
    return curled_count >= 3


def is_hand_open(landmarks):
    wrist = landmarks[mp_hands.HandLandmark.WRIST]
    index_tip = landmarks[mp_hands.HandLandmark.INDEX_FINGER_TIP]
    middle_tip = landmarks[mp_hands.HandLandmark.MIDDLE_FINGER_TIP]
    ring_tip = landmarks[mp_hands.HandLandmark.RING_FINGER_TIP]
    pinky_tip = landmarks[mp_hands.HandLandmark.PINKY_TIP]

    def distance(point1, point2):
        return np.sqrt((point1.x - point2.x) ** 2 + (point1.y - point2.y) ** 2)

    distances = [
        distance(wrist, index_tip),
        distance(wrist, middle_tip),
        distance(wrist, ring_tip),
        distance(wrist, pinky_tip),
    ]

    return np.mean(distances) > 0.3


def map_value(x, in_min, in_max, out_min, out_max):
    return int((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)


def start_hand_tracker(apply_servo_update, claw_closed=60, claw_open=160):
    """
    apply_servo_update(s1=None, s2=None, s3=None) is injected by Robot_Control.py.
    It owns the serial connection, the lock, and the wire format - this
    function only ever reports where it thinks the wrist/claw/elbow should go.

    Runs a blocking capture loop, so the caller is expected to run this on a
    background thread (Robot_Control.py does). Note: cv2.imshow/waitKey from
    a non-main thread is fine on Linux with the GTK/Qt backends this project
    uses, but isn't guaranteed on every platform (notably macOS) - if you
    port this elsewhere and the preview window misbehaves, that's why.
    """
    global cap, _running
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Error: Could not open webcam.")
        return

    _running = True
    claw_grabbing = True
    baseline_angle = None
    previous_claw_state = None

    while _running:
        ret, image = cap.read()
        if not ret:
            print("Error: Could not read frame from webcam.")
            continue

        image = cv2.flip(image, 1)
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        results = hands.process(image_rgb)

        image = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2BGR)
        hand_landmarks_list = results.multi_hand_landmarks if results.multi_hand_landmarks else []

        if hand_landmarks_list:
            hand_landmarks = hand_landmarks_list[0].landmark

            hand_open = is_hand_open(hand_landmarks)
            hand_fist = is_fist(hand_landmarks)

            if hand_fist and not claw_grabbing:
                claw_grabbing = True
            elif hand_open and claw_grabbing:
                claw_grabbing = False

            if claw_grabbing != previous_claw_state:
                print(f"Claw state: {'Grabbing' if claw_grabbing else 'Releasing'}")
                previous_claw_state = claw_grabbing

            wrist = hand_landmarks[mp_hands.HandLandmark.WRIST]
            middle_tip = hand_landmarks[mp_hands.HandLandmark.MIDDLE_FINGER_TIP]

            dx = middle_tip.x - wrist.x
            dy = middle_tip.y - wrist.y
            current_angle = np.arctan2(dy, dx) * 180 / np.pi

            if baseline_angle is None:
                baseline_angle = current_angle

            dial_angle = current_angle - baseline_angle
            dial_angle = (dial_angle + 180) % 360 - 180

            center = (int(image.shape[1] * wrist.x), int(image.shape[0] * wrist.y))
            cv2.ellipse(image, center, (50, 50), -90, 0, dial_angle, (255, 0, 0), 5)
            cv2.putText(image, f'{int(dial_angle)}', (center[0], center[1] - 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 0, 0), 3, cv2.LINE_AA)

            hand_pos_y = wrist.y * image.shape[0]

            # Only 3 real servos exist on the arm (wrist/claw/elbow), so the
            # old servo2/servo3 split for vertical+horizontal hand position
            # doesn't apply any more - servo2 is the claw now. Dial rotation
            # drives the wrist, vertical hand position drives the elbow.
            apply_servo_update(
                s1=map_value(dial_angle, -180, 180, 0, 180),
                s2=claw_closed if claw_grabbing else claw_open,
                s3=map_value(hand_pos_y, 0, image.shape[0], 10, 170),
            )

            mp_drawing.draw_landmarks(
                image,
                results.multi_hand_landmarks[0],
                mp_hands.HAND_CONNECTIONS,
                mp_drawing.DrawingSpec(color=(0, 0, 255), thickness=5, circle_radius=5),
                mp_drawing.DrawingSpec(color=(0, 255, 0), thickness=5))

        cv2.imshow('Hand Tracker', image)
        if cv2.waitKey(1) & 0xFF == 27:
            break

    if cap:
        cap.release()
    cv2.destroyAllWindows()


def stop_hand_tracker():
    global cap, _running
    _running = False
    if cap:
        cap.release()
        cap = None
    cv2.destroyAllWindows()