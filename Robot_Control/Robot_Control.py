import tkinter as tk
from logo import ascii_art
import Hand_Tracker
import hid
from pynput import mouse
import struct
import serial
import serial.tools.list_ports
import threading
import queue
import json
import time
import hid
import sounddevice as sd
from vosk import Model, KaldiRecognizer
from voice_movement import handle_command

# GLOBAL STATE
tracking_mouse = False
tracking_hand = False

mouse_x, mouse_y = 0, 0

servo1_pos = 90
servo2_pos = 140
servo3_pos = 90

listener = None
ser = None
serial_lock = threading.Lock()

claw_grabbing = False
claw_busy = False

CLAW_OPEN_POS = 160
CLAW_CLOSED_POS = 60

# JOYSTICK CONTROL
tracking_joystick = False
joystick_thread = None
joystick_running = threading.Event()

JOYSTICK_VID = 0x046D
JOYSTICK_PID = 0xC215

joystick_x = 0
joystick_y = 0
joystick_button = 0

last_joystick_send_time = 0
JOYSTICK_SEND_INTERVAL = 0.03

# Voice
voice_thread = None
voice_running = threading.Event()
VOICE_SAMPLE_RATE = 16_000
VOICE_BLOCK_LEN = 8_000
VOICE_MODEL_DIR = "models/vosk-model-small-en-us-0.15"

# HELPERS
def clamp(val, lo=0, hi=180):
    return max(lo, min(int(val), hi))

def find_arduino_port():
    ports = list(serial.tools.list_ports.comports())
    for port in ports:
        if 'Arduino' in port.description or 'usbmodem' in port.device:
            return port.device
    return None

def initialize_serial_connection():
    global ser
    port = find_arduino_port()
    if port:
        try:
            ser = serial.Serial(port, 9600, timeout=1)
            print(f"Connected to Arduino on {port}")
            time.sleep(2)
        except Exception as e:
            ser = None
            print(f"Failed to connect to Arduino: {e}")
    else:
        ser = None
        print("Arduino not found.")

def map_value(x, in_min, in_max, out_min, out_max):
    if in_max == in_min:
        return out_min
    return int((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

def send_command():
    global ser, servo1_pos, servo2_pos, servo3_pos

    s1 = clamp(servo1_pos)
    s2 = clamp(servo2_pos)
    s3 = clamp(servo3_pos)

    data = struct.pack('<BHHH', 255, s1, s2, s3)

    if ser:
        try:
            with serial_lock:
                ser.write(data)
        except serial.SerialException:
            print("Serial write failed. Reconnecting...")
            initialize_serial_connection()
    else:
        print("Serial connection not initialized. Reconnecting...")
        initialize_serial_connection()


def update_telemetry():
    mouse_pos_label.config(text=f"Mouse Position: ({mouse_x}, {mouse_y})")
    servo_pos_label.config(
        text=f"Servo Positions: (Servo1: {servo1_pos}, Servo2/Claw: {servo2_pos}, Servo3: {servo3_pos})"
    )
    claw_state_label.config(text=f"Claw State: {'Grabbing' if claw_grabbing else 'Released'}")

    if 'joystick_pos_label' in globals():
        joystick_pos_label.config(text=f"Joystick Position: ({joystick_x}, {joystick_y})")

def load_text_character_by_character(widget, text, index=0, delay=50):
    if index < len(text):
        if isinstance(widget, tk.Text):
            widget.configure(state='normal')
            widget.insert(tk.END, text[index])
            widget.configure(state='disabled')
            widget.see(tk.END)
        elif isinstance(widget, tk.Label):
            current_text = widget.cget("text")
            widget.configure(text=current_text + text[index])

        widget.after(delay, lambda: load_text_character_by_character(widget, text, index + 1, delay))

# CLAW CONTROL
def set_claw_position(target):
    global servo2_pos
    servo2_pos = clamp(target)
    update_telemetry()
    send_command()


def close_claw():
    global claw_busy, claw_grabbing

    if claw_busy:
        return

    claw_busy = True
    claw_grabbing = True
    set_claw_position(CLAW_CLOSED_POS)

    claw_busy = False


def open_claw():
    global claw_busy, claw_grabbing

    if claw_busy:
        return

    claw_busy = True
    claw_grabbing = False

    set_claw_position(CLAW_OPEN_POS)

    claw_busy = False


def toggle_claw():
    if claw_busy:
        return

    if claw_grabbing:
        threading.Thread(target=open_claw, daemon=True).start()
    else:
        threading.Thread(target=close_claw, daemon=True).start()



# MOUSE CONTROL
def on_move(x, y):
    global mouse_x, mouse_y, servo1_pos, servo3_pos

    if tracking_mouse:
        mouse_x, mouse_y = x, y
        servo1_pos = clamp(map_value(mouse_x, 0, 1920, 10, 170))
        servo3_pos = clamp(map_value(mouse_y, 0, 1920, 10, 170))

        update_telemetry()
        send_command()


def on_click(x, y, button, pressed):
    if pressed and button == mouse.Button.left:
        print("Toggling claw...")
        toggle_claw()


def start_mouse_tracking():
    global listener, tracking_mouse

    if tracking_mouse:
        return

    tracking_mouse = True
    listener = mouse.Listener(on_move=on_move, on_click=on_click)
    listener.start()

    activate_mouse_button.config(state="disabled")
    deactivate_mouse_button.config(state="normal")


def stop_mouse_tracking():
    global listener, tracking_mouse

    tracking_mouse = False

    if listener is not None:
        listener.stop()
        listener = None

    activate_mouse_button.config(state="normal")
    deactivate_mouse_button.config(state="disabled")

# JOYSTICK CONTROL
def list_hid_devices():
    print("Listing all HID devices:")
    for device in hid.enumerate():
        print(
            f"VID: {hex(device['vendor_id'])}, "
            f"PID: {hex(device['product_id'])}, "
            f"Product: {device['product_string']}"
        )

def find_joystick_device():
    for device in hid.enumerate():
        vid = device["vendor_id"]
        pid = device["product_id"]
        product = device.get("product_string", "")

        if vid == JOYSTICK_VID and pid == JOYSTICK_PID:
            print(f"Found joystick: {product}")
            print(f"Path: {device['path']}")
            return device

    return None

def joystick_worker():
    global tracking_joystick
    global joystick_x, joystick_y, joystick_button
    global servo1_pos, servo3_pos
    global last_joystick_send_time

    joystick = None

    try:
        list_hid_devices()

        print(f"Looking for joystick: VID={hex(JOYSTICK_VID)}, PID={hex(JOYSTICK_PID)}")

        device_info = find_joystick_device()

        if device_info is None:
            print("[Joystick] device not found.")
            return

        joystick = hid.device()
        joystick.open_path(device_info["path"])

        print(f"Connected to joystick: {joystick.get_product_string()}")
        print("Joystick control active.")

        while joystick_running.is_set():
            data = joystick.read(64, timeout_ms=50)

            if not data:
                continue

            print(f"Raw Joystick Data: {data}")

            joystick_x = data[0]
            joystick_y = data[1]

            new_servo1 = clamp(map_value(joystick_x, 0, 255, 10, 170))
            new_servo3 = clamp(map_value(joystick_y, 0, 255, 10, 170))

            if abs(new_servo1 - servo1_pos) < 2 and abs(new_servo3 - servo3_pos) < 2:
                continue

            servo1_pos = new_servo1
            servo3_pos = new_servo3

            now = time.time()
            if now - last_joystick_send_time >= JOYSTICK_SEND_INTERVAL:
                root.after(0, update_telemetry)
                send_command()
                last_joystick_send_time = now

    except Exception as e:
        print(f"[Joystick] error: {e}")

    finally:
        tracking_joystick = False
        joystick_running.clear()

        if joystick is not None:
            try:
                joystick.close()
                print("Joystick connection closed.")
            except Exception:
                pass

        root.after(0, lambda: activate_joystick_button.config(state="normal"))
        root.after(0, lambda: deactivate_joystick_button.config(state="disabled"))

        print("Joystick control stopped.")


def start_joystick_tracking():
    global tracking_joystick, joystick_thread

    if tracking_joystick:
        return
    if tracking_mouse:
        stop_mouse_tracking()

    if tracking_hand:
        stop_hand_tracking()

    tracking_joystick = True
    joystick_running.set()

    joystick_thread = threading.Thread(target=joystick_worker, daemon=True)
    joystick_thread.start()

    activate_joystick_button.config(state="disabled")
    deactivate_joystick_button.config(state="normal")


def stop_joystick_tracking():
    global tracking_joystick

    tracking_joystick = False
    joystick_running.clear()

    activate_joystick_button.config(state="normal")
    deactivate_joystick_button.config(state="disabled")

# HAND TRACKING
def start_hand_tracking():
    global tracking_hand
    tracking_hand = True
    Hand_Tracker.start_hand_tracker()
    activate_hand_button.config(state="disabled")
    deactivate_hand_button.config(state="normal")


def stop_hand_tracking():
    global tracking_hand

    if not tracking_hand:
        return

    tracking_hand = False

    try:
        Hand_Tracker.stop_hand_tracker()
    except NameError as e:
        print(f"[Hand Tracking] stop ignored: {e}")
    except Exception as e:
        print(f"[Hand Tracking] stop error: {e}")

    activate_hand_button.config(state="normal")
    deactivate_hand_button.config(state="disabled")

# VOICE CONTROL
def voice_worker():
    try:
        model = Model(VOICE_MODEL_DIR)
        rec = KaldiRecognizer(model, VOICE_SAMPLE_RATE)
        rec.SetWords(True)
    except Exception as e:
        print("[Voice] model load error:", e)
        return

    q_audio: queue.Queue[bytes] = queue.Queue()

    def audio_cb(indata, status):
        if status:
            print(status)
        q_audio.put(bytes(indata))

    print("[Voice] model loaded – listening")
    try:
        with sd.RawInputStream(
            samplerate=VOICE_SAMPLE_RATE,
            blocksize=VOICE_BLOCK_LEN,
            dtype="int16",
            channels=1,
            callback=audio_cb,
        ):
            while voice_running.is_set():
                data = q_audio.get()
                if rec.AcceptWaveform(data):
                    result = json.loads(rec.Result())
                    text = result.get("text", "").strip()
                    if text:
                        print(f">> {text}")
                        handle_command(text.split())
    except Exception as e:
        print("[Voice] error:", e)

    print("[Voice] stopped")


def start_voice():
    global voice_thread

    if voice_thread and voice_thread.is_alive():
        return

    voice_running.set()
    voice_thread = threading.Thread(target=voice_worker, daemon=True)
    voice_thread.start()

    activate_voice_btn.config(state="disabled")
    deactivate_voice_btn.config(state="normal")


def stop_voice():
    voice_running.clear()
    activate_voice_btn.config(state="normal")
    deactivate_voice_btn.config(state="disabled")

# UI
initialize_serial_connection()

root = tk.Tk()
root.title("Robot Control")
root.geometry("1280x720")
root.configure(bg='black')

text_color = "#00ff00"

title_label = tk.Label(
    root,
    font=("Courier New", 10),
    bg='black',
    fg=text_color,
    anchor='center',
    justify='center'
)
title_label.pack(padx=10, pady=10)

activate_mouse_button = tk.Button(
    root,
    text="Activate Mouse Tracking",
    command=start_mouse_tracking,
    bg='green',
    fg='black'
)
activate_mouse_button.pack(pady=10, padx=10)

deactivate_mouse_button = tk.Button(
    root,
    text="Deactivate Mouse Tracking",
    command=stop_mouse_tracking,
    bg='red',
    fg='black'
)
deactivate_mouse_button.pack(pady=10, padx=10)
deactivate_mouse_button.config(state="disabled")

activate_hand_button = tk.Button(
    root,
    text="Activate Hand Tracking",
    command=start_hand_tracking,
    bg='blue',
    fg='black'
)
activate_hand_button.pack(pady=10, padx=10)

deactivate_hand_button = tk.Button(
    root,
    text="Deactivate Hand Tracking",
    command=stop_hand_tracking,
    bg='orange',
    fg='black'
)
deactivate_hand_button.pack(pady=10, padx=10)
deactivate_hand_button.config(state="disabled")

activate_joystick_button = tk.Button(
    root,
    text="Activate Joystick Control",
    command=start_joystick_tracking,
    bg='purple',
    fg='black'
)
activate_joystick_button.pack(pady=10, padx=10)

deactivate_joystick_button = tk.Button(
    root,
    text="Deactivate Joystick Control",
    command=stop_joystick_tracking,
    bg='orange',
    fg='black'
)
deactivate_joystick_button.pack(pady=10, padx=10)
deactivate_joystick_button.config(state="disabled")

activate_voice_btn = tk.Button(
    root,
    text="Activate Voice Cmd",
    command=start_voice,
    bg="green",
    fg="black"
)
activate_voice_btn.pack(pady=10, padx=10)

deactivate_voice_btn = tk.Button(
    root,
    text="Stop Voice Cmd",
    command=stop_voice,
    bg="orange",
    fg="black"
)
deactivate_voice_btn.pack(pady=10, padx=10)
deactivate_voice_btn.config(state="disabled")

mouse_pos_label = tk.Label(
    root,
    text="Mouse Position: (0, 0)",
    bg='black',
    fg=text_color
)
mouse_pos_label.pack(pady=10)

joystick_pos_label = tk.Label(
    root,
    text="Joystick Position: (0, 0)",
    bg='black',
    fg=text_color
)
joystick_pos_label.pack(pady=10)

servo_pos_label = tk.Label(
    root,
    text=f"Servo Positions: (Servo1: {servo1_pos}, Servo2/Claw: {servo2_pos})",
    bg='black',
    fg=text_color
)
servo_pos_label.pack(pady=10)

claw_state_label = tk.Label(
    root,
    text="Claw State: Released",
    bg='black',
    fg=text_color
)
claw_state_label.pack(pady=10)

root.after(1000, lambda: load_text_character_by_character(title_label, ascii_art, 0, 1))

update_telemetry()
send_command()

root.mainloop()