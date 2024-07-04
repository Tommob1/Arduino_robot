import serial
from pynput import mouse
import tkinter as tk
import struct

ser = serial.Serial('/dev/tty.usbmodem11401', 9600)  # Update this to your specific port
mouse_x, mouse_y = 0, 0
servo1_pos, servo2_pos, servo3_pos = 90, 90, 90
listener = None

def on_move(x, y):
    global mouse_x, mouse_y, servo1_pos, servo2_pos, servo3_pos
    mouse_x, mouse_y = x, y
    servo1_pos = int(map_value(mouse_x, 0, 1920, 10, 170))  # Adjust screen height as needed
    servo2_pos = int(map_value(mouse_y, 0, 1080, 10, 170))  # Adjust screen width as needed
    servo3_pos = int(map_value(servo2_pos, 10, 170, 10, 170))
    update_telemetry()
    send_command()

def send_command():
    data = struct.pack('HHH', servo1_pos, servo2_pos, servo3_pos)
    ser.write(data)

def map_value(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def update_telemetry():
    mouse_pos_label.config(text=f"Mouse Position: ({mouse_x}, {mouse_y})")
    servo_pos_label.config(text=f"Servo Positions: (Servo1: {servo1_pos}, Servo2: {servo2_pos}, Servo3: {servo3_pos})")

def start_tracking():
    global listener
    listener = mouse.Listener(on_move=on_move)
    listener.start()
    activate_button.config(state="disabled")
    deactivate_button.config(state="normal")

def stop_tracking():
    global listener
    if listener:
        listener.stop()
    activate_button.config(state="normal")
    deactivate_button.config(state="disabled")

root = tk.Tk()
root.title("Robot Control")
root.configure(bg='black')

activate_button = tk.Button(root, text="Activate", command=start_tracking, bg='green', fg='black')
activate_button.grid(row=0, column=0, padx=10, pady=10, sticky='w')

deactivate_button = tk.Button(root, text="Deactivate", command=stop_tracking, bg='red', fg='black')
deactivate_button.grid(row=0, column=1, padx=10, pady=10, sticky='w')
deactivate_button.config(state="disabled")

mouse_pos_label = tk.Label(root, text="Mouse Position: (0, 0)", bg='black', fg='white')
mouse_pos_label.grid(row=1, column=0, columnspan=2, padx=10, pady=10, sticky='w')

servo_pos_label = tk.Label(root, text="Servo Positions: (Servo1: 90, Servo2: 90, Servo3: 90)", bg='black', fg='white')
servo_pos_label.grid(row=2, column=0, columnspan=2, padx=10, pady=10, sticky='w')

root.mainloop()