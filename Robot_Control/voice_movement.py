from __future__ import annotations
import argparse, json, queue, sys, threading, time
from pathlib import Path
import sounddevice as sd
from vosk import KaldiRecognizer, Model

COMMAND_WORDS = {
    "left", "right", "up", "down", "stop",
    "open", "release", "close", "grab",
    "reset", "hello",
}

SERVO_MIN, SERVO_MAX = 10, 170
STEP_DEG = 2
TICK_HZ = 8

WAVE_DURATION = 2.0
WAVE_HZ = 3
WAVE_AMPL = 20

# Local mirror of arm position, just so the jog/reset/wave math has something
# to work from. The actual send + wire format live in Robot_Control.py -
# this module used to pack its own struct.pack('HHHHH', ...) 5-servo packet
# with a fake claw-servo pair, which never matched Robot_Control.ino (3
# servos, 255-framed). Now it only ever calls _apply(), which is either
# Robot_Control.py's shared apply_servo_update() or the standalone fallback
# below when this file is run directly.
_wrist = 90
_elbow = 90
_claw_closed_pos = 60
_claw_open_pos = 160

_dir_x = _dir_y = 0
_move_evt = threading.Event()
_mover_thr: threading.Thread | None = None
_wave_thr: threading.Thread | None = None

_apply = None  # apply_servo_update(s1=None, s2=None, s3=None)


def set_controller(apply_servo_update, claw_closed=60, claw_open=160):
    """Called once by Robot_Control.py to wire voice commands into the shared serial link."""
    global _apply, _claw_closed_pos, _claw_open_pos
    _apply = apply_servo_update
    _claw_closed_pos = claw_closed
    _claw_open_pos = claw_open


def _clamp(v) -> int:
    return max(SERVO_MIN, min(SERVO_MAX, int(v)))


def _push(s1=None, s2=None, s3=None):
    global _wrist, _elbow
    if s1 is not None:
        _wrist = _clamp(s1)
        s1 = _wrist
    if s3 is not None:
        _elbow = _clamp(s3)
        s3 = _elbow
    if _apply:
        _apply(s1=s1, s2=s2, s3=s3)


def _mover_loop():
    global _dir_x, _dir_y
    tick = 1 / TICK_HZ
    while _move_evt.is_set():
        if not (_dir_x or _dir_y):
            _move_evt.clear()
            break
        if _dir_x:
            _push(s1=_wrist + STEP_DEG * _dir_x)
        if _dir_y:
            _push(s3=_elbow + STEP_DEG * _dir_y)
        time.sleep(tick)


def _wrist_wave_loop(origin: int):
    half_period = 1 / (2 * WAVE_HZ)
    end_time = time.time() + WAVE_DURATION
    direction = 1

    while time.time() < end_time:
        _push(s1=_clamp(origin + direction * WAVE_AMPL))
        direction *= -1
        time.sleep(half_period)

    _push(s1=origin)


def handle_command(words: list[str]):
    global _dir_x, _dir_y, _wrist, _elbow, _mover_thr, _wave_thr

    cmds = [w.lower() for w in words if w.lower() in COMMAND_WORDS]
    if not cmds:
        return

    for w in cmds:
        if w == "left":
            _dir_x = -1
        elif w == "right":
            _dir_x = 1
        elif w == "up":
            _dir_y = -1
        elif w == "down":
            _dir_y = 1
        elif w == "stop":
            _dir_x = _dir_y = 0
        elif w in {"open", "release"}:
            _push(s2=_claw_open_pos)
        elif w in {"close", "grab"}:
            _push(s2=_claw_closed_pos)
        elif w == "reset":
            _dir_x = _dir_y = 0
            _push(s1=90, s2=_claw_open_pos, s3=90)
        elif w == "hello":
            _dir_x = _dir_y = 0
            if not _wave_thr or not _wave_thr.is_alive():
                origin = _wrist
                _wave_thr = threading.Thread(target=_wrist_wave_loop, args=(origin,), daemon=True)
                _wave_thr.start()

    if (_dir_x or _dir_y) and not _move_evt.is_set():
        _move_evt.set()
        _mover_thr = threading.Thread(target=_mover_loop, daemon=True)
        _mover_thr.start()


# --- standalone fallback -----------------------------------------------
# Only used when this file is run directly (`python voice_movement.py
# <model_dir>`) rather than imported by Robot_Control.py. Uses the correct
# <BHHH> 255-framed packet that Robot_Control.ino actually expects.
def _standalone_controller():
    import struct
    import serial
    import serial.tools.list_ports as list_ports

    state = {"ser": None}

    def find_arduino():
        for p in list_ports.comports():
            if "Arduino" in p.description or "usbmodem" in p.device:
                return p.device
        return None

    def open_serial():
        port = find_arduino()
        if port:
            try:
                state["ser"] = serial.Serial(port, 9600, timeout=1)
                time.sleep(2)
            except Exception as e:
                print("[Serial]", e, file=sys.stderr)
        else:
            print("[Serial] Arduino not found", file=sys.stderr)

    open_serial()

    def apply_servo_update(s1=None, s2=None, s3=None):
        nonlocal_wrist = s1 if s1 is not None else _wrist
        nonlocal_claw = s2 if s2 is not None else _claw_closed_pos
        nonlocal_elbow = s3 if s3 is not None else _elbow
        pkt = struct.pack('<BHHH', 255, _clamp(nonlocal_wrist), max(0, min(180, int(nonlocal_claw))), _clamp(nonlocal_elbow))
        if state["ser"]:
            try:
                state["ser"].write(pkt)
            except serial.SerialException:
                open_serial()
        else:
            open_serial()

    return apply_servo_update


def _voice_loop(model_dir: Path, voice_evt: threading.Event):
    mdl = Model(str(model_dir))
    rec = KaldiRecognizer(mdl, 16_000)
    q: queue.Queue[bytes] = queue.Queue()

    def cb(indata, frames, t, status):
        if status:
            print(status, file=sys.stderr)
        q.put(bytes(indata))

    print(f"[Voice] say {' / '.join(sorted(COMMAND_WORDS))}")
    with sd.RawInputStream(samplerate=16_000, blocksize=8_000, dtype="int16", channels=1, callback=cb):
        while voice_evt.is_set():
            data = q.get()
            if rec.AcceptWaveform(data):
                txt = json.loads(rec.Result()).get("text", "")
                words = [w for w in txt.split() if w in COMMAND_WORDS]
                if words:
                    print(">>", " ".join(words))
                    handle_command(words)


def main():
    ap = argparse.ArgumentParser("Continuous voice jog controller")
    ap.add_argument("model", type=Path, help="Path to Vosk model directory")
    args = ap.parse_args()
    if not args.model.is_dir():
        sys.exit("Model directory not found")

    set_controller(_standalone_controller())

    voice_evt = threading.Event()
    voice_evt.set()
    t = threading.Thread(target=_voice_loop, args=(args.model, voice_evt), daemon=True)
    t.start()

    try:
        while t.is_alive():
            time.sleep(0.2)
    except KeyboardInterrupt:
        print("\n[stopped]")
        voice_evt.clear()
        t.join()


if __name__ == "__main__":
    main()