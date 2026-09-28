# CubeMars AKE60-8 control (MIT mode over CAN)

Single-file Python controller for the CubeMars AKE60-8 KV80 using `python-can`.
Protocol: CubeMars AK Series manual Ver.3.0.1, sections 4.2-4.3.

## Setup
1. In the CubeMars upper computer (via R-Link): note the motor CAN ID, CAN rate = 1 Mbps, enable CAN feedback (e.g. 100-500 Hz).
2. Wire CAN-H/CAN-L to a USB-CAN adapter (120 Ohm termination on both bus ends), power 24/48 V.
3. `pip install -r requirements.txt`

## Run
```bash
# Default: CANine / CANable (slcan firmware) over USB, port auto-detected (/dev/cu.usbmodem*)
python ake60_8.py --id 1 monitor                 # read status only, sends nothing
python ake60_8.py --id 1 zero                    # current position = 0
python ake60_8.py --id 1 pos 1.57 --kp 10 --kd 0.5
python ake60_8.py --id 1 vel 3 --kd 1            # rad/s
python ake60_8.py --id 1 torque 0.5              # Nm, unloaded motor will spin up fast!

# Linux SocketCAN
sudo ip link set can0 up type can bitrate 1000000
python ake60_8.py --interface socketcan --channel can0 --id 1 monitor
```
Ctrl+C sends a zero command (motor goes limp). Commands are resent at 200 Hz.

## Web UI
```bash
.venv/bin/python app.py        # then open http://127.0.0.1:8060
```
- Modes: Position (deg, Kp/Kd, max speed slew), Sine, Velocity (rad/s, own Kd), Torque (Nm).
- **Sine** emulates an arm swinging: `center + A*sin(2*pi*f*t)` with velocity feed-forward.
  Presets: Slow swing (45 deg, 0.25 Hz), Reach (60 deg, 0.5 Hz), Wave (20 deg, 1 Hz), Shake (5 deg, 3 Hz).
  Start/Stop ramps the amplitude (30 deg/s), frequency glides at 0.5 Hz/s, and the amplitude is capped so
  peak speed `A*2*pi*f` stays under 720 deg/s. The panel shows live tracking error (RMS / peak).
- **Gait** = Sine + ground-contact detection (emulates a leg hitting the ground):
  1. *Learning* (3 free cycles): per-phase baseline of current and tracking lag (24 bins) - keep the arm free.
  2. *Free*: contact = current above baseline in the direction of motion (> `Current, A`) or extra lag
     (> `Lag, deg`) on 2 consecutive status frames.
  3. *Contact*: phase speed x`Speed` (0.25), Kp x`Stiffness` (0.3), setpoint may not go more than
     `Push, deg` past the contact point - so the joint slows down and stops pressing.
  4. *Release*: when the sine turns around, speed/stiffness glide back to normal.
  Changing amplitude/frequency/center relearns the baseline. Contacts are shaded red on the position plot.
  Detection latency ~ 2 status frames: raise the CAN feedback rate (upper computer, up to 500 Hz) to react faster.
  In simulation (test_app.py) it cuts the push into a stiff ground from 3.95 Nm (Sine) to 1.44 Nm.
- Enable holds the current position; STOP button or **Space** releases the motor; arrows jog 5 deg.
- Live readouts and plots (position vs setpoint, current, speed).
- Safety runs server-side at 200 Hz: motor is released if the UI stops polling for 1 s,
  status frames stop for 0.3 s, or the motor reports a fault. Mode switches reset setpoints.

## Limits (AKE60-8)
| P rad | V rad/s | T Nm | Kp | Kd |
|---|---|---|---|---|
| +-12.56 | +-40 | +-15 | 0-500 | 0-5 |

Start with low gains (Kp 5-20, Kd 0.3-1) and no load attached.

## Tests (no hardware)
`python -m pytest -q`
