# Robotics motors workshop

Hands-on control of robot joint actuators over CAN.

| Folder | What |
|---|---|
| [`ake60_8/`](ake60_8/) | CubeMars AKE60-8 (MIT mode over CAN): Python driver, CLI, web UI with Position / Sine / Gait (ground-contact detection) / Velocity / Torque modes |

Hardware used: CubeMars AKE60-8 KV80, CANine/CANable USB-CAN adapter (slcan firmware), 1 Mbps CAN.

<img width="300" alt="IMG_20260928_195132" src="https://github.com/user-attachments/assets/00279dbe-eed3-4aa7-abd8-06995160d0e8" />
<img width="300" alt="IMG_20260928_193637" src="https://github.com/user-attachments/assets/719ef3cf-312b-4723-b83b-787f9846957e" />


Quick start:
```bash
cd ake60_8
python3 -m venv .venv && .venv/bin/pip install -r requirements.txt
.venv/bin/python -m pytest -q      # hardware-free tests
.venv/bin/python app.py            # web UI on http://127.0.0.1:8060
```

`playbook.md` collects lessons learned while building this.
