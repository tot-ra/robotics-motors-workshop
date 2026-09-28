# Robotics motors workshop

Hands-on control of robot joint actuators over CAN.

| Folder | What |
|---|---|
| [`ake60_8/`](ake60_8/) | CubeMars AKE60-8 (MIT mode over CAN): Python driver, CLI, web UI with Position / Sine / Gait (ground-contact detection) / Velocity / Torque modes |

Hardware used: CubeMars AKE60-8 KV80, CANine/CANable USB-CAN adapter (slcan firmware), 1 Mbps CAN.

Quick start:
```bash
cd ake60_8
python3 -m venv .venv && .venv/bin/pip install -r requirements.txt
.venv/bin/python -m pytest -q      # hardware-free tests
.venv/bin/python app.py            # web UI on http://127.0.0.1:8060
```

`playbook.md` collects lessons learned while building this.
