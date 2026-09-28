# Playbook - lessons learned

- Python: macOS system python3 is 3.9 - `X | None` type hints need `from __future__ import annotations`.
- Fixed-point packing (float_to_uint): use round() or tolerant tests; float math can give 65534.999 at the range limit and int() truncates it.
- CubeMars AK V3 / AKE firmware MIT frame is Kp,Kd,P,V,T with extended ID (8<<8)|id - not the old AK V1/V2 P,V,Kp,Kd,T standard-ID layout.
- Workshop hardware: CANine adapter = CANable slcan firmware (VID 0xAD50), /dev/cu.usbmodem*, python-can interface "slcan" (needs pyserial).
- AKE60-8 on the bench has CAN ID 101 (0x65), not factory 1. Status frames 0x29<id> ~50 Hz, plus 4-byte 0x2C<id> heartbeat. Sniff the bus passively first to discover IDs.
- If the bus is silent, first check that the motor is powered before debugging code.
- macOS: time.monotonic() starts near 0 per process - never use "last_time = 0.0" alone as "never happened"; check a None flag too.
- Background servers: `run_in_background` + later pkill by pattern may miss the real python PID; use `exec` in the command and check `lsof -i :PORT` before restarting.
- Headless Chrome screenshot of a page that polls forever: --virtual-time-budget never finishes; kill it after a timeout, the PNG is already written.
- pytest collects *_test.py too - name hardware scripts differently or set python_files = test_*.py.
- MIT velocity mode is a pure Kd loop: Kd=0.5 only reached ~50% of target speed on AKE60-8, Kd=2 reached ~93%.
- Before starting a new server, always check `lsof -t -iTCP:PORT -sTCP:LISTEN` - servers from previous sessions survive and silently answer with old code.
- Moving hardware: ask before each new motion test; if the question times out, don't move - prepare the test and report.
- Physics sims in tests: compute stiff contact forces inside every substep, not once per outer step - otherwise the sim "bounces" and reports fake huge forces.
