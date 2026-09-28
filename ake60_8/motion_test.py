"""Safe bring-up test: hold current position with ramped Kp, move +0.5 rad and back, then release.

Aborts (sends zero command) if tracking error > MAX_ERR rad or the motor reports a fault.
"""

import glob
import math
import sys
import time

import can

from ake60_8 import AKE60_8

MOTOR_ID = int(sys.argv[1]) if len(sys.argv) > 1 else 101
KP, KD = 5.0, 0.3
MAX_ERR = 0.6  # rad
RATE = 200.0


def main():
    port = sorted(glob.glob("/dev/cu.usbmodem*"))[0]
    with can.Bus(interface="slcan", channel=port, bitrate=1_000_000) as bus:
        m = AKE60_8(bus, MOTOR_ID)
        st = None
        t0 = time.monotonic()
        while st is None and time.monotonic() - t0 < 1.0:
            st = m.poll(0.1)
        if st is None:
            sys.exit("no status from motor")
        p0 = math.radians(st.pos_deg)
        print(f"start pos {st.pos_deg:.1f} deg = {p0:.3f} rad")

        # (duration s, target function of phase time -> rad, kp function)
        phases = [
            ("ramp Kp, hold", 1.0, lambda s: p0, lambda s: KP * s),
            ("hold", 1.0, lambda s: p0, lambda s: KP),
            ("move +0.5", 1.5, lambda s: p0 + 0.5 * (1 - math.cos(math.pi * s)) / 2, lambda s: KP),
            ("hold", 1.0, lambda s: p0 + 0.5, lambda s: KP),
            ("move back", 1.5, lambda s: p0 + 0.5 * (1 + math.cos(math.pi * s)) / 2, lambda s: KP),
            ("hold", 1.0, lambda s: p0, lambda s: KP),
        ]
        try:
            for name, dur, target, kp in phases:
                print(f"-- {name}")
                start = time.monotonic()
                next_print = 0.0
                while (el := time.monotonic() - start) < dur:
                    s = el / dur  # normalized phase time 0..1
                    tgt = target(s)
                    m.mit(p=tgt, kp=kp(s), kd=KD)
                    st = m.poll()
                    err = math.radians(st.pos_deg) - tgt
                    if el >= next_print:
                        print(f"  tgt={math.degrees(tgt):7.1f}  {st}")
                        next_print = el + 0.25
                    if st.error or abs(err) > MAX_ERR:
                        raise RuntimeError(f"abort: err={err:.3f} rad, fault={st.error}")
                    time.sleep(1.0 / RATE)
        finally:
            m.stop()
            time.sleep(0.05)
            print("released, final:", m.poll(0.1))


if __name__ == "__main__":
    main()
