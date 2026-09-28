"""Hardware-free tests: frame packing/unpacking and a virtual-bus round trip."""

import can

from ake60_8 import AKE60_8, KP_MAX, P_MAX, T_MIN, pack_mit, unpack_status


def test_pack_zero_centered():
    d = pack_mit(0, 0, 0, 0, 0)
    assert len(d) == 8
    assert d[0] == 0 and d[1] == 0 and d[2] == 0          # kp=0, kd=0
    # zero maps to the middle of the range (half-step rounding either way is fine)
    assert (d[3] << 8 | d[4]) in (0x7FFF, 0x8000)        # p, 16 bit
    assert (d[5] << 4 | d[6] >> 4) in (0x7FF, 0x800)     # v, 12 bit
    assert ((d[6] & 0xF) << 8 | d[7]) in (0x7FF, 0x800)  # t, 12 bit


def test_pack_limits_and_clamping():
    d = pack_mit(P_MAX * 10, 0, KP_MAX * 10, 0, T_MIN * 10)
    assert (d[3] << 8 | d[4]) == 0xFFFF
    assert (d[0] << 4 | d[1] >> 4) == 0xFFF
    assert ((d[6] & 0xF) << 8 | d[7]) == 0


def test_unpack_status():
    st = unpack_status(bytes([0x03, 0x84, 0xFF, 0x9C, 0x00, 0x64, 30, 0]))
    assert st.pos_deg == 90.0 and st.speed_erpm == -1000.0
    assert abs(st.current_a - 1.0) < 1e-9 and st.temp_c == 30 and st.error == 0


def test_virtual_bus_roundtrip():
    with can.Bus(interface="virtual", channel="t", receive_own_messages=False) as host, \
         can.Bus(interface="virtual", channel="t") as fake_motor:
        m = AKE60_8(host, motor_id=3)
        m.mit(p=1.0, kp=10, kd=1)
        rx = fake_motor.recv(1.0)
        assert rx.is_extended_id and rx.arbitration_id == (8 << 8) | 3
        fake_motor.send(can.Message(arbitration_id=0x2903, is_extended_id=True,
                                    data=bytes([0, 10, 0, 0, 0, 0, 25, 0])))
        st = m.poll(1.0)
        assert st is not None and st.pos_deg == 1.0
