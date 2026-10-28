"""MKS CAN frames against the worked examples in the SERVO42D/57D CAN manual (V1.0.5)."""
import pytest

from deskarm.mks import BROADCAST, frame, stop_frame, stop_ok


def test_checksum_matches_the_manual_examples():
    assert frame(0x01, 0x30) == (0x01, bytes([0x30, 0x31]))                      # "01 30 CRC" -> 0x31
    assert frame(0x01, 0xF6, bytes(3)) == (0x01, bytes.fromhex("F6000000F7"))    # "01 F6 00 00 00 F7"


def test_stop_frame_is_two_bytes_and_can_be_broadcast():
    assert stop_frame(1) == (1, bytes([0xF7, 0xF8]))
    assert stop_frame(7) == (7, bytes([0xF7, 0xFE]))
    assert stop_frame() == (BROADCAST, bytes([0xF7, 0xF7]))
    assert stop_frame(0x300)[1][-1] == (0x300 + 0xF7) & 0xFF                     # ids above 255 wrap


def test_stop_reply_is_checked():
    assert stop_ok(1, bytes([0xF7, 0x01, 0xF9]))
    assert not stop_ok(1, bytes([0xF7, 0x00, 0xF8]))        # driver says "stop fail"
    assert not stop_ok(1, bytes([0xF7, 0x01, 0x00]))        # bad checksum
    assert not stop_ok(2, bytes([0xF7, 0x01, 0xF9]))        # another driver's reply
    assert not stop_ok(1, bytes([0xF6, 0x01, 0xF8]))        # a different command


def test_bad_can_id_is_rejected():
    with pytest.raises(ValueError):
        frame(2048, 0xF7)
