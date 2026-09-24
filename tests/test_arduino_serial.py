"""Tests for the Motion Arduino DTR reset and startup handshake."""

from collections import deque
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from arduino_serial import parse_encoder_line, reset_and_wait_for_encoder


class FakeClock:
    def __init__(self):
        self.now = 0.0

    def monotonic(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds


class FakeSerial:
    """Small pySerial stand-in that releases scripted bytes on each DTR pulse."""

    def __init__(self, clock, responses_per_pulse):
        self.clock = clock
        self.responses_per_pulse = [deque(batch) for batch in responses_per_pulse]
        self.active = deque()
        self.pulses = 0
        self._dtr = True

    @property
    def dtr(self):
        return self._dtr

    @dtr.setter
    def dtr(self, value):
        previous = self._dtr
        self._dtr = value
        if previous is False and value is True:
            self.pulses += 1
            index = self.pulses - 1
            if index < len(self.responses_per_pulse):
                self.active.extend(self.responses_per_pulse[index])

    def reset_input_buffer(self):
        self.active.clear()

    def readline(self):
        if self.active:
            return self.active.popleft()
        self.clock.sleep(0.01)
        return b""


class ArduinoSerialStartupTests(unittest.TestCase):
    def setUp(self):
        self.clock = FakeClock()

    def recover(self, responses, timeout=0.03):
        ser = FakeSerial(self.clock, responses)
        outcomes = reset_and_wait_for_encoder(
            ser,
            timeout_seconds=timeout,
            attempts=2,
            dtr_pulse_seconds=0.25,
            monotonic=self.clock.monotonic,
            sleep=self.clock.sleep,
        )
        return ser, outcomes

    def test_accepts_valid_encoder_frame_on_first_reset(self):
        ser, outcomes = self.recover([[b"STATUS:READY\r\n", b"ENCODER:0,0\r\n"]])

        self.assertEqual(len(outcomes), 1)
        self.assertEqual(outcomes[0].frame, (0, 0))
        self.assertEqual(ser.pulses, 1)

    def test_retries_dtr_after_data_without_encoder(self):
        ser, outcomes = self.recover(
            [[b"STATUS:READY\r\n"], [b"ENCODER:-2,17\r\n"]]
        )

        self.assertEqual(len(outcomes), 2)
        self.assertTrue(outcomes[0].received_data)
        self.assertIsNone(outcomes[0].frame)
        self.assertEqual(outcomes[1].frame, (-2, 17))
        self.assertEqual(ser.pulses, 2)

    def test_reports_malformed_or_non_encoder_data(self):
        _, outcomes = self.recover(
            [[b"ENCODER:x,1\r\n"], [b"unrecognized output\r\n"]]
        )

        self.assertEqual(len(outcomes), 2)
        self.assertTrue(all(result.received_data for result in outcomes))
        self.assertTrue(all(result.frame is None for result in outcomes))
        self.assertEqual(outcomes[0].samples, ("ENCODER:x,1",))

    def test_reports_no_data_after_both_resets(self):
        ser, outcomes = self.recover([[], []])

        self.assertEqual(len(outcomes), 2)
        self.assertTrue(all(not result.received_data for result in outcomes))
        self.assertTrue(all(result.frame is None for result in outcomes))
        self.assertEqual(ser.pulses, 2)

    def test_encoder_parser_requires_exactly_two_integer_ticks(self):
        self.assertEqual(parse_encoder_line(b"ENCODER:-10,23\r\n"), (-10, 23))
        self.assertIsNone(parse_encoder_line("ENCODER:10,23,4"))
        self.assertIsNone(parse_encoder_line("ENCODER:left,right"))
        self.assertIsNone(parse_encoder_line("STATUS:READY"))


if __name__ == "__main__":
    unittest.main()
