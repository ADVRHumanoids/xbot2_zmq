"""ClockDeltaEstimator on synthetic state streams, with the server clock at several rates."""

import unittest

import numpy as np
import time
from pyxbot.zmq_client import ClockDeltaEstimator

STATE_PERIOD = 1e-3  # server seconds between states
MIN_LATENCY = 3e-4  # client seconds
CLIENT_ORIGIN = 12345.0


def stream(rate, server_origin, count, seed=0):
    """Stamps and receive times of `count` states: the server clock runs at `rate` server
    seconds per client second, and each state arrives after MIN_LATENCY plus jitter."""
    rng = np.random.default_rng(seed)
    stamps = server_origin + STATE_PERIOD * np.arange(count)
    latency = MIN_LATENCY + rng.exponential(2e-4, count)
    latency[::100] = MIN_LATENCY  # some states arrive as fast as possible
    recv = CLIENT_ORIGIN + (stamps - server_origin) / rate + latency
    return stamps, recv, latency


def feed(estimator, stamps, recv):
    for stamp, t in zip(stamps, recv):
        estimator.update(stamp, t)


class ClockDeltaEstimatorTest(unittest.TestCase):

    RATES = (1.0, 0.5, 0.25, 2.0)
    # A robot's Unix time and a simulation starting at zero.
    SERVER_ORIGINS = (1.79e9, 0.0)

    def test_rate_follows_the_server_clock(self):
        for rate in self.RATES:
            for origin in self.SERVER_ORIGINS:
                with self.subTest(rate=rate, origin=origin):
                    estimator = ClockDeltaEstimator()
                    feed(estimator, *stream(rate, origin, 3000)[:2])
                    self.assertAlmostEqual(estimator.rate, rate, delta=1e-3 * rate)

    def test_age_on_arrival_is_the_latency_beyond_the_fastest(self):
        for rate in self.RATES:
            for origin in self.SERVER_ORIGINS:
                with self.subTest(rate=rate, origin=origin):
                    stamps, recv, latency = stream(rate, origin, 3000)
                    estimator = ClockDeltaEstimator()
                    feed(estimator, stamps, recv)
                    for i in range(-50, 0):
                        expected = rate * (latency[i] - MIN_LATENCY)
                        self.assertAlmostEqual(
                            estimator.age(stamps[i], recv[i]), expected, delta=1e-4)

    def test_age_grows_with_the_server_clock(self):
        for rate in self.RATES:
            with self.subTest(rate=rate):
                stamps, recv, latency = stream(rate, 0.0, 3000)
                estimator = ClockDeltaEstimator()
                feed(estimator, stamps, recv)
                on_arrival = estimator.age(stamps[-1], recv[-1])
                later = estimator.age(stamps[-1], recv[-1] + 0.1)
                self.assertAlmostEqual(later - on_arrival, 0.1 * rate, delta=1e-4)

    def test_a_delayed_state_is_reported_old(self):
        for rate in self.RATES:
            with self.subTest(rate=rate):
                stamps, recv, _ = stream(rate, 0.0, 3000)
                estimator = ClockDeltaEstimator()
                feed(estimator, stamps, recv)
                stamp = stamps[-1] + STATE_PERIOD
                arrival = CLIENT_ORIGIN + stamp / rate + MIN_LATENCY + 0.05
                estimator.update(stamp, arrival)
                self.assertAlmostEqual(estimator.age(stamp, arrival), 0.05 * rate, delta=1e-4)

    def test_a_restarted_server_restarts_the_fit(self):
        estimator = ClockDeltaEstimator()
        feed(estimator, *stream(0.5, 100.0, 3000)[:2])
        stamps, recv, latency = stream(0.5, 0.0, 3000, seed=1)
        recv += 10.0
        feed(estimator, stamps, recv)
        self.assertAlmostEqual(estimator.rate, 0.5, delta=5e-4)
        self.assertAlmostEqual(estimator.age(stamps[-1], recv[-1]),
                               0.5 * (latency[-1] - MIN_LATENCY), delta=1e-4)

    def test_server_time_tracks_the_server_clock(self):
        for rate in self.RATES:
            for origin in self.SERVER_ORIGINS:
                with self.subTest(rate=rate, origin=origin):
                    stamps, recv, latency = stream(rate, origin, 3000)
                    estimator = ClockDeltaEstimator()
                    feed(estimator, stamps, recv)
                    # The server clock when a state arrives, less the minimum latency.
                    for i in range(-50, 0):
                        expected = stamps[i] + rate * (latency[i] - MIN_LATENCY)
                        self.assertAlmostEqual(estimator.server_time(recv[i]), expected, delta=1e-4)
                        self.assertEqual(estimator.age(stamps[i], recv[i]),
                                         estimator.server_time(recv[i]) - stamps[i])

    def test_server_time_is_nan_before_the_first_state(self):
        self.assertTrue(np.isnan(ClockDeltaEstimator().server_time(CLIENT_ORIGIN)))

    def test_first_state_has_zero_age_on_arrival(self):
        estimator = ClockDeltaEstimator()
        estimator.update(1.79e9, CLIENT_ORIGIN)
        self.assertEqual(estimator.age(1.79e9, CLIENT_ORIGIN), 0.0)

    def test_sleep_server_time(self):
        for rate in self.RATES:
            for origin in self.SERVER_ORIGINS:
                with self.subTest(rate=rate, origin=origin):
                    stamps, recv, latency = stream(rate, origin, 3000)
                    estimator = ClockDeltaEstimator()
                    feed(estimator, stamps, recv)
                    # The server clock when a state arrives, less the minimum latency.
                    durations = [0.001,0.01,0.1,1.0]
                    for duration in durations:
                        t0 = time.monotonic()
                        estimator.sleep_server_time(duration)
                        t1 = time.monotonic()
                        self.assertAlmostEqual(t1 - t0, duration / rate, delta=1e-3)

if __name__ == "__main__":
    unittest.main()
