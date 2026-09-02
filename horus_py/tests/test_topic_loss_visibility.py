"""Message loss must be visible and resistible from Python.

A HORUS ring buffer is drop-oldest. That is the right default for sensor
streams — a 30 Hz vision policy wants the newest state sample, not a backlog —
but the loss it causes left no trace on the Python side:

- `Topic.send()` hardcodes `True`, so a publisher could not tell a delivered
  message from a refused one.
- Nothing exposed `Topic::missed_count()`, so a lapped subscriber could not tell
  that its stream had a hole in it. Every message it received was well-formed
  and recent; a subscriber losing 15 of every 16 samples looked exactly like one
  losing none.

These tests cover the three calls that fixed that: `missed_count()`,
`try_send()`, and `send_blocking()`.
"""

import threading
import time
import uuid

import horus
import pytest
from horus import CmdVel, Topic


# The ring capacity used by the backpressure tests. Small on purpose: filling it
# is the point, and every capacity is rounded up to a power of two.
SMALL = 8


def _endpoint(prefix, unique_test_prefix):
    """A topic name no other test (or run) shares."""
    return f"{unique_test_prefix}_{prefix}"


# ============================================================================
# missed_count — the subscriber can see what it lost
# ============================================================================

class TestMissedCount:
    """Tests for Topic.missed_count() and its stats() entry."""

    def test_missed_count_starts_at_zero(self, unique_test_prefix):
        """A fresh subscriber has missed nothing."""
        topic = Topic(CmdVel, capacity=SMALL,
                      endpoint=_endpoint("missed_fresh", unique_test_prefix))
        assert topic.missed_count() == 0

    def test_stats_reports_missed_count(self, unique_test_prefix):
        """missed_count is in stats(), where an operator will actually look."""
        topic = Topic(CmdVel, capacity=SMALL,
                      endpoint=_endpoint("missed_stats", unique_test_prefix))
        stats = topic.stats()
        assert "missed_count" in stats, (
            "stats() reported send_failures and recv_failures but not the loss "
            "on the subscriber's end, which is the one it cannot otherwise see"
        )
        assert stats["missed_count"] == topic.missed_count()

    def test_missed_count_stays_zero_when_nothing_is_lost(self, unique_test_prefix):
        """A subscriber that keeps up reports no misses.

        Guards the counter against being trivially non-zero: if this passed for
        a lapped subscriber only because everything reports a miss, the number
        would be worthless.
        """
        topic = Topic(CmdVel, capacity=SMALL,
                      endpoint=_endpoint("missed_clean", unique_test_prefix))
        for i in range(SMALL // 2):
            topic.send(CmdVel(float(i), 0.0))
        received = [topic.recv() for _ in range(SMALL // 2)]

        assert all(m is not None for m in received), "the subscriber kept up"
        assert topic.missed_count() == 0
        assert topic.stats()["missed_count"] == 0

    @pytest.mark.slow
    def test_lapped_subscriber_reports_what_it_skipped(self, unique_test_prefix):
        """The 30 Hz-policy-on-a-500 Hz-stream case, compressed.

        Two subscribers, each on its own thread so the backend detector picks a
        broadcast backend rather than the single-consumer ring two same-thread
        handles collapse onto — broadcast is the mode that overwrites instead of
        applying backpressure, so it is where the loss goes unrecorded. Both
        stand still while the publisher runs, which is what makes them lapped.
        """
        endpoint = _endpoint("missed_lapped", unique_test_prefix)
        sent = 20_000

        registered = threading.Barrier(3, timeout=20)
        produced = threading.Event()
        results = {}

        def subscriber(tag):
            sub = Topic(CmdVel, capacity=64, endpoint=endpoint)
            sub.recv()  # lazy registration — this thread is now a subscriber
            registered.wait()
            produced.wait(20.0)
            got = 0
            while sub.recv() is not None:
                got += 1
            results[tag] = (got, sub.missed_count(), sub.stats())

        threads = [threading.Thread(target=subscriber, args=(tag,))
                   for tag in ("stalled", "also_stalled")]
        for thread in threads:
            thread.start()

        pub = Topic(CmdVel, capacity=64, endpoint=endpoint)
        registered.wait()
        for i in range(sent):
            pub.send(CmdVel(float(i), 0.0))
        produced.set()
        for thread in threads:
            thread.join(30)

        assert len(results) == 2, "both subscriber threads should have finished"
        for tag, (got, missed, stats) in results.items():
            assert got < sent, (
                f"{tag} received all {sent} messages — the test is not "
                f"exercising the lapping path it claims to"
            )
            assert missed > 0, (
                f"{tag} received {got} of {sent} messages and reported missing "
                f"none. That is the whole defect: the loss is real and, from "
                f"Python, invisible."
            )
            assert stats["missed_count"] == missed


# ============================================================================
# try_send — the publisher can see a refused send
# ============================================================================

class TestTrySend:
    """Tests for Topic.try_send()."""

    def test_try_send_succeeds_when_there_is_room(self, unique_test_prefix):
        topic = Topic(CmdVel, capacity=SMALL,
                      endpoint=_endpoint("try_room", unique_test_prefix))
        assert topic.try_send(CmdVel(1.5, 0.5)) is True
        got = topic.recv()
        assert got is not None and got.linear == pytest.approx(1.5)

    def test_try_send_reports_a_full_buffer(self, unique_test_prefix):
        """try_send() answers False when the message did not get through.

        This is the difference between a motor command that reached the driver
        and one that did not.
        """
        topic = Topic(CmdVel, capacity=SMALL,
                      endpoint=_endpoint("try_full", unique_test_prefix))

        accepted = 0
        refused = False
        for i in range(SMALL * 8):
            if topic.try_send(CmdVel(float(i), 0.0)):
                accepted += 1
            else:
                refused = True
                break

        assert refused, (
            f"try_send() accepted {accepted} messages into a {SMALL}-slot ring "
            f"and never reported a refusal"
        )
        assert accepted >= 1, "try_send() should have accepted the early sends"

    def test_send_still_reports_success_on_the_same_full_buffer(self, unique_test_prefix):
        """The contrast that makes try_send() worth having.

        send() is fire-and-forget: on a full ring it overwrites and still
        answers True. Pinning that here means a future change that makes send()
        itself lossy-but-honest is a deliberate decision, not a silent one.
        """
        topic = Topic(CmdVel, capacity=SMALL,
                      endpoint=_endpoint("try_contrast", unique_test_prefix))
        while topic.try_send(CmdVel(0.0, 0.0)):
            pass
        assert topic.send(CmdVel(0.0, 0.0)) is True

    def test_try_send_works_on_an_untyped_topic(self, unique_test_prefix):
        """Topic("name") carries MessagePack, and must report refusals too."""
        topic = Topic(_endpoint("try_generic", unique_test_prefix), capacity=SMALL)
        assert topic.try_send({"cmd": "go"}) is True
        assert topic.recv() == {"cmd": "go"}


# ============================================================================
# send_blocking — the publisher can refuse to lose the message
# ============================================================================

class TestSendBlocking:
    """Tests for Topic.send_blocking()."""

    def test_send_blocking_delivers_when_there_is_room(self, unique_test_prefix):
        topic = Topic(CmdVel, capacity=SMALL,
                      endpoint=_endpoint("blk_room", unique_test_prefix))
        topic.send_blocking(CmdVel(2.0, 1.0), 0.5)
        got = topic.recv()
        assert got is not None and got.linear == pytest.approx(2.0)

    def test_send_blocking_raises_when_the_buffer_never_drains(self, unique_test_prefix):
        """A message that was not delivered must raise, not return quietly."""
        endpoint = _endpoint("blk_full", unique_test_prefix)
        topic = Topic(CmdVel, capacity=SMALL, endpoint=endpoint)
        while topic.try_send(CmdVel(0.0, 0.0)):
            pass

        with pytest.raises(horus.HorusTimeoutError) as excinfo:
            topic.send_blocking(CmdVel(9.0, 9.0), 0.02)

        message = str(excinfo.value)
        assert endpoint in message, (
            f"the timeout must name the topic — a robot publishes to dozens: "
            f"{message}"
        )
        assert "send_blocking" in message

    def test_send_blocking_waits_for_the_timeout_it_was_given(self, unique_test_prefix):
        """It applies backpressure rather than failing instantly."""
        topic = Topic(CmdVel, capacity=SMALL,
                      endpoint=_endpoint("blk_waits", unique_test_prefix))
        while topic.try_send(CmdVel(0.0, 0.0)):
            pass

        started = time.monotonic()
        with pytest.raises(horus.HorusTimeoutError):
            topic.send_blocking(CmdVel(0.0, 0.0), 0.05)
        waited = time.monotonic() - started

        assert waited >= 0.04, f"returned after only {waited:.4f}s of a 0.05s wait"

    def test_send_blocking_releases_the_gil_while_it_waits(self, unique_test_prefix):
        """Blocking with the GIL held would freeze every other Python thread.

        On a robot that is the whole process: the control loop, the watchdog
        feeder, and the telemetry thread all stop for the duration of one
        publisher's backpressure.
        """
        topic = Topic(CmdVel, capacity=SMALL,
                      endpoint=_endpoint("blk_gil", unique_test_prefix))
        while topic.try_send(CmdVel(0.0, 0.0)):
            pass

        ticks = [0]
        stop = threading.Event()

        def spin():
            while not stop.is_set():
                ticks[0] += 1
                time.sleep(0.001)

        spinner = threading.Thread(target=spin)
        spinner.start()
        try:
            time.sleep(0.02)  # let the spinner get going
            before = ticks[0]
            with pytest.raises(horus.HorusTimeoutError):
                topic.send_blocking(CmdVel(0.0, 0.0), 0.3)
            during = ticks[0] - before
        finally:
            stop.set()
            spinner.join(5)

        assert during > 10, (
            f"another Python thread advanced only {during} times during a 0.3s "
            f"send_blocking — the GIL was held across the wait"
        )

    @pytest.mark.parametrize("bad", [-1.0, float("nan"), float("inf"), float("-inf")])
    def test_send_blocking_rejects_an_invalid_timeout(self, bad, unique_test_prefix):
        """Duration::from_secs_f64 panics on these, and a panic across the FFI
        boundary aborts the interpreter instead of raising."""
        topic = Topic(CmdVel, capacity=SMALL,
                      endpoint=_endpoint("blk_badtimeout", unique_test_prefix))
        with pytest.raises(ValueError):
            topic.send_blocking(CmdVel(0.0, 0.0), bad)

    def test_send_blocking_works_on_an_untyped_topic(self, unique_test_prefix):
        topic = Topic(_endpoint("blk_generic", unique_test_prefix), capacity=SMALL)
        topic.send_blocking({"cmd": "stop"}, 0.5)
        assert topic.recv() == {"cmd": "stop"}


# ============================================================================
# Pool-backed transports — what is and is not available, said out loud
# ============================================================================

class TestPoolBackedTopics:
    """Image/PointCloud/DepthImage/Tensor travel as pool descriptors.

    horus_core gives the first three a try_send but gives none of them a
    send_blocking. A caller who asks for the missing one should be told which
    call and which type, not handed a downcast failure.
    """

    def test_try_send_works_on_an_image_topic(self, unique_test_prefix):
        topic = Topic(horus.Image, capacity=SMALL,
                      endpoint=_endpoint("pool_img_try", unique_test_prefix))
        assert topic.try_send(horus.Image(64, 48)) is True

    def test_send_blocking_refuses_an_image_topic(self, unique_test_prefix):
        topic = Topic(horus.Image, capacity=SMALL,
                      endpoint=_endpoint("pool_img_blk", unique_test_prefix))
        with pytest.raises(NotImplementedError) as excinfo:
            topic.send_blocking(horus.Image(64, 48), 0.01)
        message = str(excinfo.value)
        assert "send_blocking()" in message and "Image" in message
        assert "missed_count" in message, (
            "the refusal should point at the counter that makes the loss "
            f"visible instead: {message}"
        )

    def test_try_send_refuses_a_tensor_topic(self, unique_test_prefix):
        topic = Topic(horus.Tensor, capacity=SMALL,
                      endpoint=_endpoint("pool_tensor", unique_test_prefix))
        with pytest.raises(NotImplementedError):
            topic.try_send(None)


class TestEndpointHostIsNotSilentlyDropped:
    """An `@host` in an endpoint is discarded, so it must say so.

    `create_topic`/`create_pool_topic` both split on '@', keep the name and throw
    the host away, then build an ordinary local SHM topic — while the
    constructor's docstring advertises ``"topic@host:port"`` as *Direct UDP to
    specific host*. A user driving a second machine got a topic that published
    to shared memory on this one and dropped every cross-machine message in
    silence, and ``stats()["is_network"]`` agreed it was networked because that
    flag was derived from the string containing '@'.
    """

    def test_an_endpoint_host_warns_that_it_is_ignored(self):
        import warnings
        import horus

        with warnings.catch_warnings(record=True) as caught:
            warnings.simplefilter("always")
            horus.Topic(horus.CmdVel, endpoint="ep_warn_cmdvel@192.168.1.5:9000")

        msgs = [str(w.message) for w in caught if issubclass(w.category, RuntimeWarning)]
        assert msgs, "an ignored endpoint host must warn, not pass silently"
        assert "192.168.1.5" in msgs[0], f"the warning must name the host it dropped: {msgs[0]}"
        assert "IGNORED" in msgs[0] or "ignored" in msgs[0]

    def test_a_local_topic_does_not_claim_to_be_networked(self):
        import warnings
        import horus

        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            t = horus.Topic(horus.CmdVel, endpoint="ep_flag_cmdvel@10.0.0.9:9000")

        assert t.stats()["is_network"] is False, (
            "the topic is local shared memory; reporting is_network=True because the "
            "endpoint string contained '@' is the API reporting the very lie that "
            "makes this hard to notice"
        )

    def test_a_plain_endpoint_still_warns_about_nothing(self):
        import warnings
        import horus

        with warnings.catch_warnings(record=True) as caught:
            warnings.simplefilter("always")
            horus.Topic(horus.CmdVel, endpoint="ep_plain_cmdvel")

        assert not [w for w in caught if issubclass(w.category, RuntimeWarning)], \
            "a local endpoint with no host must not warn"
