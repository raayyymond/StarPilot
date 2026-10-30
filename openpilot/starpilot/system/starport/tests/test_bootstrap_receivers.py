"""Wireless-adapter tolerance in the RFCOMM bootstrap, without changing what a built-in head unit sees."""

import threading
import time
from unittest.mock import Mock

import pytest

from openpilot.starpilot.system.starport import bootstrap as bs
from openpilot.starpilot.system.starport.wire import field

ENDPOINT = field(1, "192.168.50.1") + field(2, 5288)
INFO = field(1, "AAWireless") + field(2, "secret-key") + field(3, "AA:BB:CC:DD:EE:FF") + field(4, 8) + field(5, 1)
SETUP_INFO = field(4, ENDPOINT) + field(5, field(1, "AAWireless") + field(3, "secret-key") + field(4, 8))


def scripted(frames, *, ping_interval=bs.JOIN_PING_INTERVAL):
  """A bootstrap whose car sends `frames` in order, then stays silent; records what the comma sent."""
  boot = bs.WirelessBootstrap(None, lambda *a, **k: None, stage_timeout=2, join_ping_interval=ping_interval)
  sent = []
  boot.send = Mock(side_effect=lambda message_id, payload=b"": sent.append(message_id))
  queue = list(frames)

  def receive(timeout):
    if boot.queue:  # frames the bootstrap put back, as the real next_frame reads them first
      return boot.queue.pop(0)
    if queue:
      item = queue.pop(0)
      if isinstance(item, BaseException):
        raise item
      return item
    time.sleep(min(timeout, 0.02))
    raise bs.BootstrapTimeout(boot.stage, "head unit did not answer in time")

  boot.next_frame = Mock(side_effect=receive)
  return boot, sent


def slow_join(seconds, joined):
  def join(credentials):
    time.sleep(seconds)
    joined.append(credentials)
  return join


def test_network_sent_before_the_endpoint_is_kept():
  boot, sent = scripted([(bs.WIFI_INFO_RESPONSE, INFO), (bs.WIFI_START_REQUEST, ENDPOINT)])
  joined = []
  result = boot.run(slow_join(0, joined))
  assert result.endpoint.port == 5288 and joined[0].ssid == "AAWireless"
  assert bs.WIFI_INFO_REQUEST not in sent
  assert sent[-2:] == [bs.WIFI_START_RESPONSE, bs.WIFI_CONNECT_STATUS]


def test_setup_info_answers_the_network_request():
  boot, sent = scripted([(bs.WIFI_START_REQUEST, ENDPOINT), (bs.WIFI_SETUP_INFO, SETUP_INFO)])
  joined = []
  boot.run(slow_join(0, joined))
  assert bs.WIFI_INFO_REQUEST in sent and joined[0].ssid == "AAWireless" and joined[0].key == "secret-key"


def test_no_join_pings_to_a_receiver_that_never_pings():
  # Built-in head units (2025 Civic) see exactly what they saw before: no unsolicited pings.
  boot, sent = scripted([(bs.WIFI_START_REQUEST, ENDPOINT), (bs.WIFI_INFO_RESPONSE, INFO)], ping_interval=0.05)
  boot.run(slow_join(0.4, []))
  assert bs.WIFI_PING_REQUEST not in sent


def test_join_pings_to_a_receiver_that_pings():
  boot, sent = scripted([(bs.WIFI_PING_REQUEST, field(1, 1)), (bs.WIFI_START_REQUEST, ENDPOINT),
                         (bs.WIFI_INFO_RESPONSE, INFO)], ping_interval=0.05)
  boot.run(slow_join(0.4, []))
  assert sent.count(bs.WIFI_PING_REQUEST) >= 3
  assert sent[-1] == bs.WIFI_CONNECT_STATUS


def test_a_failed_handshake_stops_the_join_before_returning():
  boot, sent = scripted([(bs.WIFI_START_REQUEST, ENDPOINT), (bs.WIFI_INFO_RESPONSE, INFO),
                         bs.BootstrapError("joining_wifi", "head unit closed the RFCOMM connection")])
  join_started, join_stopped = threading.Event(), threading.Event()

  def join(credentials):
    join_started.set()
    while not boot.join_is_cancelled():
      time.sleep(0.01)
    join_stopped.set()
    raise RuntimeError("cancelled")

  with pytest.raises(bs.BootstrapError, match="closed"):
    boot.run(join)
  assert join_started.is_set() and join_stopped.is_set(), "the join worker must not outlive the failed handshake"
  assert bs.WIFI_CONNECT_STATUS not in sent


def test_stop_during_the_join_is_not_reported_as_connected():
  stop = threading.Event()
  boot, sent = scripted([(bs.WIFI_START_REQUEST, ENDPOINT), (bs.WIFI_INFO_RESPONSE, INFO)])

  def join(credentials):
    stop.set()  # the user stops just as the join completes

  with pytest.raises(bs.BootstrapError, match="cancelled"):
    boot.run(join, cancelled=stop.is_set)
  assert bs.WIFI_CONNECT_STATUS not in sent


def test_adapter_restarting_during_the_join_is_answered_on_the_same_link():
  # 2026-10-29 OTTOCAST: ~12 s after WifiStartResponse the adapter asks for the version again, then resets RFCOMM.
  restart = [(bs.WIFI_VERSION_REQUEST, field(1, 1) + field(2, 7)), (bs.WIFI_START_REQUEST, ENDPOINT), (bs.WIFI_INFO_RESPONSE, INFO)]
  boot, sent = scripted([(bs.WIFI_START_REQUEST, ENDPOINT), (bs.WIFI_INFO_RESPONSE, INFO)])
  arrived = []
  scripted_receive = boot.next_frame.side_effect
  boot.next_frame.side_effect = lambda timeout: arrived.pop(0) if arrived and not boot.queue else scripted_receive(timeout)
  joins, first_join_stopped = [], threading.Event()

  def join(credentials):
    joins.append(credentials)
    if len(joins) == 1:
      arrived.extend(restart)  # the adapter gives up while we are still joining
      while not boot.join_is_cancelled():
        time.sleep(0.01)
      first_join_stopped.set()
      raise RuntimeError("cancelled")

  result = boot.run(join)
  assert first_join_stopped.is_set() and len(joins) == 2 and result.version == (1, 7)
  assert sent.count(bs.WIFI_VERSION_RESPONSE) == 1 and sent.count(bs.WIFI_START_RESPONSE) == 2
  assert sent[-1] == bs.WIFI_CONNECT_STATUS and sent.count(bs.WIFI_CONNECT_STATUS) == 1  # no failure reported for the first join


def test_an_adapter_that_keeps_restarting_fails_the_attempt():
  restart = (bs.WIFI_VERSION_REQUEST, field(1, 1) + field(2, 7))
  boot, sent = scripted([(bs.WIFI_START_REQUEST, ENDPOINT), (bs.WIFI_INFO_RESPONSE, INFO)] +
                        [restart, (bs.WIFI_START_REQUEST, ENDPOINT), (bs.WIFI_INFO_RESPONSE, INFO)] *
                        (bs.MAX_HANDSHAKE_RESTARTS + 1))

  def join(credentials):
    while not boot.join_is_cancelled():
      time.sleep(0.01)
    raise RuntimeError("cancelled")

  with pytest.raises(bs.BootstrapError, match="kept restarting"):
    boot.run(join)


def test_keepalive_reports_the_head_unit_closing_rfcomm():
  boot, _ = scripted([ConnectionResetError(104, "Connection reset by peer")])
  boot.keepalive(threading.Event())
  assert boot.link_closed.is_set()


def test_keepalive_stopped_by_the_session_is_not_a_closed_link():
  boot, _ = scripted([])
  stop = threading.Event()
  stop.set()
  boot.keepalive(stop)
  assert not boot.link_closed.is_set()
