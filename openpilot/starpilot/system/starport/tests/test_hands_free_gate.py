"""2026-11-02 Honda/OTTOCAST, 43 projections: the car grants its screen only once its own hands-free link to the comma is
up; the first Wi-Fi join and the adapter's handshake restart cost the first attempt; cold boots started before
NetworkManager managed the radio. Each piece here is one of those four findings."""

import socket
import threading
import time
from types import SimpleNamespace

import pytest

from openpilot.starpilot.system.starport import bluez_phone, bootstrap as bs, car_profiles, network, supervisor
from openpilot.starpilot.system.starport.tests.test_starport import FakeNetworkManager, credentials, make_lease
from openpilot.starpilot.system.starport.tests.test_stream_focus import Session, Source, Watch

CAR = "02:00:00:00:00:01"
BSSID = "AA:BB:CC:DD:EE:FF"


# ------------------------------------------------------------------ fast first join: the remembered channel

def test_first_join_pins_the_remembered_channel_and_keeps_the_profile_hidden():
  nm = FakeNetworkManager()
  lease = make_lease(nm)
  events = []
  lease.log = lambda name, **values: events.append((name, values))
  saved = {}
  lease.channel_memory = network.ChannelMemory(lambda: {BSSID: 5180}, saved.update)
  assert lease.acquire(credentials(), timeout=2.0) == "192.168.50.23"
  wireless = nm.added[0]["802-11-wireless"]
  assert (wireless["band"], wireless["channel"], wireless["hidden"]) == (("s", "a"), ("u", 36), ("b", True))
  joining = next(values for name, values in events if name == "wifi_joining")
  assert (joining["pinned_freq"], joining["pinned_source"]) == (5180, "remembered")
  assert saved == {BSSID: 5745}  # where the access point actually was is what the next session pins


def test_a_remembered_channel_that_fails_is_forgotten_before_the_retry_scans():
  nm = FakeNetworkManager(fail_with_bssid=True)
  lease = make_lease(nm)
  events = []
  lease.log = lambda name, **values: events.append((name, values))
  memory = {BSSID: 5180}
  lease.channel_memory = network.ChannelMemory(lambda: dict(memory), lambda channels: (memory.clear(), memory.update(channels)))
  assert lease.acquire(credentials(), timeout=2.0) == "192.168.50.23"
  assert "channel" in nm.added[0]["802-11-wireless"] and "bssid" not in nm.added[1]["802-11-wireless"]
  assert memory == {} and [name for name, _ in events if name == "wifi_channel_forgotten"] == ["wifi_channel_forgotten"]


def test_without_a_memory_or_a_sighting_the_join_scans_as_before():
  nm = FakeNetworkManager()
  lease = make_lease(nm)
  events = []
  lease.log = lambda name, **values: events.append((name, values))
  lease.acquire(credentials(), timeout=2.0)
  wireless = nm.added[0]["802-11-wireless"]
  assert "channel" not in wireless and wireless["hidden"] == ("b", True)
  assert next(values for name, values in events if name == "wifi_joining")["pinned_source"] == ""


def test_channel_memory_is_tolerant_and_saves_only_changes():
  saves = []
  memory = network.ChannelMemory(lambda: {BSSID: 5180, "bad": "x"}, saves.append)  # keys are stored upper-case
  assert memory.frequency(BSSID) == 5180 and memory.frequency("bad") == 0 and memory.frequency("none") == 0
  memory.remember(BSSID, 5180)  # unchanged
  memory.remember("none", 0)  # nothing to forget
  assert saves == []
  memory.remember(BSSID, 5200)
  memory.remember(BSSID, 0)
  assert saves == [{BSSID: 5200, "bad": "x"}, {"bad": "x"}]


# ------------------------------------------------------------------ fast first join: the adapter's handshake restart

def _frame(message_id: int, payload: bytes) -> bytes:
  return bs.encode_frame(message_id, payload)


def test_keepalive_hands_a_handshake_restart_to_the_supervisor_during_setup():
  phone, car = socket.socketpair()
  events = []
  boot = bs.WirelessBootstrap(phone, lambda name, **values: events.append(name), stage_timeout=1.0)
  boot.on_restart = lambda: True
  stop = threading.Event()
  thread = threading.Thread(target=boot.keepalive, args=(stop,), daemon=True)
  thread.start()
  car.sendall(_frame(bs.WIFI_VERSION_REQUEST, b"\x08\x01"))
  thread.join(3.0)
  try:
    assert not thread.is_alive() and boot.restarted.is_set() and boot.restart_payload == b"\x08\x01"
    assert not boot.link_closed.is_set() and "bootstrap_restart_during_setup" in events
  finally:
    stop.set()
    phone.close()
    car.close()


def test_keepalive_ignores_a_handshake_restart_once_the_session_is_up():
  phone, car = socket.socketpair()
  events = []
  boot = bs.WirelessBootstrap(phone, lambda name, **values: events.append(name), stage_timeout=1.0)
  stop = threading.Event()
  thread = threading.Thread(target=boot.keepalive, args=(stop,), daemon=True)
  thread.start()
  car.sendall(_frame(bs.WIFI_VERSION_REQUEST, b"\x08\x01"))
  deadline = time.monotonic() + 2.0
  while "bootstrap_ignored" not in events and time.monotonic() < deadline:
    time.sleep(0.01)
  try:
    assert thread.is_alive() and not boot.restarted.is_set() and "bootstrap_ignored" in events
  finally:
    stop.set()
    phone.close()
    car.close()
    thread.join(2.0)


class FakeBoot:
  def __init__(self):
    self.restarted = threading.Event()
    self.link_closed = threading.Event()
    self.restart_payload = b""
    self.queue: list = []
    self.runs = 0
    self.on_restart = lambda: False

  def keepalive(self, stop):
    stop.wait(5.0)

  def run(self, join_wifi, cancelled=None):
    self.runs += 1
    return SimpleNamespace(head_unit={"restarted": self.runs})

  def join_is_cancelled(self):
    return False


def bare_supervisor(events):
  sup = supervisor.Supervisor.__new__(supervisor.Supervisor)
  sup.log = lambda name, **values: events.append((name, values))
  sup._stop = threading.Event()
  sup._status = {"state": "", "detail": "", "last_stage": "authenticating"}
  sup._set = lambda **values: sup._status.update(values)
  sup._lock = threading.RLock()
  sup._sockets = set()
  sup._setup_sock = None
  sup._setup_phase = False
  sup._companion_keeper = None
  return sup


def test_a_handshake_restart_during_setup_is_answered_and_the_connection_made_again():
  events = []
  sup = bare_supervisor(events)
  boot = FakeBoot()
  projections = []

  def project(result, lease, ident, link_lost):
    projections.append(result.head_unit)
    if len(projections) == 1:
      assert boot.on_restart() is True  # still setting up: the keepalive may take the adapter's new request
      boot.restart_payload = b"\x08\x01"
      boot.restarted.set()
      raise EOFError("Head unit disconnected after 0/4 bytes")
    assert sup._setup_phase and not boot.restarted.is_set()

  sup._project = project
  tuning = {**car_profiles.DEFAULTS, "setup_restarts": 2, "tcp_stops_with_rfcomm": True}
  lease = SimpleNamespace(acquire=lambda c, cancelled=None: "192.168.1.102")
  sup._project_with_restarts(boot, SimpleNamespace(head_unit={}), lease, None, tuning)
  assert projections == [{}, {"restarted": 1}] and boot.runs == 1
  assert boot.queue == [(bs.WIFI_VERSION_REQUEST, b"\x08\x01")]  # the new request is answered first, as the first one was
  assert [name for name, _ in events] == ["bootstrap_restarted_during_setup"] and not sup._setup_phase


def test_setup_restarts_stay_off_unless_the_car_profile_asks():
  events = []
  sup = bare_supervisor(events)
  boot = FakeBoot()

  def project(result, lease, ident):  # as every other adapter: no restart handling, no link watch
    assert boot.on_restart() is False
    boot.restarted.set()
    raise EOFError("Head unit disconnected after 0/4 bytes")

  sup._project = project
  with pytest.raises(EOFError):
    sup._project_with_restarts(boot, SimpleNamespace(head_unit={}), None, None, dict(car_profiles.DEFAULTS))
  assert boot.runs == 0 and events == []


def test_a_restart_past_the_setup_phase_is_not_taken():
  events = []
  sup = bare_supervisor(events)
  boot = FakeBoot()

  def project(result, lease, ident, link_lost):
    sup._setup_phase = False  # session.start() finished: the car is projecting
    assert boot.on_restart() is False
    boot.restarted.set()
    raise RuntimeError("Head unit stopped answering for 15 s")

  sup._project = project
  with pytest.raises(RuntimeError):
    sup._project_with_restarts(boot, SimpleNamespace(head_unit={}), None, None, {**car_profiles.DEFAULTS, "setup_restarts": 2})
  assert boot.runs == 0


def test_the_dead_setup_socket_is_shut_down_when_a_restart_is_taken():
  sup = bare_supervisor([])
  boot = FakeBoot()
  a, b = socket.socketpair()
  sup._track(a)

  def project(result, lease, ident, link_lost):
    if result.head_unit:
      return  # the second connection, after the restart
    sup._setup_sock = a
    assert boot.on_restart() is True
    assert b.recv(1) == b""  # the other end sees the shutdown at once
    boot.restarted.set()
    raise EOFError("closed")

  sup._project = project
  lease = SimpleNamespace(acquire=lambda c, cancelled=None: "ip")
  sup._project_with_restarts(boot, SimpleNamespace(head_unit={}), lease, None, {**car_profiles.DEFAULTS, "setup_restarts": 1})
  assert sup._setup_sock is None and a not in sup._sockets and boot.runs == 1
  b.close()


# ------------------------------------------------------------------ the hands-free gate

def test_companion_link_keeper_retries_until_the_link_is_up_then_rests():
  link = {"up": False}
  events = []
  outcomes = iter(["org.bluez.Error.Failed: br-connection-refused", "connected"])

  def connect(timeout):
    outcome = next(outcomes)
    link["up"] = outcome == "connected"
    return outcome

  keeper = supervisor.CompanionLinkKeeper(connect, lambda: link["up"], lambda name, **values: events.append((name, values)),
                                          interval=0.03, timeout=1.0).start()
  deadline = time.monotonic() + 2.0
  while keeper.attempts < 2 and time.monotonic() < deadline:
    time.sleep(0.005)
  time.sleep(0.1)  # with the link up it rests instead of connecting again
  keeper.stop()
  assert keeper.attempts == 2
  assert [values["outcome"] for _, values in events] == ["org.bluez.Error.Failed: br-connection-refused", "connected"]
  assert [values["up"] for _, values in events] == [False, True]


def test_companion_link_keeper_survives_a_connect_that_raises():
  events = []

  def connect(timeout):
    raise RuntimeError("D-Bus went away")

  keeper = supervisor.CompanionLinkKeeper(connect, lambda: False, lambda name, **values: events.append(values), interval=0.02).start()
  deadline = time.monotonic() + 2.0
  while keeper.attempts < 2 and time.monotonic() < deadline:
    time.sleep(0.005)
  keeper.stop()
  assert keeper.attempts >= 2 and events[0]["outcome"] == "RuntimeError: D-Bus went away"


def test_connecting_the_saved_car_follows_its_hands_free_link_not_bluez_connected():
  sup = supervisor.Supervisor.__new__(supervisor.Supervisor)
  sup.config = {"companion_address": CAR}
  links: set = set()
  sup._bluez = SimpleNamespace(hfp_link=lambda address: address in links)
  calls = []

  def prepare(address, selected, *, connect=False, timeout=12.0, link_up=None):
    calls.append((address, connect, timeout, link_up is not None and link_up()))
    links.add(address)

  sup._shared_bluetooth_owner = SimpleNamespace(prepare_companion=prepare)
  assert sup._connect_companion_link(8.0) == "connected" and calls == [(CAR, True, 8.0, False)]
  assert sup._connect_companion_link(8.0) == "up" and len(calls) == 1
  links.clear()
  sup._shared_bluetooth_owner = SimpleNamespace(prepare_companion=lambda *a, **k: (_ for _ in ()).throw(RuntimeError("br-connection-refused")))
  assert sup._connect_companion_link(8.0) == "br-connection-refused"
  sup.config = {"companion_address": ""}
  assert sup._connect_companion_link(8.0) == "no saved car"


def test_the_keeper_runs_only_for_a_profile_with_the_gate_and_a_saved_car():
  sup = supervisor.Supervisor.__new__(supervisor.Supervisor)
  sup.config = {"companion_address": CAR}
  sup._bluez = SimpleNamespace(hfp_link=lambda address: True)
  sup._shared_bluetooth_owner = SimpleNamespace(prepare_companion=lambda *a, **k: None)
  sup.log = lambda *a, **k: None
  sup._companion_keeper = None
  sup._start_companion_keeper(dict(car_profiles.DEFAULTS))
  assert sup._companion_keeper is None
  sup._start_companion_keeper({**car_profiles.DEFAULTS, "hfp_gate": True})
  assert sup._companion_keeper is not None
  sup._stop_companion_keeper()
  assert sup._companion_keeper is None
  sup.config = {"companion_address": ""}
  sup._start_companion_keeper({**car_profiles.DEFAULTS, "hfp_gate": True})
  assert sup._companion_keeper is None


def _stream_with_keeper(monkeypatch, session, link_up, seconds=0.2):
  monkeypatch.setattr(supervisor, "FOCUS_RETRY_FIRST", 10.0)
  monkeypatch.setattr(supervisor.device_health, "sample", lambda interface: {})
  sup = bare_supervisor([])
  sup._stage = lambda *a, **k: None
  sup._native_reconnects, sup._native_reconnect_limit = 0, 0
  sup._companion_link_up = link_up
  stops = []

  def stop_keeper():
    stops.append(1)
    sup._companion_keeper = None

  sup._stop_companion_keeper = stop_keeper
  sup._companion_keeper = SimpleNamespace()
  timer = threading.Timer(seconds, sup._stop.set)
  timer.start()
  try:
    sup._stream(session, type("Encoder", (), {"last_encode_ms": 0.0})(), Source(), Watch(), 1 / 30)
  finally:
    sup._stop.set()
    timer.cancel()
  return sup, stops


def test_the_car_screen_wait_names_the_missing_hands_free_link(monkeypatch):
  sup, stops = _stream_with_keeper(monkeypatch, Session(), lambda: False)
  assert sup._status["detail"] == supervisor.COMPANION_LINK_HINT and stops == []
  sup, stops = _stream_with_keeper(monkeypatch, Session(), lambda: True)
  assert sup._status["detail"] == "" and stops == []


def test_the_keeper_stops_once_the_car_hands_over(monkeypatch):
  session = Session()
  session.focused, session.focus_epoch = True, 1
  sup, stops = _stream_with_keeper(monkeypatch, session, lambda: False)
  assert stops == [1] and sup._status["detail"] == ""


def test_bluez_phone_tracks_open_hands_free_links_per_peer(monkeypatch):
  phone = bluez_phone.BluezPhone.__new__(bluez_phone.BluezPhone)
  phone._peers_lock, phone._hfp_peers, phone._stop = threading.Lock(), {}, threading.Event()
  phone.log = lambda *a, **k: None
  gone = []
  phone.on_disconnection = gone.append
  release = threading.Event()
  monkeypatch.setattr(bluez_phone.hfp, "serve", lambda sock, stop, log, peer: release.wait(2.0))
  phone._hfp_peers["AA:BB"] = 2  # the car reconnected before its first link closed
  threads = [threading.Thread(target=phone._serve_peer, args=(None, "AA:BB", "dev_AA_BB"), daemon=True) for _ in range(2)]
  for thread in threads:
    thread.start()
  assert phone.hfp_link("aa:bb")
  release.set()
  for thread in threads:
    thread.join(2.0)
  assert not phone.hfp_link("AA:BB") and gone == ["AA:BB", "AA:BB"]


# ------------------------------------------------------------------ cold boot: the radio is not managed yet

def test_auto_connect_waits_for_networkmanager_to_manage_the_radio():
  sup = supervisor.Supervisor.__new__(supervisor.Supervisor)
  events = []
  sup.log = lambda name, **values: events.append((name, values))
  sup.config = {"connection": "wireless", "wifi_interface": "wlan0"}
  sup._lease_factory = None
  sup._wifi_probe_at, sup._wifi_ready_state = -10.0, True
  states = iter([10, 20, 30])
  sup._wifi_probe = SimpleNamespace(interface_state=lambda: next(states))
  assert sup._wifi_ready(0.0) is False and sup._wifi_ready(1.0) is False  # within the probe interval: no second ask
  assert sup._wifi_ready(5.0) is False  # unavailable: no supplicant yet
  assert sup._wifi_ready(10.0) is True
  assert [(name, values["state"]) for name, values in events] == [("wifi_radio_waiting", 10), ("wifi_radio_ready", 30)]


def test_the_radio_probe_fails_open():
  sup = supervisor.Supervisor.__new__(supervisor.Supervisor)
  sup.log = lambda name, **values: None
  sup.config = {"connection": "wireless", "wifi_interface": "wlan0"}
  sup._lease_factory = None
  sup._wifi_probe_at, sup._wifi_ready_state = -10.0, False
  sup._wifi_probe = SimpleNamespace(interface_state=lambda: (_ for _ in ()).throw(RuntimeError("no NetworkManager")))
  assert sup._wifi_ready(0.0) is True
  sup.config = {"connection": "wired", "wifi_interface": "wlan0"}
  sup._wifi_probe_at, sup._wifi_ready_state = -10.0, False
  assert sup._wifi_ready(0.0) is True  # wired sessions never wait for the radio


# ------------------------------------------------------------------ cold boot: the keeper owns the link, nothing pages before a session

def _auto_connect_unit(events, profile):
  from openpilot.starpilot.system.starport.tests.test_car_first_restart import fixture
  unit = fixture()
  unit.config["car_profile_override"] = profile
  unit.log = lambda name, **values: events.append((name, values))
  return unit


def test_with_the_gate_ignition_starts_the_adapter_without_waiting_for_the_car():
  # 2026-11-03 cold boot: the start waited 23 s on a page of the car whose service lookup hung; the keeper then
  # brought the link up 32 s into the session and the car handed over a second later.
  events = []
  unit = _auto_connect_unit(events, "ottocast")
  unit._onroad = lambda: True
  unit._shared_bluetooth_owner.prepare_companion.return_value = {"connected": False}
  unit._auto_connect(100)
  assert [call.kwargs for call in unit._shared_bluetooth_owner.prepare_companion.call_args_list] == [{}]  # admitted, never paged
  unit.start.assert_called_once_with(trigger="onroad")
  gate = next(values for name, values in events if name == "auto_connect_gate")
  assert (gate["onroad"], gate["car_link"], gate["ready"]) == (True, False, True)


def test_with_the_gate_a_parked_comma_never_pages_the_car_but_still_answers_it():
  # The 11-02 journal: 218 page timeouts an hour, all day, from the 15 s retry while the car was in the garage.
  events = []
  unit = _auto_connect_unit(events, "ottocast")
  unit._shared_bluetooth_owner.prepare_companion.return_value = {"connected": False}
  for now in (100, 101, 115, 130, 200, 400):
    unit._auto_connect(now)
  assert all(call.kwargs == {} for call in unit._shared_bluetooth_owner.prepare_companion.call_args_list)
  unit.start.assert_not_called()
  assert [name for name, _ in events].count("auto_connect_gate") == 1  # unchanged gate: logged once
  unit._car_seen_at = 405.0  # the car opened hands-free to the comma by itself
  unit._auto_connect(410)
  unit.start.assert_called_once_with(trigger="car_connected")


def test_without_the_gate_the_car_is_still_paged_before_the_adapter():
  events = []
  unit = _auto_connect_unit(events, "default")
  unit._onroad = lambda: True
  unit._shared_bluetooth_owner.prepare_companion.return_value = {"connected": False}
  unit._auto_connect(100)
  assert [call.kwargs for call in unit._shared_bluetooth_owner.prepare_companion.call_args_list] == [{}, {"connect": True}]
  unit.start.assert_not_called()


def test_the_attempt_only_admits_the_car_under_the_gate():
  events = []
  unit = supervisor.Supervisor.__new__(supervisor.Supervisor)
  unit.config = {"companion_address": CAR}
  unit.log = lambda name, **values: events.append((name, values))
  unit._check_cancel = lambda: None
  calls = []
  unit._shared_bluetooth_owner = SimpleNamespace(prepare_companion=lambda address, selected, *, connect: calls.append((address, connect)))
  unit._admit_companion({**car_profiles.DEFAULTS, "hfp_gate": True})
  unit._admit_companion({**car_profiles.DEFAULTS, "hfp_gate": False})
  assert calls == [(CAR, False), (CAR, True)]
  assert [name for name, _ in events] == ["companion_car_admitted", "companion_car_connected"]
  unit._shared_bluetooth_owner = None
  with pytest.raises(RuntimeError):
    unit._admit_companion(car_profiles.DEFAULTS)


def test_events_between_sessions_go_to_a_rolling_daemon_log(tmp_path, monkeypatch):
  import json
  log = supervisor.EventLog(tmp_path)
  log("auto_connect_gate", onroad=False)
  log.open()
  log("session_start")
  log.close()
  daemon = tmp_path / "daemon.jsonl"
  assert [json.loads(line)["event"] for line in daemon.read_text().splitlines()] == ["auto_connect_gate"]
  session = next(path for path in tmp_path.glob("session-*.jsonl"))
  assert [json.loads(line)["event"] for line in session.read_text().splitlines()] == ["session_start"]
  monkeypatch.setattr(supervisor, "DAEMON_LOG_BYTES", 1)
  log("wifi_radio_waiting")
  assert [json.loads(line)["event"] for line in (tmp_path / "daemon.1.jsonl").read_text().splitlines()] == ["auto_connect_gate"]
  assert [json.loads(line)["event"] for line in daemon.read_text().splitlines()] == ["wifi_radio_waiting"]
  assert log.recent[-1]["event"] == "wifi_radio_waiting"
