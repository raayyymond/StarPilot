import io

from openpilot.starpilot.system.starport import bt_trace
from openpilot.starpilot.system.starport.bt_trace import BluetoothTrace, TraceFilter

SAMPLE = """\
= Note: Linux version 4.9 (aarch64)                                      0.000000
> HCI Event: Connect Request (0x04) plen 10                #12 [hci0] 2026-11-09 10:17:29.123456
        Address: 58:B8:58:11:22:E4 (Honda Motor)
        Class: 0x340408
        Link type: ACL (0x01)
< HCI Command: Accept Connection Request (0x01|0x0009) plen 7  #13 [hci0] 2026-11-09 10:17:29.124000
        Address: 58:B8:58:11:22:E4 (Honda Motor)
        Role: Central (0x00)
> HCI Event: Link Key Request (0x17) plen 6                #14 [hci0] 2026-11-09 10:17:29.300000
        Address: 58:B8:58:11:22:E4 (Honda Motor)
< HCI Command: Link Key Request Reply (0x01|0x000b) plen 22  #15 [hci0] 2026-11-09 10:17:29.301000
        Address: 58:B8:58:11:22:E4 (Honda Motor)
        Link key: 00112233445566778899aabbccddeeff
> ACL Data RX: Handle 3804 flags 0x02 dlen 12              #16 [hci0] 2026-11-09 10:17:29.400000
      L2CAP: Connection Request (0x02) ident 1 len 4
        Address: 11:22:33:44:55:66
> HCI Event: Remote Name Req Complete (0x07) plen 255      #17 [hci0] 2026-11-09 10:17:29.500000
        Status: Success (0x00)
        Name: Some Phone
> HCI Event: Disconnect Complete (0x05) plen 4             #18 [hci0] 2026-11-09 10:17:40.000000
        Status: Success (0x00)
        Handle: 3804
        Reason: Remote User Terminated Connection (0x13)
"""


def run(text):
  trace, kept, events = TraceFilter(), [], []
  for line in text.splitlines():
    line_kept, event = trace.feed(line)
    kept += [line_kept] if line_kept is not None else []
    events += [event] if event is not None else []
  last = trace.flush()
  return "\n".join(kept), events + ([last] if last else [])


def test_the_trace_keeps_headers_and_link_details_and_no_secret_or_payload():
  kept, _ = run(SAMPLE)
  assert "Connect Request" in kept and "Disconnect Complete" in kept and "Reason: Remote User Terminated" in kept
  assert "xx:xx:xx:xx:xx:E4" in kept and "58:B8" not in kept, "addresses are masked to the last byte"
  assert "Link key" not in kept and "00112233" not in kept, "key material never reaches the file"
  assert "Some Phone" not in kept and "Class" not in kept, "no device names"
  assert "ACL Data" not in kept and "L2CAP" not in kept and ":66" not in kept, "no payload, nor the lines under it"


def test_link_events_reach_the_session_log_with_their_own_timestamp():
  _, events = run(SAMPLE)
  assert [event["what"] for event in events] == ["HCI Event: Connect Request (0x04)",
                                                 "HCI Command: Accept Connection Request (0x01|0x0009)",
                                                 "HCI Event: Disconnect Complete (0x05)"]
  request, _, gone = events
  assert request == {"what": "HCI Event: Connect Request (0x04)", "direction": "received",
                     "at": "2026-11-09 10:17:29.123456", "address": "xx:xx:xx:xx:xx:E4 (Honda Motor)",
                     "link_type": "ACL (0x01)"}
  assert gone["handle"] == "3804" and gone["reason"].startswith("Remote User Terminated") and gone["at"].endswith("40.000000")


class FakeProcess:
  def __init__(self, text):
    self.stdout = io.StringIO(text)

  def poll(self):
    return 0


def test_a_session_trace_rotates_the_last_one_and_logs_start_events_and_stop(tmp_path):
  (tmp_path / bt_trace.TRACE_NAME).write_text("previous session\n")
  logged, commands = [], []

  def popen(command, **_):
    commands.append(command)
    return FakeProcess(SAMPLE)
  trace = BluetoothTrace(tmp_path, lambda name, **values: logged.append((name, values)), seconds=7, popen=popen,
                         run=lambda *a, **k: commands.append(a[0])).start()
  trace.thread.join(timeout=5)
  trace.stop()
  assert commands == [["sudo", "-n", "stdbuf", "-oL", "timeout", "--signal=INT", "7", "btmon", "-T"]], "it ended alone: no kill"
  assert (tmp_path / "bt_trace.1.txt").read_text() == "previous session\n"
  assert "Disconnect Complete" in (tmp_path / bt_trace.TRACE_NAME).read_text()
  names = [name for name, _ in logged]
  assert names[0] == "bt_trace_started" and names.count("bt_hci") == 3 and names[-1] == "bt_trace_stopped"
  assert logged[-1][1]["events"] == 3


def test_a_comma_without_btmon_only_logs_that_and_carries_on(tmp_path):
  logged = []

  def popen(*_, **__):
    raise FileNotFoundError("sudo")
  trace = BluetoothTrace(tmp_path, lambda name, **values: logged.append(name), popen=popen).start()
  trace.stop()
  assert logged == ["bt_trace_unavailable"]
