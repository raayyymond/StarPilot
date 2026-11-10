"""What the Bluetooth controller did in the first minutes of a session, without its secrets.

2026-11-09: after a comma reboot the Honda took 60 s to bring up the hands-free link that grants the screen, and
nothing on the comma recorded what the controller was doing or when the phone connected. This runs btmon for the
start of each session, keeps only the command and event headers with their address, handle, status and reason lines
(no payloads, no keys, no names), masks addresses to their last byte, and writes them to bt_trace.txt beside the
session logs. Link-level connects and disconnects also go into the session log, on the same timeline as
hfp_connected and video_focus.
"""
import re
import subprocess
import threading
from pathlib import Path

from openpilot.starpilot.system.starport.system_snapshot import redact

TRACE_NAME = "bt_trace.txt"
TRACE_SECONDS = 300
TRACE_MAX_BYTES = 2_000_000
COMMAND = ("sudo", "-n", "stdbuf", "-oL", "timeout", "--signal=INT", "{seconds}", "btmon", "-T")
STOP_COMMAND = ("sudo", "-n", "pkill", "-INT", "-x", "btmon")
HEADER = re.compile(r"^[<>@=] ")
PAYLOAD = ("ACL Data", "SCO Data", "ISO Data")
DETAIL = re.compile(r"^\s+(Address|Peer address|BD_ADDR|LE Address|Handle|Status|Reason|Link type|Role|Error)\b")
LINK_EVENTS = ("Connect Request", "Connect Complete", "Connection Complete", "Disconnect Complete",
               "Disconnection Complete", "Create Connection", "Accept Connection", "Reject Connection",
               "HCI Command: Disconnect", "Device Connected", "Device Disconnected", "Connect Failed")
STAMP = re.compile(r"(\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}\.\d+)\s*$")
TRAILER = re.compile(r"\s+(plen \d+|\{0x[0-9a-fA-F]+\}|#\d+|\[hci\d+\]).*$")
FIELD = re.compile(r"^\s+([A-Za-z_ ]+?):\s*(.*)$")


class TraceFilter:
  """Line by line: the kept, masked text, and one summary per link-level event once its details are in."""

  def __init__(self):
    self.keeping = False
    self.event: dict | None = None

  def feed(self, line: str) -> tuple[str | None, dict | None]:
    line = line.rstrip("\r\n")
    if HEADER.match(line):
      finished, self.event = self.event, None
      self.keeping = not any(word in line for word in PAYLOAD)
      if not self.keeping:
        return None, finished
      if any(word in line for word in LINK_EVENTS):
        stamp = STAMP.search(line)
        self.event = {"what": TRAILER.sub("", line[2:]).strip()[:80],
                      "direction": {"<": "sent", ">": "received"}.get(line[0], "host"),
                      "at": stamp.group(1) if stamp else ""}
      return redact(line.encode()).decode(), finished
    if self.keeping and DETAIL.match(line):
      kept = redact(line.encode()).decode()
      field = FIELD.match(kept)
      if self.event is not None and field:
        self.event.setdefault(field.group(1).strip().lower().replace(" ", "_"), field.group(2).strip()[:60])
      return kept, None
    return None, None

  def flush(self) -> dict | None:
    finished, self.event = self.event, None
    return finished


class BluetoothTrace:
  def __init__(self, directory: Path, log, *, seconds: int = TRACE_SECONDS, popen=subprocess.Popen, run=subprocess.run):
    self.path = Path(directory) / TRACE_NAME
    self.log, self.seconds, self.popen, self.run = log, seconds, popen, run
    self.process = None
    self.thread: threading.Thread | None = None

  def start(self) -> "BluetoothTrace":
    self.stop()
    try:
      if self.path.exists():
        self.path.replace(self.path.with_name(self.path.stem + ".1" + self.path.suffix))
      command = [part.format(seconds=self.seconds) for part in COMMAND]
      self.process = self.popen(command, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, stdin=subprocess.DEVNULL,
                                text=True, errors="replace")
    except (OSError, ValueError) as error:
      self.process = None
      self.log("bt_trace_unavailable", error=str(error)[:120])
      return self
    self.thread = threading.Thread(target=self._pump, args=(self.process,), name="starport-bt-trace", daemon=True)
    self.thread.start()
    self.log("bt_trace_started", seconds=self.seconds)
    return self

  def _pump(self, process) -> None:
    trace, written, events = TraceFilter(), 0, 0
    try:
      with open(self.path, "w", encoding="utf-8") as out:
        for line in process.stdout:
          kept, event = trace.feed(line)
          if event is not None:
            events += 1
            self.log("bt_hci", **event)
          if kept is not None and written < TRACE_MAX_BYTES:
            out.write(kept + "\n")
            out.flush()
            written += len(kept) + 1
        event = trace.flush()
        if event is not None:
          events += 1
          self.log("bt_hci", **event)
    except (OSError, ValueError) as error:
      self.log("bt_trace_failed", error=str(error)[:120])
    self.log("bt_trace_stopped", bytes=written, events=events)

  def stop(self) -> None:
    process, self.process = self.process, None
    thread, self.thread = self.thread, None
    if process is None:
      return
    try:
      if process.poll() is None:
        self.run(list(STOP_COMMAND), stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=5, check=False)
    except (OSError, subprocess.SubprocessError):
      pass
    if thread is not None:
      thread.join(timeout=3)
