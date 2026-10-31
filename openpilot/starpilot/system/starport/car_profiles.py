"""Android Auto tuning selected from the saved head-unit fingerprint or an override."""
import re

DEFAULTS = {
  "connect_delay_s": 0.0,       # wait before paging the car over Bluetooth
  "negotiate_delay_s": 0.0,     # wait after authentication before service discovery
  "max_window": 2,
  "phone_services": False,
  "phone_frames": False,
  "native_reconnects": 0,       # a car that never grants the screen: shut down cleanly and reconnect this many times
  "reuse_wifi_join": False,     # the adapter restarted the handshake with the same network: keep the join in progress
  "tcp_stops_with_rfcomm": False,  # the adapter closed RFCOMM before accepting TCP: stop retrying, reconnect instead
}

FINGERPRINT_KEYS = ("head_unit_make", "head_unit_model", "car_make", "car_model", "car_year")

RULES: list[dict] = [
  {"id": "aawireless", "label": "AAWireless", "match": {},
   "tuning": {"max_window": 30, "phone_services": True, "phone_frames": True}},
  # OTTOCAST (Wi-Fi "CAR2-xxxx"), 2020 Honda Accord, 2026-10-28..31 over 19 projections: the car granted the screen only
  # when the dongle answered the first Android Auto message at once (5 of 6), never when it answered ~2 s later (0 of 13),
  # which is every connection made at ignition. The dongle's own session (its "connect via Bluetooth" screen) and a phone
  # are always granted; both offer the car-data and Bluetooth channels we leave closed, so open them too (phone_services,
  # 2026-10-31 experiment). Everything below is the operator's experiment set for that dongle and applies to no other
  # adapter; the standard behaviour stays as it was for AAWireless and "Other".
  {"id": "ottocast", "label": "OTTOCAST", "match": {},
   "tuning": {"negotiate_delay_s": 3.0, "phone_services": True, "native_reconnects": 2, "reuse_wifi_join": True,
              "tcp_stops_with_rfcomm": True}},
]


def profile_ids() -> list[str]:
  return [rule["id"] for rule in RULES]


def valid_override(value: str) -> bool:
  return value in ("", "default", *profile_ids())


def fingerprint_from(head_unit: dict, address: str) -> dict:
  found = {key: str(head_unit[key])[:80] for key in FINGERPRINT_KEYS if head_unit.get(key)}
  return {"address": address.upper(), **found} if found else {}


def _matches(rule: dict, fingerprint: dict) -> bool:
  match = rule.get("match") or {}
  return bool(match) and all(re.search(pattern, str(fingerprint.get(key, "")), re.IGNORECASE) for key, pattern in match.items())


def resolve(config: dict) -> tuple[str, str]:
  """(profile id, source) where source is "override", "detected" or "standard"."""
  override = config.get("car_profile_override", "")
  if override == "default":
    return "default", "override"
  if override in profile_ids():
    return override, "override"
  fingerprint = config.get("car_fingerprint") or {}
  if fingerprint.get("address") == str(config.get("receiver_address", "")).upper():
    for rule in RULES:
      if _matches(rule, fingerprint):
        return rule["id"], "detected"
  return "default", "standard"


def tuning(config: dict) -> dict:
  profile, _ = resolve(config)
  rule = next((item for item in RULES if item["id"] == profile), {})
  return {**DEFAULTS, **{key: value for key, value in (rule.get("tuning") or {}).items() if key in DEFAULTS}}


def summary(config: dict) -> dict:
  """What the UI may show: nothing at all while the standard behavior applies."""
  profile, source = resolve(config)
  rule = next((item for item in RULES if item["id"] == profile), None)
  return {"car_profile": profile, "car_profile_source": source, "car_profile_label": rule["label"] if rule else "",
          "car_profile_ids": profile_ids()}
