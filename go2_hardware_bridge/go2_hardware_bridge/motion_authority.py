"""
Motion-authority gate for the hardware bridge. Pure logic, no ROS imports.

A grant is a JSON string published by the motion-authority guardian:

    {"v":1,"owner":"come_here"|"nav2"|null,"epoch":int,"guardian":str,
     "state":str,"mode":str}

The bridge forwards non-zero Move only while it holds a fresh grant that names
it. Anything malformed, of the wrong version, or of the wrong types is ignored
(it neither grants nor refreshes). With the gate disabled (empty topic) the
legacy behaviour is kept: the bridge is always considered the owner.
"""
from __future__ import annotations

import json
from typing import Any, Dict, Optional, Tuple

REVOKED = "REVOKED"
ACQUIRED = "ACQUIRED"
SUPPORTED_VERSION = 1


def parse_grant(raw: Any) -> Optional[Dict[str, Any]]:
    """Return the validated grant dict, or None when raw is malformed."""
    try:
        if isinstance(raw, (bytes, bytearray)):
            raw = bytes(raw).decode("utf-8")
        if not isinstance(raw, str):
            return None
        data = json.loads(raw)
    except (ValueError, UnicodeDecodeError):
        return None
    if not isinstance(data, dict):
        return None
    v = data.get("v")
    if isinstance(v, bool) or not isinstance(v, int) or v != SUPPORTED_VERSION:
        return None
    owner = data.get("owner")
    if owner is not None and not isinstance(owner, str):
        return None
    epoch = data.get("epoch")
    if isinstance(epoch, bool) or not isinstance(epoch, int):
        return None
    for key in ("guardian", "state", "mode"):
        if not isinstance(data.get(key), str):
            return None
    return {
        "owner": owner,
        "epoch": epoch,
        "guardian": data["guardian"],
        "state": data["state"],
        "mode": data["mode"],
    }


class AuthorityGate:
    def __init__(self, name: str, timeout_s: float, enabled: bool = True) -> None:
        self.name = name
        self.timeout_s = float(timeout_s)
        self.enabled = bool(enabled)
        self._grant: Optional[Dict[str, Any]] = None
        self._rx: Optional[float] = None
        self._was_owned = False
        self._key: Optional[Tuple[int, str]] = None

    def on_grant(self, raw: Any, now: float) -> bool:
        """Record a grant received at monotonic time ``now``. False if ignored."""
        grant = parse_grant(raw)
        if grant is None:
            return False
        self._grant = grant
        self._rx = float(now)
        return True

    def owned(self, now: float) -> bool:
        if not self.enabled:
            return True
        if self._grant is None or self._rx is None:
            return False
        if (now - self._rx) > self.timeout_s:
            return False
        return self._grant["owner"] == self.name

    def update(self, now: float) -> Optional[str]:
        """Edge detector. Call once per control tick."""
        if not self.enabled:
            return None
        owned = self.owned(now)
        event: Optional[str] = None
        if owned and not self._was_owned:
            event = ACQUIRED
        elif not owned and self._was_owned:
            event = REVOKED
        elif owned and self._grant is not None:
            if self._key != (self._grant["epoch"], self._grant["guardian"]):
                event = ACQUIRED
        self._was_owned = owned
        self._key = (
            (self._grant["epoch"], self._grant["guardian"])
            if owned and self._grant is not None
            else None
        )
        return event

    def status(self, now: float) -> Dict[str, Any]:
        g = self._grant or {}
        return {
            "enabled": self.enabled,
            "owned": self.owned(now),
            "owner": g.get("owner"),
            "epoch": g.get("epoch"),
        }
