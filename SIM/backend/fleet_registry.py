"""
SwarmTech3_UpRising: Fleet Enrollment Registry & Hardware Identity Bubble (fleet_registry.py)
Implements 3-tier airframe hardware identity:
  1. 64-bit MCU Hardware Silicon UID (immutable hex string, e.g. 0x3842_A10F_9B20_0001)
  2. Tactical Tail Number / Call-Sign (e.g. ALITHEIA-V01 .. V08, reserves ALITHEIA-RES-01 ..)
  3. Dynamic Swarm Slot & Network System ID (sysid = drone.id + 1)
Maintains active registry, standby reserves, and permanent casualty logs.
"""

from __future__ import annotations
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Any, Union
import numpy as np


@dataclass
class AirframeRecord:
    """Persistent identity and enrollment record for an individual airframe."""
    hardware_uid: str                 # 64-bit Hex string (e.g. "3842A10F9B200001")
    tail_number: str                  # Alphanumeric call-sign (e.g. "ALITHEIA-V01")
    sysid: Optional[int] = None       # MAVLink system ID (e.g. 1, 2, ...)
    drone_id: Optional[int] = None    # Swarm numerical ID (0, 1, ...)
    pad_id: Optional[int] = None      # Assigned launch/recovery pad (0..7)
    status: str = "ACTIVE"            # "ACTIVE", "CASUALTY", "RESERVE", "STANDBY"
    registered_at: float = 0.0        # Sim time of enrollment
    lkp: Optional[List[float]] = None # Last known position coordinates
    casualty_reason: Optional[str] = None
    flight_hours: float = 0.0

    @property
    def hardware_uid_int(self) -> int:
        """64-bit integer representation of silicon UID."""
        try:
            return int(self.hardware_uid, 16)
        except ValueError:
            return 0x3842A10F9B200000 + (self.drone_id if self.drone_id is not None else 0)

    @property
    def uid2_bytes(self) -> List[int]:
        """18-byte sequence for MAVLink AUTOPILOT_VERSION uid2 array."""
        uid_int = self.hardware_uid_int
        b = uid_int.to_bytes(8, "big").ljust(18, b"\x00")
        return list(b)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "hardware_uid": self.hardware_uid,
            "tail_number": self.tail_number,
            "sysid": self.sysid,
            "drone_id": self.drone_id,
            "pad_id": self.pad_id,
            "status": self.status,
            "registered_at": round(float(self.registered_at), 2),
            "lkp": [round(float(v), 2) for v in self.lkp] if self.lkp is not None else None,
            "casualty_reason": self.casualty_reason,
            "flight_hours": round(float(self.flight_hours), 2),
        }


@dataclass
class CasualtyEntry:
    """Permanent mission log record of a downed airframe."""
    hardware_uid: str
    tail_number: str
    sysid: Optional[int]
    drone_id: int
    crash_time: float
    lkp: List[float]
    reason: str

    def to_dict(self) -> Dict[str, Any]:
        return {
            "hardware_uid": self.hardware_uid,
            "tail_number": self.tail_number,
            "sysid": self.sysid,
            "drone_id": self.drone_id,
            "crash_time": round(float(self.crash_time), 2),
            "lkp": [round(float(v), 2) for v in self.lkp],
            "reason": self.reason,
        }


class FleetRegistry:
    """Central fleet enrollment and hardware identity manager."""

    BASE_UID_PREFIX = 0x3842A10F9B200000
    RESERVE_UID_PREFIX = 0x3842A10F9B20FF00

    def __init__(self, initial_count: int = 8, reserve_count: int = 16):
        self.records: Dict[str, AirframeRecord] = {}  # Keyed by hardware_uid
        self.reserves: List[AirframeRecord] = []       # Available standby reserves
        self.casualty_log: List[CasualtyEntry] = []
        self._reserve_counter: int = 0

        # 1. Enroll initial active airframes: ALITHEIA-V01 through ALITHEIA-V{initial_count:02d}
        for i in range(initial_count):
            uid_val = self.BASE_UID_PREFIX + (i + 1)
            uid_str = f"{uid_val:016X}"
            tail_num = f"ALITHEIA-V{i + 1:02d}"
            record = AirframeRecord(
                hardware_uid=uid_str,
                tail_number=tail_num,
                sysid=i + 1,
                drone_id=i,
                pad_id=i % 8,
                status="ACTIVE",
                registered_at=0.0,
            )
            self.records[uid_str] = record

        # 2. Populate standby reserves: ALITHEIA-RES-01 through ALITHEIA-RES-{reserve_count:02d}
        for r in range(1, reserve_count + 1):
            self._reserve_counter += 1
            uid_val = self.RESERVE_UID_PREFIX + self._reserve_counter
            uid_str = f"{uid_val:016X}"
            tail_num = f"ALITHEIA-RES-{self._reserve_counter:02d}"
            record = AirframeRecord(
                hardware_uid=uid_str,
                tail_number=tail_num,
                sysid=None,
                drone_id=None,
                pad_id=None,
                status="RESERVE",
                registered_at=0.0,
            )
            self.reserves.append(record)
            self.records[uid_str] = record

    def get_by_uid(self, hardware_uid: str) -> Optional[AirframeRecord]:
        return self.records.get(hardware_uid)

    def get_by_drone_id(self, drone_id: int) -> Optional[AirframeRecord]:
        for rec in self.records.values():
            if rec.drone_id == drone_id and rec.status in ("ACTIVE", "CASUALTY"):
                return rec
        return None

    def get_by_sysid(self, sysid: int) -> Optional[AirframeRecord]:
        for rec in self.records.values():
            if rec.sysid == sysid and rec.status == "ACTIVE":
                return rec
        return None

    def allocate_reserve(
        self, drone_id: int, pad_id: Optional[int] = None, sim_time: float = 0.0
    ) -> AirframeRecord:
        """Pulls the next standby reserve airframe and enrolls it into active service."""
        available = [r for r in self.reserves if r.status == "RESERVE"]
        if available:
            record = available[0]
        else:
            # Dynamically mint a fresh reserve if pool exhausted
            self._reserve_counter += 1
            uid_val = self.RESERVE_UID_PREFIX + self._reserve_counter
            uid_str = f"{uid_val:016X}"
            tail_num = f"ALITHEIA-RES-{self._reserve_counter:02d}"
            record = AirframeRecord(
                hardware_uid=uid_str,
                tail_number=tail_num,
                status="RESERVE",
            )
            self.reserves.append(record)
            self.records[uid_str] = record

        record.drone_id = drone_id
        record.sysid = drone_id + 1
        record.pad_id = pad_id
        record.status = "ACTIVE"
        record.registered_at = sim_time
        return record

    def record_casualty(
        self,
        drone_id: int,
        lkp: List[float],
        reason: str = "",
        sim_time: float = 0.0,
    ) -> Optional[CasualtyEntry]:
        """Mark an airframe as a casualty, permanently archiving its UID and crash coordinates."""
        record = self.get_by_drone_id(drone_id)
        if not record:
            return None

        record.status = "CASUALTY"
        record.lkp = list(lkp)
        record.casualty_reason = reason

        entry = CasualtyEntry(
            hardware_uid=record.hardware_uid,
            tail_number=record.tail_number,
            sysid=record.sysid,
            drone_id=drone_id,
            crash_time=sim_time,
            lkp=list(lkp),
            reason=reason or "UNKNOWN",
        )
        self.casualty_log.append(entry)
        return entry

    def reboot_airframe(self, drone_id: int) -> Optional[AirframeRecord]:
        """Re-activates a casualty airframe in-place (e.g. in field reboot lab), retaining its identity."""
        record = self.get_by_drone_id(drone_id)
        if record:
            record.status = "ACTIVE"
            record.casualty_reason = None
            return record
        return None

    def get_active_airframes(self) -> List[AirframeRecord]:
        return [r for r in self.records.values() if r.status == "ACTIVE"]

    def get_casualty_records(self) -> List[CasualtyEntry]:
        return list(self.casualty_log)

    def get_registry_snapshot(self) -> Dict[str, Any]:
        """Generate summary snapshot for GCS telemetry."""
        active = [r.to_dict() for r in self.records.values() if r.status == "ACTIVE"]
        casualties = [c.to_dict() for c in self.casualty_log]
        return {
            "total_registered": len(self.records),
            "active_count": len(active),
            "casualty_count": len(casualties),
            "reserve_count": len([r for r in self.reserves if r.status == "RESERVE"]),
            "active_fleet": active,
            "casualty_log": casualties,
        }
