import argparse
import serial
import sys
import time
from typing import Dict, List, Optional

DEFAULT_PORT = "COM5"        # change to your Arduino port
DEFAULT_BAUD = 115200

class TicBridge:
    """Serial bridge for two-motor Tic controller: send targets and read telemetry."""

    def __init__(self, port: str, baud: int = DEFAULT_BAUD, timeout: float = 1.0) -> None:
        self.port = port
        self.baud = baud
        self.timeout = timeout
        self.ser: Optional[serial.Serial] = None
        # Live telemetry attributes (updated on successful read)
        self.pos: List[int] = []
        self.target: List[int] = []
        self.vel: List[int] = []
        # Derived state: True when all positions match targets
        self.target_reached: bool = False

    def open(self) -> None:
        if self.ser is None:
            self.ser = serial.Serial(self.port, self.baud, timeout=self.timeout)

    def close(self) -> None:
        if self.ser is not None:
            try:
                self.ser.close()
            finally:
                self.ser = None

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close()

    @staticmethod
    def parse_status(line: str) -> Dict[str, List[int]]:
        # expected: pos:p1,p2;vel:v1,v2 (two motors)
        parts = dict(segment.split(":", 1) for segment in line.strip().split(";") if ":" in segment)
        def to_ints(key: str) -> List[int]:
            vals = parts.get(key, "")
            return [int(x) for x in vals.split(",") if x]
        return {
            "pos": to_ints("pos"),
            "vel": to_ints("vel"),
        }

    def send_targets(self, a: int, b: int) -> None:
        if not self.ser:
            raise RuntimeError("Serial not open. Call open() or use context manager.")
        msg = f"SET:{a},{b}\n".encode("ascii")
        self.ser.write(msg)
        self.ser.flush()
        # Cache targets locally; Arduino telemetry does not include target
        self.target = [a, b]

    def send_reset(self) -> None:
        """Send RESET command to halt and reset all motors to position 0."""
        if not self.ser:
            raise RuntimeError("Serial not open. Call open() or use context manager.")
        msg = b"RESET\n"
        self.ser.write(msg)
        self.ser.flush()
        # Clear cached target after reset
        self.target = [0, 0]

    def readline(self) -> Optional[str]:
        if not self.ser:
            raise RuntimeError("Serial not open. Call open() or use context manager.")
        try:
            raw = self.ser.readline()
        except serial.SerialException as e:
            print(f"Serial read error: {e}", file=sys.stderr)
            return None
        if not raw:
            return None
        try:
            return raw.decode(errors="ignore").strip()
        except UnicodeDecodeError:
            return None

    def stream_telemetry(self) -> Optional[Dict[str, List[int]]]:
        """Read one status line; return parsed telemetry dict or None."""
        line = self.readline()
        if not line:
            return None
        if line.startswith("pos:"):
            data = self.parse_status(line)
            # Update live attributes
            self.pos = data.get("pos", [])
            self.vel = data.get("vel", [])
            # Update derived flag: all pos equal target
            self.target_reached = (
                len(self.pos) == len(self.target) and
                all(p == t for p, t in zip(self.pos, self.target))
            )
            return data
        # Unknown or auxiliary messages; print and ignore
        print(line)
        return None

    def update(self) -> bool:
        """Convenience: read once and refresh attributes; return True if updated."""
        return self.stream_telemetry() is not None

    def set_target(self, target: int, motor: Optional[int] = None) -> None:
        """
        Set targets using single value.
        - If motor is None: set both motors to the same target.
        - If motor is 0 or 1: set only that motor, keeping the other at its current cached target (or 0 if unknown).
        """
        if motor is None:
            self.send_targets(target, target)
            return
        if motor not in (0, 1):
            raise ValueError("motor must be 0 or 1 for two-motor setup")
        other = 0
        if self.target and len(self.target) == 2:
            other = self.target[1 - motor]
        a, b = (target, other) if motor == 0 else (other, target)
        self.send_targets(a, b)

def main():
    ap = argparse.ArgumentParser(description="Tic bridge: telemetry + target commands (two motors)")
    ap.add_argument("--port", default=DEFAULT_PORT, help="Serial port (e.g., COM5)")
    ap.add_argument("--baud", type=int, default=DEFAULT_BAUD, help="Baud rate")
    ap.add_argument("--set", help="Send targets 'a,b' for two motors (e.g., 400,200)")
    ap.add_argument("--reset", action="store_true", help="Send reset command to halt and zero all motors")
    args = ap.parse_args()

    try:
        with TicBridge(args.port, args.baud) as bridge:
            print(f"Connected on {args.port} @ {args.baud}")

            if args.reset:
                print("Sending reset...")
                bridge.send_reset()
                time.sleep(0.1)

            if args.set:
                try:
                    a_str, b_str = [s.strip() for s in args.set.split(",", 1)]
                    a, b = int(a_str), int(b_str)
                except Exception:
                    print("--set requires two comma-separated integers, e.g., 400,200", file=sys.stderr)
                    sys.exit(2)
                print(f"Sending targets: {a},{b}")
                bridge.send_targets(a, b)
                time.sleep(0.1)  # allow Arduino to process and ACK in telemetry

            while True:
                data = bridge.stream_telemetry()
                if not data:
                    continue
                print(f"pos={bridge.pos} vel={bridge.vel} target={bridge.target} reached={bridge.target_reached}")
    except serial.SerialException as e:
        print(f"Serial error: {e}", file=sys.stderr)
        sys.exit(1)
    except KeyboardInterrupt:
        print("\nExiting...")

def main_mini():
    with TicBridge("COM5", 115200) as bridge:
        if bridge.update():
            print(bridge.pos, bridge.vel)

        bridge.set_target(400) # set both moors
        bridge.send_reset() # homing?
        # Microstps input!

# missing;: endstops, homing, etc.

if __name__ == "__main__":
    main()