import argparse
import serial
import sys
import time
from typing import Dict, List, Optional

DEFAULT_PORT = "COM10"        # change to your Arduino port
DEFAULT_BAUD = 115200

class TicBridge:
    """Serial bridge for two-motor Tic controller: send targets and read telemetry."""

    def __init__(self, port: str, baud: int = DEFAULT_BAUD, timeout: float = 1.0, motor_count: int = 4) -> None:
        self.port = port
        self.baud = baud
        self.timeout = timeout
        self.ser: Optional[serial.Serial] = None
        self.motor_count = motor_count
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

    def send_targets(self, *targets: int) -> None:
        if not self.ser:
            raise RuntimeError("Serial not open. Call open() or use context manager.")
        # Allow list/tuple as a single argument
        if len(targets) == 1 and isinstance(targets[0], (list, tuple)):
            targets = tuple(int(x) for x in targets[0])
        if len(targets) == 0:
            raise ValueError("send_targets requires at least one target value")

        # If only one target is provided, apply it to all motors
        targets_list = list(targets)
        if len(targets_list) == 1:
            targets_list = targets_list * self.motor_count
        elif len(targets_list) != self.motor_count:
            raise ValueError(f"Expected 1 or {self.motor_count} targets, got {len(targets_list)}")

        msg = ("SET:" + ",".join(str(t) for t in targets_list) + "\n").encode("ascii")
        self.ser.write(msg)
        self.ser.flush()
        # Cache targets locally; Arduino telemetry does not include target
        self.target = targets_list

    def send_reset(self) -> None:
        """Send RESET command to halt and reset all motors to position 0."""
        if not self.ser:
            raise RuntimeError("Serial not open. Call open() or use context manager.")
        msg = b"RESET\n"
        self.ser.write(msg)
        self.ser.flush()
        # Clear cached target after reset
        self.target = [0] * self.motor_count

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
            if self.pos:
                self.motor_count = len(self.pos)
                if len(self.target) != self.motor_count:
                    # Keep existing targets when known; pad/truncate to match motor count
                    padded = list(self.target[:self.motor_count])
                    padded += [0] * (self.motor_count - len(padded))
                    self.target = padded
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
        - If motor is None: set all motors to the same target.
        - If motor is specified: set only that motor, keeping others at their current cached targets (or 0 if unknown).
        """
        if motor is None:
            self.send_targets([target] * self.motor_count)
            return
        if motor < 0 or motor >= self.motor_count:
            raise ValueError(f"motor must be between 0 and {self.motor_count - 1}")

        current = list(self.target)
        if len(current) != self.motor_count:
            current = ([0] * self.motor_count)
        current[motor] = target
        self.send_targets(current)

def main():
    ap = argparse.ArgumentParser(description="Tic bridge: telemetry + target commands for multiple motors")
    ap.add_argument("--port", default=DEFAULT_PORT, help="Serial port (e.g., COM10)")
    ap.add_argument("--baud", type=int, default=DEFAULT_BAUD, help="Baud rate")
    ap.add_argument("--motors", type=int, default=4, help="Number of motors (targets per SET)")
    ap.add_argument("--set", help="Send targets as comma list for all motors (e.g., 400,200,0,0)")
    ap.add_argument("--reset", action="store_true", help="Send reset command to halt and zero all motors")
    args = ap.parse_args()

    try:
        with TicBridge(args.port, args.baud, motor_count=args.motors) as bridge:
            print(f"Connected on {args.port} @ {args.baud} for {bridge.motor_count} motors")

            if args.reset:
                print("Sending reset...")
                bridge.send_reset()
                time.sleep(0.1)

            if args.set:
                try:
                    raw_vals = [s.strip() for s in args.set.split(",")]
                    values = [int(v) for v in raw_vals if v != ""]
                except Exception:
                    print("--set requires comma-separated integers, e.g., 400,200,0,0", file=sys.stderr)
                    sys.exit(2)
                if len(values) not in (1, bridge.motor_count):
                    print(f"--set expects 1 or {bridge.motor_count} values", file=sys.stderr)
                    sys.exit(2)
                print(f"Sending targets: {','.join(str(v) for v in values)}")
                bridge.send_targets(values)
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
    print("Starting Tic Bridge...")
    while True:
        print("loop is running ...")
        
        with TicBridge("COM10", 115200) as bridge:
            if bridge.update():
                print("Updated telemetry:")
                print(bridge.pos, bridge.vel)

            bridge.set_target(200, motor=3)
            time.sleep(1.5)
            bridge.set_target(0, motor=3)
                #bridge.send_reset() # homing?
            # Microstps input!

# missing;: endstops, homing, etc.

if __name__ == "__main__":
    main_mini()