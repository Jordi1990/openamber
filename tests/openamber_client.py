import json
import socket
import time
from typing import Any, Callable, Dict, Optional


class OpenAmberClient:
    """Client for controlling and testing OpenAmber via TestBridge socket."""

    def __init__(self, host: str = "127.0.0.1", port: int = 8888, timeout: float = 5.0, raise_on_error: bool = True):
        self.host = host
        self.port = port
        self.timeout = timeout
        self.raise_on_error = raise_on_error
        self.sock: Optional[socket.socket] = None

    def _check_status(self, res: Dict[str, Any], action: str) -> bool:
        ok = res.get("status") == "ok"
        if not ok and self.raise_on_error:
            msg = res.get("message", "unknown error")
            raise RuntimeError(f"OpenAmber command '{action}' failed: {msg}")
        return ok

    def connect(self, retries: int = 40, delay: float = 0.5) -> bool:
        """Connect to the TestBridge server."""
        for _ in range(retries):
            try:
                self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.sock.settimeout(self.timeout)
                self.sock.connect((self.host, self.port))
                return True
            except (ConnectionRefusedError, socket.timeout, OSError):
                time.sleep(delay)
        raise TimeoutError(f"Could not connect to OpenAmber test bridge at {self.host}:{self.port}")

    def close(self):
        """Close connection to server."""
        if self.sock:
            try:
                self.sock.close()
            except Exception:
                pass
            self.sock = None

    def send_cmd(self, cmd_dict: Dict[str, Any]) -> Dict[str, Any]:
        """Send command dict and receive JSON response."""
        for attempt in range(2):
            if not self.sock:
                self.connect(retries=10, delay=0.2)

            payload = json.dumps(cmd_dict) + "\n"
            try:
                self.sock.sendall(payload.encode("utf-8"))

                buf = b""
                while not buf.endswith(b"\n"):
                    chunk = self.sock.recv(4096)
                    if not chunk:
                        raise ConnectionResetError("Connection closed by OpenAmber")
                    buf += chunk

                resp = json.loads(buf.decode("utf-8").strip())
                return resp
            except (socket.timeout, TimeoutError, ConnectionResetError, BrokenPipeError, OSError):
                self.close()
                if attempt == 1:
                    raise
                time.sleep(0.3)

    def ping(self) -> bool:
        """Check server connectivity."""
        res = self.send_cmd({"cmd": "ping"})
        return res.get("status") == "ok"

    def click(self, widget_id: str) -> bool:
        """Simulate clicking an LVGL widget."""
        res = self.send_cmd({"cmd": "click", "id": widget_id})
        time.sleep(0.05)
        return self._check_status(res, f"click({widget_id})")

    def get_widget(self, widget_id: str) -> Dict[str, Any]:
        """Query widget status (visible, hidden, text, checked, disabled)."""
        return self.send_cmd({"cmd": "get_widget", "id": widget_id})

    def is_visible(self, widget_id: str) -> bool:
        """Check if widget is visible."""
        w = self.get_widget(widget_id)
        return bool(w.get("visible", False))

    def is_hidden(self, widget_id: str) -> bool:
        """Check if widget is hidden."""
        w = self.get_widget(widget_id)
        return bool(w.get("hidden", True))

    def get_label(self, widget_id: str) -> str:
        """Get text of a label widget."""
        w = self.get_widget(widget_id)
        return str(w.get("text", ""))

    def set_sensor(self, sensor_id: str, value: float) -> bool:
        """Publish a new float state to a sensor."""
        res = self.send_cmd({"cmd": "set_sensor", "id": sensor_id, "value": float(value)})
        return self._check_status(res, f"set_sensor({sensor_id}, {value})")

    def set_number(self, number_id: str, value: float) -> bool:
        """Publish a new float state to a number entity."""
        res = self.send_cmd({"cmd": "set_number", "id": number_id, "value": float(value)})
        return self._check_status(res, f"set_number({number_id}, {value})")

    def set_switch(self, switch_id: str, value: bool) -> bool:
        """Turn switch on (True) or off (False)."""
        res = self.send_cmd({"cmd": "set_switch", "id": switch_id, "value": bool(value)})
        return self._check_status(res, f"set_switch({switch_id}, {value})")

    def set_binary_sensor(self, sensor_id: str, value: bool) -> bool:
        """Publish state to a binary sensor."""
        res = self.send_cmd({"cmd": "set_binary_sensor", "id": sensor_id, "value": bool(value)})
        return self._check_status(res, f"set_binary_sensor({sensor_id}, {value})")

    def set_select(self, select_id: str, option: str) -> bool:
        """Set the active option of a select entity."""
        res = self.send_cmd({"cmd": "set_select", "id": select_id, "option": str(option)})
        return self._check_status(res, f"set_select({select_id}, {option})")

    def set_climate(
        self,
        climate_id: str,
        target_temperature: Optional[float] = None,
        mode: Optional[str] = None,
    ) -> bool:
        """Set target temperature or mode on a climate entity."""
        cmd: Dict[str, Any] = {"cmd": "set_climate", "id": climate_id}
        if target_temperature is not None:
            cmd["target_temperature"] = float(target_temperature)
        if mode is not None:
            cmd["mode"] = str(mode)
        res = self.send_cmd(cmd)
        return self._check_status(res, f"set_climate({climate_id})")

    def advance_time(self, seconds: float = 0, ms: int = 0, step_s: float = 0) -> int:
        """Advance virtual time by the given amount (in seconds or ms).
        If step_s > 0, advances in increments to trigger polling component update cycles.
        """
        total_ms = int(ms + seconds * 1000)
        if step_s > 0 and total_ms > int(step_s * 1000):
            chunk_ms = int(step_s * 1000)
            elapsed = 0
            offset = 0
            while elapsed < total_ms:
                slice_ms = min(chunk_ms, total_ms - elapsed)
                res = self.send_cmd({"cmd": "advance_time", "ms": slice_ms})
                self.step(ms=30)
                offset = res.get("offset_ms", 0)
                elapsed += slice_ms
            return offset

        res = self.send_cmd({"cmd": "advance_time", "ms": total_ms})
        # Give host event loop a chance to process timers/scheduler
        self.step(ms=30)
        return res.get("offset_ms", 0)

    def reset_time(self) -> bool:
        """Reset virtual time offset to 0."""
        res = self.send_cmd({"cmd": "reset_time"})
        self.step(ms=30)
        return res.get("status") == "ok"

    def get_time(self) -> Dict[str, Any]:
        """Query current virtual time and offset."""
        return self.send_cmd({"cmd": "get_time"})

    def get_entity(self, entity_id: str) -> Any:
        """Get state of any registered entity."""
        res = self.send_cmd({"cmd": "get_entity", "id": entity_id})
        if res.get("type") == "climate":
            return res
        return res.get("state")

    def step(self, ms: int = 100) -> bool:
        """Pause test runner and allow ESPHome event loop cycles to run."""
        res = self.send_cmd({"cmd": "step", "ms": ms})
        time.sleep(ms / 1000.0)
        return res.get("status") == "ok"

    def wait_for_condition(
        self,
        predicate: Callable[["OpenAmberClient"], bool],
        timeout: float = 3.0,
        poll_interval: float = 0.1,
    ) -> bool:
        """Poll until predicate returns True or timeout is reached."""
        start = time.time()
        while time.time() - start < timeout:
            if predicate(self):
                return True
            time.sleep(poll_interval)
        return False

    def exit(self):
        """Ask OpenAmber to exit."""
        try:
            self.send_cmd({"cmd": "exit"})
        except Exception:
            pass
        self.close()
