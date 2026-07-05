from dataclasses import dataclass, replace
from threading import Event, Lock, Thread

from inputs import UnpluggedError, get_gamepad  # type: ignore[import-untyped]


@dataclass
class ControllerState:
    left_x: float = 0.0
    left_y: float = 0.0
    right_x: float = 0.0
    right_y: float = 0.0

    a: bool = False
    b: bool = False
    x: bool = False
    y: bool = False

    left_bumper: bool = False
    right_bumper: bool = False


def normalize_axis(value: int, centre: int = 128, half_range: int = 128) -> float:
    """Convert an unsigned controller axis approximately into [-1.0, 1.0]."""
    normalized = (value - centre) / half_range
    return max(-1.0, min(1.0, normalized))


class ControllerReader:
    def __init__(self) -> None:
        self._state = ControllerState()
        self._lock = Lock()
        self._stop_event = Event()
        self._thread = Thread(target=self._run, daemon=True)

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()

    def get_state(self) -> ControllerState:
        with self._lock:
            return replace(self._state)

    def _run(self) -> None:
        while not self._stop_event.is_set():
            try:
                events = get_gamepad()
            except UnpluggedError:
                continue

            with self._lock:
                for event in events:
                    self._handle_event(event.code, event.state)

    def _handle_event(self, code: str, value: int) -> None:
        # Replace these mappings with the codes printed by controller_test.py.
        match code:
            case "ABS_X":
                self._state.left_x = normalize_axis(value)
            case "ABS_Y":
                self._state.left_y = -normalize_axis(value)

            case "ABS_RX":
                self._state.right_x = normalize_axis(value)
            case "ABS_RY":
                self._state.right_y = -normalize_axis(value)

            case "BTN_SOUTH":
                self._state.a = bool(value)
            case "BTN_EAST":
                self._state.b = bool(value)
            case "BTN_NORTH":
                self._state.x = bool(value)
            case "BTN_WEST":
                self._state.y = bool(value)

            case "BTN_TL":
                self._state.left_bumper = bool(value)
            case "BTN_TR":
                self._state.right_bumper = bool(value)
