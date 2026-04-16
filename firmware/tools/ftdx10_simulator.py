from __future__ import annotations

import queue
import threading
import time
from dataclasses import dataclass, field
from typing import Callable, Dict, Optional

import tkinter as tk
from tkinter import messagebox, ttk

try:
    import serial
    from serial.tools import list_ports
except ImportError:  # pragma: no cover - runtime dependency hint
    serial = None
    list_ports = None


APP_TITLE = "FTDX10 CAT Simulator"
DEFAULT_ID_REPLY = "ID0761;"
AGC_VALUES = {
    "OFF": "00",
    "FAST": "01",
    "MID": "02",
    "SLOW": "03",
    "AUTO": "04",
}
MODE_VALUES = {
    "LSB": "1",
    "USB": "2",
    "CW": "3",
    "FM": "4",
    "AM": "5",
    "RTTY": "6",
    "CWR": "7",
    "DIGI": "8",
    "RTTYR": "9",
}
MODE_NAMES = {value: key for key, value in MODE_VALUES.items()}


def _bool_digit(value: bool) -> str:
    return "1" if value else "0"


def _bounded_int(text: str, fallback: int, minimum: int, maximum: int) -> int:
    try:
        value = int(text.strip())
    except (TypeError, ValueError):
        return fallback
    return max(minimum, min(maximum, value))


@dataclass
class RadioState:
    active_vfo: str = "A"
    vfo_a_hz: int = 14_074_000
    vfo_b_hz: int = 7_074_000
    mode_code: str = "2"
    vfo_a_mode_code: str = "2"
    vfo_b_mode_code: str = "2"
    split_on: bool = False
    tx_on: bool = False
    lock_on: bool = False
    tuner_on: bool = False
    preamp_on: bool = False
    nr_on: bool = False
    nb_on: bool = False
    notch_on: bool = False
    power_on: bool = True
    agc_code: str = "03"
    smeter: int = 50
    power_meter: int = 20
    swr_meter: int = 10
    ctcss_on: bool = False
    ctcss_tone: str = "0885"
    repeater_shift: str = "0"
    last_tune_started_at: float = 0.0
    last_raw_command: str = ""
    custom_exact: Dict[str, str] = field(default_factory=dict)
    custom_prefix: Dict[str, str] = field(default_factory=dict)

    @property
    def rx_hz(self) -> int:
        return self.vfo_a_hz if self.active_vfo == "A" else self.vfo_b_hz

    @property
    def tx_hz(self) -> int:
        if not self.split_on:
            return self.rx_hz
        return self.vfo_b_hz if self.active_vfo == "A" else self.vfo_a_hz

    def sync_modes_for_active_vfo(self) -> None:
        if self.active_vfo == "A":
            self.mode_code = self.vfo_a_mode_code
        else:
            self.mode_code = self.vfo_b_mode_code

    def set_active_vfo(self, which: str) -> None:
        self.active_vfo = "A" if which != "B" else "B"
        self.sync_modes_for_active_vfo()

    def set_active_frequency(self, hz: int) -> None:
        if self.active_vfo == "A":
            self.vfo_a_hz = hz
        else:
            self.vfo_b_hz = hz

    def set_active_mode(self, mode_code: str) -> None:
        self.mode_code = mode_code
        if self.active_vfo == "A":
            self.vfo_a_mode_code = mode_code
        else:
            self.vfo_b_mode_code = mode_code


class CommandRegistry:
    def __init__(self, state: RadioState, log: Callable[[str], None], state_changed: Callable[[], None]):
        self.state = state
        self.log = log
        self.state_changed = state_changed
        self.handlers: Dict[str, Callable[[str], Optional[str]]] = {
            "AC": self.handle_ac,
            "BP": self.handle_bp,
            "CN": self.handle_cn,
            "CT": self.handle_ct,
            "FA": self.handle_fa,
            "FB": self.handle_fb,
            "GT": self.handle_gt,
            "ID": self.handle_id,
            "IF": self.handle_if,
            "LK": self.handle_lk,
            "MD": self.handle_md,
            "NB": self.handle_nb,
            "NR": self.handle_nr,
            "OS": self.handle_os,
            "PA": self.handle_pa,
            "PS": self.handle_ps,
            "RI": self.handle_ri,
            "RM": self.handle_rm,
            "SM": self.handle_sm,
            "ST": self.handle_st,
            "SV": self.handle_sv,
            "TX": self.handle_tx,
            "VS": self.handle_vs,
        }

    def dispatch(self, command: str) -> Optional[str]:
        self.state.last_raw_command = command

        exact = self.state.custom_exact.get(command)
        if exact is not None:
            return exact

        for prefix, reply in self.state.custom_prefix.items():
            if command.startswith(prefix):
                return reply

        prefix = command[:2]
        handler = self.handlers.get(prefix)
        if handler is None:
            self.log(f"RX unknown: {command}")
            return None

        reply = handler(command)
        self.state_changed()
        return reply

    def handle_fa(self, command: str) -> Optional[str]:
        if command == "FA":
            return f"FA{self.state.vfo_a_hz:09d};"
        value = self._parse_number(command[2:], 9)
        if value is not None:
            self.state.vfo_a_hz = value
            if self.state.active_vfo == "A":
                self.state.sync_modes_for_active_vfo()
            return None
        return f"FA{self.state.vfo_a_hz:09d};"

    def handle_fb(self, command: str) -> Optional[str]:
        if command == "FB":
            return f"FB{self.state.vfo_b_hz:09d};"
        value = self._parse_number(command[2:], 9)
        if value is not None:
            self.state.vfo_b_hz = value
            if self.state.active_vfo == "B":
                self.state.sync_modes_for_active_vfo()
            return None
        return f"FB{self.state.vfo_b_hz:09d};"

    def handle_md(self, command: str) -> Optional[str]:
        if command == "MD0":
            return f"MD0{self.state.mode_code};"
        if not command.startswith("MD0"):
            return None
        mode_code = command[3:4]
        if mode_code in MODE_NAMES:
            self.state.set_active_mode(mode_code)
            return None
        return f"MD0{self.state.mode_code};"

    def handle_lk(self, command: str) -> Optional[str]:
        if command == "LK":
            return f"LK{_bool_digit(self.state.lock_on)};"
        self.state.lock_on = command[2:3] == "1"
        return None

    def handle_ac(self, command: str) -> Optional[str]:
        if command == "AC":
            return f"AC00{_bool_digit(self.state.tuner_on)};"
        arg = command[2:]
        if arg == "002":
            self.state.last_tune_started_at = time.time()
            return None
        if arg == "001":
            self.state.tuner_on = True
        elif arg == "000":
            self.state.tuner_on = False
        return None

    def handle_st(self, command: str) -> Optional[str]:
        if command == "ST":
            return f"ST{_bool_digit(self.state.split_on)};"
        self.state.split_on = command[2:3] == "1"
        return None

    def handle_vs(self, command: str) -> Optional[str]:
        if command == "VS":
            return f"VS{0 if self.state.active_vfo == 'A' else 1};"
        arg = command[2:3]
        if arg == "0":
            self.state.set_active_vfo("A")
        elif arg == "1":
            self.state.set_active_vfo("B")
        return None

    def handle_sv(self, command: str) -> str:
        self.state.vfo_a_hz, self.state.vfo_b_hz = self.state.vfo_b_hz, self.state.vfo_a_hz
        self.state.vfo_a_mode_code, self.state.vfo_b_mode_code = (
            self.state.vfo_b_mode_code,
            self.state.vfo_a_mode_code,
        )
        self.state.sync_modes_for_active_vfo()
        return "SV;"

    def handle_nr(self, command: str) -> Optional[str]:
        if command == "NR0":
            return f"NR0{_bool_digit(self.state.nr_on)};"
        arg = command[2:]
        if arg == "01":
            self.state.nr_on = True
        elif arg == "00":
            self.state.nr_on = False
        return None

    def handle_nb(self, command: str) -> Optional[str]:
        if command == "NB0":
            return f"NB0{_bool_digit(self.state.nb_on)};"
        arg = command[2:]
        if arg == "01":
            self.state.nb_on = True
        elif arg == "00":
            self.state.nb_on = False
        return None

    def handle_bp(self, command: str) -> Optional[str]:
        if command == "BP0":
            return f"BP0{1 if self.state.notch_on else 0:03d};"
        arg = command[2:]
        if arg == "0001":
            self.state.notch_on = True
        elif arg == "0000":
            self.state.notch_on = False
        return None

    def handle_pa(self, command: str) -> Optional[str]:
        if command == "PA0":
            return f"PA0{_bool_digit(self.state.preamp_on)};"
        arg = command[2:]
        if arg == "01":
            self.state.preamp_on = True
        elif arg == "00":
            self.state.preamp_on = False
        return None

    def handle_gt(self, command: str) -> Optional[str]:
        if command == "GT0":
            return f"GT0{self.state.agc_code};"
        code = command[2:]
        if code in AGC_VALUES.values():
            self.state.agc_code = code
            return None
        return f"GT0{self.state.agc_code};"

    def handle_ps(self, command: str) -> Optional[str]:
        if command == "PS":
            return f"PS{_bool_digit(self.state.power_on)};"
        self.state.power_on = command[2:3] == "1"
        return None

    def handle_ri(self, command: str) -> Optional[str]:
        code = command[2:]
        state = self._ri_state_for_code(code)
        if state is None:
            return None
        return f"RI{code}{state};"

    def handle_id(self, command: str) -> str:
        return DEFAULT_ID_REPLY

    def handle_if(self, command: str) -> str:
        return self._build_if_reply()

    def handle_sm(self, command: str) -> Optional[str]:
        if command != "SM0":
            return None
        return f"SM0{self.state.smeter:03d};"

    def handle_rm(self, command: str) -> Optional[str]:
        if command == "RM5":
            return f"RM5{self.state.power_meter:03d};"
        if command == "RM6":
            return f"RM6{self.state.swr_meter:03d};"
        return None

    def handle_tx(self, command: str) -> Optional[str]:
        if command == "TX":
            return f"TX{_bool_digit(self.state.tx_on)};"
        self.state.tx_on = command[2:3] == "1"
        return None

    def handle_ct(self, command: str) -> Optional[str]:
        if command == "CT":
            return f"CT{_bool_digit(self.state.ctcss_on)};"
        self.state.ctcss_on = command[2:3] == "1"
        return None

    def handle_cn(self, command: str) -> Optional[str]:
        if command == "CN":
            return f"CN{self.state.ctcss_tone};"
        digits = command[2:]
        if digits.isdigit() and len(digits) == 4:
            self.state.ctcss_tone = digits
            return None
        return f"CN{self.state.ctcss_tone};"

    def handle_os(self, command: str) -> Optional[str]:
        if command == "OS":
            return f"OS{self.state.repeater_shift};"
        value = command[2:3]
        if value in {"0", "1", "2"}:
            self.state.repeater_shift = value
            return None
        return f"OS{self.state.repeater_shift};"

    def _build_if_reply(self) -> str:
        freq = self.state.rx_hz
        mem = "000"
        clar = "+0000"
        rx_clar = "0"
        tx_clar = "0"
        tx = "1" if self.state.tx_on else "0"
        mode = self.state.mode_code
        vfo = "0"
        ctcss = "0"
        fixed = "00"
        repeater = self.state.repeater_shift
        split = "1" if self.state.split_on else "0"
        return f"IF{mem}{freq:09d}{clar}{rx_clar}{tx_clar}{mode}{vfo}{ctcss}{fixed}{repeater}{tx}{split};"

    @staticmethod
    def _parse_number(text: str, expected_len: int) -> Optional[int]:
        if len(text) != expected_len or not text.isdigit():
            return None
        return int(text)

    def _ri_state_for_code(self, code: str) -> Optional[str]:
        if code == "5":
            return "1" if self.state.active_vfo == "A" and self.state.tx_on else "0"
        if code == "6":
            return "1" if self.state.active_vfo == "B" and self.state.tx_on else "0"
        if code == "7":
            return "1" if self.state.active_vfo == "A" and not self.state.tx_on else "0"
        if code == "8":
            return "1" if self.state.active_vfo == "B" and not self.state.tx_on else "0"
        return None


class SerialWorker(threading.Thread):
    def __init__(self, port: str, baudrate: int, registry: CommandRegistry, event_queue: "queue.Queue[tuple[str, str]]"):
        super().__init__(daemon=True)
        self.port = port
        self.baudrate = baudrate
        self.registry = registry
        self.event_queue = event_queue
        self.stop_event = threading.Event()
        self.ser = None

    def run(self) -> None:
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
        except Exception as exc:  # pragma: no cover - hardware dependent
            self.event_queue.put(("error", f"Port open failed: {exc}"))
            return

        self.event_queue.put(("status", f"Connected to {self.port} @ {self.baudrate}"))
        buffer = ""

        while not self.stop_event.is_set():
            try:
                chunk = self.ser.read(256)
            except Exception as exc:  # pragma: no cover - hardware dependent
                self.event_queue.put(("error", f"Serial read failed: {exc}"))
                break

            if not chunk:
                continue

            try:
                text = chunk.decode("ascii", errors="ignore")
            except Exception:
                continue
            buffer += text

            while ";" in buffer:
                raw, buffer = buffer.split(";", 1)
                command = raw.strip().upper()
                if not command:
                    continue
                self.event_queue.put(("rx", command + ";"))
                reply = self.registry.dispatch(command)
                if reply:
                    try:
                        self.ser.write(reply.encode("ascii"))
                        self.ser.flush()
                        self.event_queue.put(("tx", reply))
                    except Exception as exc:  # pragma: no cover - hardware dependent
                        self.event_queue.put(("error", f"Serial write failed: {exc}"))
                        self.stop_event.set()
                        break

        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
        self.event_queue.put(("status", "Disconnected"))

    def stop(self) -> None:
        self.stop_event.set()


class SimulatorApp:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title(APP_TITLE)
        self.root.geometry("1180x760")

        self.state = RadioState()
        self.events: "queue.Queue[tuple[str, str]]" = queue.Queue()
        self.registry = CommandRegistry(self.state, self.log_line, lambda: None)
        self.worker: Optional[SerialWorker] = None

        self.port_var = tk.StringVar()
        self.baud_var = tk.StringVar(value="38400")
        self.status_var = tk.StringVar(value="Idle")
        self.tune_status_var = tk.StringVar(value="TUNE idle")

        self.active_vfo_var = tk.StringVar(value=self.state.active_vfo)
        self.vfo_a_var = tk.StringVar(value=str(self.state.vfo_a_hz))
        self.vfo_b_var = tk.StringVar(value=str(self.state.vfo_b_hz))
        self.mode_var = tk.StringVar(value=MODE_NAMES[self.state.mode_code])
        self.vfo_a_mode_var = tk.StringVar(value=MODE_NAMES[self.state.vfo_a_mode_code])
        self.vfo_b_mode_var = tk.StringVar(value=MODE_NAMES[self.state.vfo_b_mode_code])
        self.split_var = tk.BooleanVar(value=self.state.split_on)
        self.tx_var = tk.BooleanVar(value=self.state.tx_on)
        self.lock_var = tk.BooleanVar(value=self.state.lock_on)
        self.tuner_var = tk.BooleanVar(value=self.state.tuner_on)
        self.preamp_var = tk.BooleanVar(value=self.state.preamp_on)
        self.nr_var = tk.BooleanVar(value=self.state.nr_on)
        self.nb_var = tk.BooleanVar(value=self.state.nb_on)
        self.notch_var = tk.BooleanVar(value=self.state.notch_on)
        self.power_var = tk.BooleanVar(value=self.state.power_on)
        self.agc_var = tk.StringVar(value="SLOW")
        self.smeter_var = tk.StringVar(value=str(self.state.smeter))
        self.power_meter_var = tk.StringVar(value=str(self.state.power_meter))
        self.swr_meter_var = tk.StringVar(value=str(self.state.swr_meter))
        self.custom_cmd_var = tk.StringVar()
        self.custom_reply_var = tk.StringVar()
        self._editable_widgets: Dict[str, tk.Widget] = {}

        self._build_ui()
        self.refresh_ports()
        self.refresh_view()
        self.root.after(100, self.poll_events)
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    def _build_ui(self) -> None:
        outer = ttk.Frame(self.root, padding=10)
        outer.pack(fill=tk.BOTH, expand=True)
        outer.columnconfigure(0, weight=5)
        outer.columnconfigure(1, weight=4)
        outer.rowconfigure(1, weight=1)

        conn = ttk.LabelFrame(outer, text="Connection", padding=8)
        conn.grid(row=0, column=0, columnspan=2, sticky="ew", pady=(0, 8))
        for idx in range(8):
            conn.columnconfigure(idx, weight=1 if idx in {1, 3, 6} else 0)

        ttk.Label(conn, text="COM Port").grid(row=0, column=0, sticky="w")
        self.port_combo = ttk.Combobox(conn, textvariable=self.port_var, state="readonly")
        self.port_combo.grid(row=0, column=1, sticky="ew", padx=(6, 12))
        ttk.Button(conn, text="Rescan", command=self.refresh_ports).grid(row=0, column=2, padx=(0, 12))

        ttk.Label(conn, text="Baud").grid(row=0, column=3, sticky="w")
        ttk.Entry(conn, textvariable=self.baud_var, width=10).grid(row=0, column=4, sticky="w", padx=(6, 12))
        ttk.Button(conn, text="Connect", command=self.connect).grid(row=0, column=5, padx=(0, 6))
        ttk.Button(conn, text="Disconnect", command=self.disconnect).grid(row=0, column=6, padx=(0, 6))
        ttk.Label(conn, textvariable=self.status_var).grid(row=0, column=7, sticky="e")

        left = ttk.Frame(outer)
        left.grid(row=1, column=0, sticky="nsew", padx=(0, 8))
        left.columnconfigure(0, weight=1)

        state_frame = ttk.LabelFrame(left, text="Radio State", padding=10)
        state_frame.grid(row=0, column=0, sticky="ew", pady=(0, 8))
        for idx in range(4):
            state_frame.columnconfigure(idx, weight=1)

        ttk.Label(state_frame, text="Active VFO").grid(row=0, column=0, sticky="w")
        active_vfo_entry = ttk.Entry(state_frame, textvariable=self.active_vfo_var, state="readonly")
        active_vfo_entry.grid(row=0, column=1, sticky="ew", padx=(6, 12))
        ttk.Label(state_frame, text="Active Mode").grid(row=0, column=2, sticky="w")
        mode_entry = ttk.Entry(state_frame, textvariable=self.mode_var, state="readonly")
        mode_entry.grid(row=0, column=3, sticky="ew")

        ttk.Label(state_frame, text="VFO A Hz").grid(row=1, column=0, sticky="w", pady=(8, 0))
        vfo_a_entry = ttk.Entry(state_frame, textvariable=self.vfo_a_var)
        vfo_a_entry.grid(row=1, column=1, sticky="ew", padx=(6, 12), pady=(8, 0))
        ttk.Label(state_frame, text="VFO B Hz").grid(row=1, column=2, sticky="w", pady=(8, 0))
        vfo_b_entry = ttk.Entry(state_frame, textvariable=self.vfo_b_var)
        vfo_b_entry.grid(row=1, column=3, sticky="ew", pady=(8, 0))

        ttk.Label(state_frame, text="VFO A Mode").grid(row=2, column=0, sticky="w", pady=(8, 0))
        vfo_a_mode_entry = ttk.Entry(state_frame, textvariable=self.vfo_a_mode_var, state="readonly")
        vfo_a_mode_entry.grid(row=2, column=1, sticky="ew", padx=(6, 12), pady=(8, 0))
        ttk.Label(state_frame, text="VFO B Mode").grid(row=2, column=2, sticky="w", pady=(8, 0))
        vfo_b_mode_entry = ttk.Entry(state_frame, textvariable=self.vfo_b_mode_var, state="readonly")
        vfo_b_mode_entry.grid(row=2, column=3, sticky="ew", pady=(8, 0))

        vfo_a_buttons = ttk.Frame(state_frame)
        vfo_a_buttons.grid(row=3, column=0, columnspan=2, sticky="ew", pady=(6, 0))
        vfo_b_buttons = ttk.Frame(state_frame)
        vfo_b_buttons.grid(row=3, column=2, columnspan=2, sticky="ew", pady=(6, 0))
        for idx in range(6):
            vfo_a_buttons.columnconfigure(idx, weight=1)
            vfo_b_buttons.columnconfigure(idx, weight=1)
        for idx, mode_name in enumerate(["USB", "LSB", "CW", "AM", "FM", "DIGI"]):
            ttk.Button(vfo_a_buttons, text=mode_name, command=lambda m=mode_name: self.set_vfo_mode_from_gui("A", m)).grid(row=0, column=idx, sticky="ew", padx=(0 if idx == 0 else 4, 0))
            ttk.Button(vfo_b_buttons, text=mode_name, command=lambda m=mode_name: self.set_vfo_mode_from_gui("B", m)).grid(row=0, column=idx, sticky="ew", padx=(0 if idx == 0 else 4, 0))

        vfo_select = ttk.Frame(state_frame)
        vfo_select.grid(row=4, column=0, columnspan=2, sticky="w", pady=(8, 0))
        ttk.Button(vfo_select, text="Set Active A", command=lambda: self.set_active_vfo_from_gui("A")).grid(row=0, column=0, padx=(0, 6))
        ttk.Button(vfo_select, text="Set Active B", command=lambda: self.set_active_vfo_from_gui("B")).grid(row=0, column=1)

        ttk.Label(state_frame, text="TUNE").grid(row=4, column=2, sticky="w", pady=(8, 0))
        ttk.Label(state_frame, textvariable=self.tune_status_var).grid(row=4, column=3, sticky="w", pady=(8, 0))

        toggles = ttk.LabelFrame(left, text="Switches", padding=10)
        toggles.grid(row=1, column=0, sticky="ew", pady=(0, 8))
        for idx in range(4):
            toggles.columnconfigure(idx, weight=1)

        toggle_specs = [
            ("Split", self.split_var, 0, 0),
            ("TX active", self.tx_var, 0, 1),
            ("LOCK", self.lock_var, 0, 2),
            ("Tuner", self.tuner_var, 0, 3),
            ("PA/IPO", self.preamp_var, 1, 0),
            ("NR", self.nr_var, 1, 1),
            ("NB", self.nb_var, 1, 2),
            ("NOTCH", self.notch_var, 1, 3),
            ("Power", self.power_var, 2, 0),
        ]
        for label, var, row, col in toggle_specs:
            ttk.Checkbutton(toggles, text=label, variable=var, command=self.apply_form_to_state).grid(row=row, column=col, sticky="w")

        analog = ttk.LabelFrame(left, text="Meters and Extras", padding=10)
        analog.grid(row=2, column=0, sticky="ew", pady=(0, 8))
        for idx in range(4):
            analog.columnconfigure(idx, weight=1)

        ttk.Label(analog, text="S-Meter").grid(row=0, column=0, sticky="w")
        smeter_entry = ttk.Entry(analog, textvariable=self.smeter_var)
        smeter_entry.grid(row=0, column=1, sticky="ew", padx=(6, 12))
        ttk.Label(analog, text="Power RM5").grid(row=0, column=2, sticky="w")
        power_entry = ttk.Entry(analog, textvariable=self.power_meter_var)
        power_entry.grid(row=0, column=3, sticky="ew")

        ttk.Label(analog, text="SWR RM6").grid(row=1, column=0, sticky="w", pady=(8, 0))
        swr_entry = ttk.Entry(analog, textvariable=self.swr_meter_var)
        swr_entry.grid(row=1, column=1, sticky="ew", padx=(6, 12), pady=(8, 0))
        ttk.Label(analog, text="AGC").grid(row=1, column=2, sticky="w", pady=(8, 0))
        agc_entry = ttk.Entry(analog, textvariable=self.agc_var, state="readonly")
        agc_entry.grid(row=1, column=3, sticky="ew", pady=(8, 0))

        actions = ttk.Frame(left)
        actions.grid(row=3, column=0, sticky="ew")
        actions.columnconfigure(0, weight=1)
        actions.columnconfigure(1, weight=1)
        ttk.Button(actions, text="Apply GUI State", command=self.apply_form_to_state).grid(row=0, column=0, sticky="ew", padx=(0, 6))
        ttk.Button(actions, text="Reset Defaults", command=self.reset_defaults).grid(row=0, column=1, sticky="ew", padx=(6, 0))

        right = ttk.Frame(outer)
        right.grid(row=1, column=1, sticky="nsew")
        right.columnconfigure(0, weight=1)
        right.rowconfigure(1, weight=1)

        custom = ttk.LabelFrame(right, text="Extensions", padding=10)
        custom.grid(row=0, column=0, sticky="ew", pady=(0, 8))
        custom.columnconfigure(1, weight=1)
        custom.columnconfigure(3, weight=1)

        ttk.Label(custom, text="Exact CAT command").grid(row=0, column=0, sticky="w")
        ttk.Entry(custom, textvariable=self.custom_cmd_var).grid(row=0, column=1, sticky="ew", padx=(6, 12))
        ttk.Label(custom, text="Reply").grid(row=0, column=2, sticky="w")
        ttk.Entry(custom, textvariable=self.custom_reply_var).grid(row=0, column=3, sticky="ew", padx=(6, 12))
        ttk.Button(custom, text="Add exact rule", command=self.add_exact_custom).grid(row=0, column=4)

        ttk.Label(custom, text="Unknown commands appear in the log. Add a fixed reply here without changing the code.").grid(row=1, column=0, columnspan=5, sticky="w", pady=(8, 0))

        log_frame = ttk.LabelFrame(right, text="CAT Log", padding=10)
        log_frame.grid(row=1, column=0, sticky="nsew")
        log_frame.columnconfigure(0, weight=1)
        log_frame.rowconfigure(1, weight=3)
        log_frame.rowconfigure(3, weight=2)

        ttk.Label(log_frame, text="Important commands and replies").grid(row=0, column=0, sticky="w")

        point_frame = ttk.Frame(log_frame)
        point_frame.grid(row=1, column=0, sticky="nsew", pady=(0, 8))
        point_frame.columnconfigure(0, weight=1)
        point_frame.rowconfigure(0, weight=1)
        self.point_log_widget = tk.Text(point_frame, wrap="word", height=30)
        self.point_log_widget.grid(row=0, column=0, sticky="nsew")
        point_scroll = ttk.Scrollbar(point_frame, orient="vertical", command=self.point_log_widget.yview)
        point_scroll.grid(row=0, column=1, sticky="ns")
        self.point_log_widget.configure(yscrollcommand=point_scroll.set)

        ttk.Label(log_frame, text="Polling ticker").grid(row=2, column=0, sticky="w")

        poll_frame = ttk.Frame(log_frame)
        poll_frame.grid(row=3, column=0, sticky="nsew")
        poll_frame.columnconfigure(0, weight=1)
        poll_frame.rowconfigure(0, weight=1)
        self.poll_log_widget = tk.Text(poll_frame, wrap="word", height=12)
        self.poll_log_widget.grid(row=0, column=0, sticky="nsew")
        poll_scroll = ttk.Scrollbar(poll_frame, orient="vertical", command=self.poll_log_widget.yview)
        poll_scroll.grid(row=0, column=1, sticky="ns")
        self.poll_log_widget.configure(yscrollcommand=poll_scroll.set)

        self._editable_widgets = {
            "active_vfo": active_vfo_entry,
            "mode": mode_entry,
            "vfo_a": vfo_a_entry,
            "vfo_b": vfo_b_entry,
            "vfo_a_mode": vfo_a_mode_entry,
            "vfo_b_mode": vfo_b_mode_entry,
            "smeter": smeter_entry,
            "power_meter": power_entry,
            "swr_meter": swr_entry,
            "agc": agc_entry,
        }

    def refresh_ports(self) -> None:
        if list_ports is None:
            self.status_var.set("pyserial missing")
            return
        ports = [port.device for port in list_ports.comports()]
        self.port_combo["values"] = ports
        if ports and self.port_var.get() not in ports:
            self.port_var.set(ports[0])
        if not ports:
            self.port_var.set("")

    def connect(self) -> None:
        if serial is None:
            messagebox.showerror(APP_TITLE, "Please run 'pip install pyserial' first.")
            return
        if self.worker and self.worker.is_alive():
            return
        port = self.port_var.get().strip()
        if not port:
            messagebox.showwarning(APP_TITLE, "Please select a COM port first.")
            return
        self.apply_form_to_state()
        baud = _bounded_int(self.baud_var.get(), 38400, 300, 921600)
        self.worker = SerialWorker(port, baud, self.registry, self.events)
        self.worker.start()

    def disconnect(self) -> None:
        if self.worker:
            self.worker.stop()
            self.worker = None

    def add_exact_custom(self) -> None:
        command = self.custom_cmd_var.get().strip().upper().rstrip(";")
        reply = self.custom_reply_var.get().strip().upper()
        if not command or not reply:
            return
        if not reply.endswith(";"):
            reply += ";"
        self.state.custom_exact[command] = reply
        self.log_line(f"Custom exact added: {command}; -> {reply}")
        self.custom_cmd_var.set("")
        self.custom_reply_var.set("")

    def apply_form_to_state(self, refresh_ui: bool = True) -> None:
        self.state.set_active_vfo(self.active_vfo_var.get())
        self.state.vfo_a_hz = _bounded_int(self.vfo_a_var.get(), self.state.vfo_a_hz, 0, 999_999_999)
        self.state.vfo_b_hz = _bounded_int(self.vfo_b_var.get(), self.state.vfo_b_hz, 0, 999_999_999)
        self.state.vfo_a_mode_code = MODE_VALUES.get(self.vfo_a_mode_var.get(), self.state.vfo_a_mode_code)
        self.state.vfo_b_mode_code = MODE_VALUES.get(self.vfo_b_mode_var.get(), self.state.vfo_b_mode_code)
        self.state.mode_code = self.state.vfo_a_mode_code if self.state.active_vfo == "A" else self.state.vfo_b_mode_code
        self.state.split_on = self.split_var.get()
        self.state.tx_on = self.tx_var.get()
        self.state.lock_on = self.lock_var.get()
        self.state.tuner_on = self.tuner_var.get()
        self.state.preamp_on = self.preamp_var.get()
        self.state.nr_on = self.nr_var.get()
        self.state.nb_on = self.nb_var.get()
        self.state.notch_on = self.notch_var.get()
        self.state.power_on = self.power_var.get()
        self.state.smeter = _bounded_int(self.smeter_var.get(), self.state.smeter, 0, 255)
        self.state.power_meter = _bounded_int(self.power_meter_var.get(), self.state.power_meter, 0, 255)
        self.state.swr_meter = _bounded_int(self.swr_meter_var.get(), self.state.swr_meter, 0, 255)
        self.state.agc_code = AGC_VALUES.get(self.agc_var.get(), self.state.agc_code)
        self.state.sync_modes_for_active_vfo()
        if refresh_ui:
            self.refresh_view()

    def set_active_vfo_from_gui(self, which: str) -> None:
        self.state.set_active_vfo("A" if which != "B" else "B")
        self.refresh_view()

    def set_vfo_mode_from_gui(self, which: str, mode_name: str) -> None:
        mode_code = MODE_VALUES.get(mode_name)
        if mode_code is None:
            return
        if which == "A":
            self.state.vfo_a_mode_code = mode_code
            if self.state.active_vfo == "A":
                self.state.mode_code = mode_code
        else:
            self.state.vfo_b_mode_code = mode_code
            if self.state.active_vfo == "B":
                self.state.mode_code = mode_code
        self.refresh_view()

    def reset_defaults(self) -> None:
        self.state = RadioState(custom_exact=self.state.custom_exact, custom_prefix=self.state.custom_prefix)
        self.registry.state = self.state
        self.refresh_view()

    def refresh_view(self) -> None:
        self._set_var_if_idle("active_vfo", self.active_vfo_var, self.state.active_vfo)
        self._set_var_if_idle("vfo_a", self.vfo_a_var, str(self.state.vfo_a_hz))
        self._set_var_if_idle("vfo_b", self.vfo_b_var, str(self.state.vfo_b_hz))
        self._set_var_if_idle("mode", self.mode_var, MODE_NAMES.get(self.state.mode_code, "USB"))
        self._set_var_if_idle("vfo_a_mode", self.vfo_a_mode_var, MODE_NAMES.get(self.state.vfo_a_mode_code, "USB"))
        self._set_var_if_idle("vfo_b_mode", self.vfo_b_mode_var, MODE_NAMES.get(self.state.vfo_b_mode_code, "USB"))
        self.split_var.set(self.state.split_on)
        self.tx_var.set(self.state.tx_on)
        self.lock_var.set(self.state.lock_on)
        self.tuner_var.set(self.state.tuner_on)
        self.preamp_var.set(self.state.preamp_on)
        self.nr_var.set(self.state.nr_on)
        self.nb_var.set(self.state.nb_on)
        self.notch_var.set(self.state.notch_on)
        self.power_var.set(self.state.power_on)
        self._set_var_if_idle("smeter", self.smeter_var, str(self.state.smeter))
        self._set_var_if_idle("power_meter", self.power_meter_var, str(self.state.power_meter))
        self._set_var_if_idle("swr_meter", self.swr_meter_var, str(self.state.swr_meter))
        self._set_var_if_idle("agc", self.agc_var, next((name for name, code in AGC_VALUES.items() if code == self.state.agc_code), "SLOW"))
        tune_age = time.time() - self.state.last_tune_started_at if self.state.last_tune_started_at else None
        if tune_age is None:
            self.tune_status_var.set("TUNE idle")
        elif tune_age < 2.5:
            self.tune_status_var.set("TUNE active")
        else:
            self.tune_status_var.set("TUNE triggered")

    def _set_var_if_idle(self, key: str, var: tk.Variable, value: str) -> None:
        widget = self._editable_widgets.get(key)
        if widget is not None and self.root.focus_get() == widget:
            return
        var.set(value)

    def poll_events(self) -> None:
        while True:
            try:
                kind, payload = self.events.get_nowait()
            except queue.Empty:
                break

            if kind == "status":
                self.status_var.set(payload)
                self.log_line(payload, category="point")
            elif kind == "error":
                self.status_var.set(payload)
                self.log_line(payload, category="point")
            elif kind == "rx":
                self.log_line(f"RX  {payload}", category=self._classify_log_category("RX", payload))
            elif kind == "tx":
                self.log_line(f"TX  {payload}", category=self._classify_log_category("TX", payload))
            self.refresh_view()

        self.root.after(100, self.poll_events)

    def _classify_log_category(self, direction: str, payload: str) -> str:
        command = payload.strip().upper().rstrip(";")
        if command in {"FA", "FB"}:
            return "poll"
        if command in {"RI5", "RI6", "RI7", "RI8"}:
            return "poll"
        if direction in {"RX", "TX"} and (command.startswith("FA") or command.startswith("FB")):
            return "poll"
        if direction in {"RX", "TX"} and command.startswith("RI"):
            return "poll"
        return "point"

    def log_line(self, text: str, category: str = "point") -> None:
        stamp = time.strftime("%H:%M:%S")
        widget = self.poll_log_widget if category == "poll" else self.point_log_widget
        widget.insert(tk.END, f"[{stamp}] {text}\n")
        widget.see(tk.END)

    def on_close(self) -> None:
        self.disconnect()
        self.root.destroy()


def main() -> None:
    root = tk.Tk()
    SimulatorApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
