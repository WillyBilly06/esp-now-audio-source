#!/usr/bin/env python3
import queue
import socket
import struct
import sys
import threading
import time
import math
import array
import tkinter as tk
import select
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from tkinter import messagebox, ttk

try:
    import sounddevice as sd
except Exception:
    sd = None


DEFAULT_SAMPLE_RATE = 16000
DEFAULT_CHANNELS = 2
DEFAULT_FRAME_US = 7500
DEFAULT_BYTES_PER_CH = 72

LC3_PROFILES: dict[str, tuple[int, int, int, int]] = {
    "48k / 2ch / 7.5ms / 72B": (48000, 2, 7500, 72),
    "16k / 2ch / 10ms / 24B": (16000, 2, 10000, 24),
    "16k / 2ch / 10ms / 28B": (16000, 2, 10000, 28),
    "48k / 2ch / 10ms / 24B": (48000, 2, 10000, 24),
    "48k / 2ch / 10ms / 56B": (48000, 2, 10000, 56),
    "Custom": (DEFAULT_SAMPLE_RATE, DEFAULT_CHANNELS, DEFAULT_FRAME_US, DEFAULT_BYTES_PER_CH),
}

LATENCY_MODES = ["Ultra Low", "Low Latency", "Balanced"]

# IMPORTANT:
# - target_frames controls start-up latency
# - max_frames prevents runaway latency if something stalls
# Set max_frames high enough that it NEVER hits in normal operation (=> buf_drop stays 0).
LATENCY_CFG = {
    "Ultra Low": {
        "target_frames": 2,
        "max_frames": 20,          # hard cap to keep end-to-end latency bounded
        "sock_wait_s": 0.007,
        "warmup_s": 0.035,
        "out_latency_s": 0.012,
        "block_div": 2,
        "wasapi_exclusive": True,
    },
    "Low Latency": {
        "target_frames": 3,
        "max_frames": 28,          # keep <~200ms even under jitter bursts
        "sock_wait_s": 0.010,
        "warmup_s": 0.04,
        "out_latency_s": 0.018,
        "block_div": 2,
        "wasapi_exclusive": True,
    },
    "Balanced": {
        "target_frames": 6,
        "max_frames": 60,
        "sock_wait_s": 0.020,
        "warmup_s": 0.15,
        "out_latency_s": 0.030,
        "block_div": 1,
        "wasapi_exclusive": False,
    },
}


def resolve_lc3_libpath(user_path: str | None) -> str | None:
    def pick_from_dir(base: Path) -> str | None:
        direct = [
            base / "lc3.dll",
            base / "liblc3.dll",
            base / "Release" / "lc3.dll",
            base / "Release" / "liblc3.dll",
            base / "build" / "Release" / "lc3.dll",
            base / "build" / "Release" / "liblc3.dll",
        ]
        for c in direct:
            if c.is_file():
                return str(c)
        for c in base.rglob("*lc3*.dll"):
            if c.is_file():
                return str(c)
        return None

    if user_path:
        p = Path(user_path)
        if p.is_file():
            if p.suffix.lower() == ".dll":
                return str(p)
            return None
        if p.is_dir():
            return pick_from_dir(p)

    from os import getenv

    env_lib = getenv("LC3_LIBPATH")
    if env_lib:
        p = Path(env_lib)
        if p.is_file() and p.suffix.lower() == ".dll":
            return str(p)
        if p.is_dir():
            found = pick_from_dir(p)
            if found:
                return found

    repo_root = Path(__file__).resolve().parents[1]
    candidates = [
        repo_root / "components" / "liblc3" / "build-host" / "Release" / "lc3.dll",
        repo_root / "components" / "liblc3" / "build-host" / "lc3.dll",
        repo_root / "components" / "liblc3" / "build" / "Release" / "lc3.dll",
        repo_root / "components" / "liblc3" / "build" / "lc3.dll",
        repo_root / "lc3.dll",
    ]
    for c in candidates:
        if c.is_file():
            return str(c)
    return None


def import_lc3(lc3_python_dir: str):
    sys.path.insert(0, lc3_python_dir)
    try:
        import lc3  # type: ignore
    except Exception as exc:
        raise RuntimeError(
            "Could not import lc3 Python binding. "
            "Point LC3 Python Dir to components/liblc3/python"
        ) from exc
    return lc3


@dataclass
class ReceiverConfig:
    esp_ip: str
    port: int
    room: int
    sample_rate: int
    channels: int
    frame_us: int
    bytes_per_ch: int
    latency_mode: str
    sound_device: int | str | None
    libpath: str | None
    lc3_python_dir: str


class ReceiverWorker(threading.Thread):
    def __init__(self, cfg: ReceiverConfig, event_q: queue.Queue):
        super().__init__(daemon=True)
        self.cfg = cfg
        self.event_q = event_q
        self.stop_event = threading.Event()

    def stop(self):
        self.stop_event.set()

    def _push(self, kind: str, data):
        self.event_q.put((kind, data))

    def _make_wasapi_settings(self, exclusive: bool):
        if sd is None:
            return None
        if not sys.platform.startswith("win"):
            return None
        try:
            return sd.WasapiSettings(exclusive=exclusive)
        except Exception:
            return None

    def run(self):
        payload_bytes = self.cfg.bytes_per_ch * self.cfg.channels
        header_len = 17
        frame_samples = max(1, int(self.cfg.sample_rate * self.cfg.frame_us / 1_000_000))

        mode = self.cfg.latency_mode if self.cfg.latency_mode in LATENCY_CFG else "Low Latency"
        m = LATENCY_CFG[mode]
        sock_wait_s = m["sock_wait_s"]
        target_frames = m["target_frames"]
        max_frames = m["max_frames"]
        warmup_s = m["warmup_s"]
        out_latency_s = m["out_latency_s"]
        block_div = m["block_div"]
        want_wasapi_excl = bool(m["wasapi_exclusive"])

        blocksize = max(64, frame_samples // max(1, int(block_div)))
        out_chunk_bytes = blocksize * self.cfg.channels * 2  # int16

        buf_lock = threading.Lock()
        pcm_buf = deque()  # deque[bytes]
        first_packet_event = threading.Event()

        packets = 0
        plc_frames = 0
        bad_packets = 0
        last_bad_log = 0.0

        # Split “drops”
        net_lost_frames = 0   # missing seq frames (real network loss)
        buf_drop_frames = 0   # app jitter-buffer overflow (your code’s fault)

        expected_seq = None
        last_packet_ts = 0.0
        last_keepalive = 0.0
        last_stats = 0.0
        last_no_rx_log = 0.0
        audio_peak = 0

        def buf_depth() -> int:
            with buf_lock:
                return len(pcm_buf)

        def buf_push(pcm16: bytes):
            nonlocal buf_drop_frames
            with buf_lock:
                # Safety cap: prevents runaway latency/memory if audio device stalls.
                # If you *never* want buffer dropping, raise max_frames further,
                # but then latency can grow unbounded under bad conditions.
                while len(pcm_buf) >= max_frames:
                    pcm_buf.popleft()   # drop OLDEST audio (keeps "most recent" / lowest-latency)
                    buf_drop_frames += 1
                pcm_buf.append(pcm16)

        def buf_pop() -> bytes | None:
            with buf_lock:
                if pcm_buf:
                    return pcm_buf.popleft()
            return None

        def write_pcm(stream, pcm16: bytes):
            if out_chunk_bytes <= 0 or len(pcm16) <= out_chunk_bytes:
                stream.write(pcm16)
                return
            for off in range(0, len(pcm16), out_chunk_bytes):
                stream.write(pcm16[off: off + out_chunk_bytes])

        try:
            if sd is None:
                raise RuntimeError("Missing dependency: pip install sounddevice")

            lc3 = import_lc3(self.cfg.lc3_python_dir)
            resolved_lib = resolve_lc3_libpath(self.cfg.libpath)
            if not resolved_lib:
                if self.cfg.libpath:
                    raise RuntimeError(
                        "LC3 Lib Path is set but no *.dll was found there. "
                        "Point to lc3.dll directly, or to a folder containing lc3.dll."
                    )
                raise RuntimeError(
                    "LC3 library not found. Build lc3.dll first, then set 'LC3 Lib Path' in UI.\n"
                    "Example (PowerShell):\n"
                    "  cmake -S components/liblc3 -B components/liblc3/build-host -DBUILD_SHARED_LIBS=ON\n"
                    "  cmake --build components/liblc3/build-host --config Release\n"
                    "Then use: components/liblc3/build-host/Release/lc3.dll"
                )

            dec = lc3.Decoder(
                frame_duration_us=self.cfg.frame_us,
                sample_rate_hz=self.cfg.sample_rate,
                num_channels=self.cfg.channels,
                libpath=resolved_lib,
            )
            self._push("log", f"Using LC3 library: {resolved_lib}")

            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            try:
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            except OSError:
                pass

            local_port = 0
            try:
                sock.bind(("0.0.0.0", self.cfg.port))
                local_port = self.cfg.port
            except OSError:
                sock.bind(("0.0.0.0", 0))
                local_port = sock.getsockname()[1]
            sock.setblocking(False)

            # Give the kernel room to absorb short bursts (reduces real UDP packet loss)
            try:
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
            except OSError:
                pass

            sock.sendto(b"sub", (self.cfg.esp_ip, self.cfg.port))
            self._push(
                "log",
                f"Subscribed to {self.cfg.esp_ip}:{self.cfg.port} (local UDP {local_port})",
            )
            self._push(
                "log",
                f"Latency mode: {mode} (target_frames={target_frames}, max_frames={max_frames}, "
                f"blocksize={blocksize} samples, req_out_latency={out_latency_s*1000:.0f}ms, "
                f"wasapi_exclusive={want_wasapi_excl})",
            )

            def rx_loop():
                nonlocal packets, plc_frames, bad_packets, expected_seq, last_packet_ts, last_keepalive, last_no_rx_log, net_lost_frames, last_bad_log
                while not self.stop_event.is_set():
                    now = time.monotonic()

                    if now - last_keepalive > 1.0:
                        try:
                            sock.sendto(b"k", (self.cfg.esp_ip, self.cfg.port))
                        except OSError:
                            pass
                        last_keepalive = now

                    # Wait for readability, then DRAIN all queued packets quickly
                    r, _, _ = select.select([sock], [], [], sock_wait_s)
                    if not r:
                        if packets == 0 and now - last_no_rx_log > 2.0:
                            if bad_packets > 0:
                                self._push(
                                    "log",
                                    "UDP is reachable but packets are rejected. Check LC3 Profile (frame_us/bytes_per_ch/room).",
                                )
                            else:
                                self._push("log", "No UDP audio packets yet. Check ESP IP and that source is streaming.")
                            last_no_rx_log = now
                        continue

                    while not self.stop_event.is_set():
                        try:
                            packet, _ = sock.recvfrom(2048)
                        except BlockingIOError:
                            break
                        except OSError:
                            return

                        if len(packet) < header_len:
                            bad_packets += 1
                            continue

                        try:
                            (
                                magic,
                                msg_type,
                                room_code,
                                _reserved,
                                seq,
                                payload_len,
                                _stream_id,
                                _flags,
                                _capture_us,
                            ) = struct.unpack_from("<BBBBHHIBI", packet, 0)
                        except struct.error:
                            bad_packets += 1
                            continue

                        if magic != 0xA5 or msg_type != 0x10 or room_code != (self.cfg.room & 0xFF):
                            bad_packets += 1
                            continue
                        if payload_len != payload_bytes or len(packet) < header_len + payload_len:
                            bad_packets += 1
                            if packets == 0 and now - last_bad_log > 2.0:
                                last_bad_log = now
                                self._push(
                                    "log",
                                    f"Payload mismatch: got {payload_len} bytes, expected {payload_bytes}. Check LC3 Profile.",
                                )
                            continue

                        frame = packet[header_len: header_len + payload_len]

                        if expected_seq is None:
                            expected_seq = (seq + 1) & 0xFFFF
                        else:
                            diff = (seq - expected_seq) & 0xFFFF
                            if 0 < diff < 0x8000:
                                # This is REAL loss (UDP/Wi-Fi). We can only conceal it with PLC.
                                missing = diff
                                net_lost_frames += missing

                                # Insert PLC for every missing frame to preserve timing (no "skipped time").
                                # If you get huge bursts of loss, this will add CPU, but keeps playback continuous.
                                for _ in range(missing):
                                    plc_pcm = dec.decode(None, bit_depth=16)
                                    buf_push(plc_pcm)
                                    plc_frames += 1
                            expected_seq = (seq + 1) & 0xFFFF

                        pcm16 = dec.decode(frame, bit_depth=16)
                        buf_push(pcm16)

                        packets += 1
                        last_packet_ts = now
                        first_packet_event.set()

            rx_t = threading.Thread(target=rx_loop, daemon=True)
            rx_t.start()

            self._push("state", "connected")

            extra = self._make_wasapi_settings(exclusive=want_wasapi_excl)

            def open_stream(extra_settings):
                return sd.RawOutputStream(
                    samplerate=self.cfg.sample_rate,
                    channels=self.cfg.channels,
                    dtype="int16",
                    device=self.cfg.sound_device,
                    blocksize=blocksize,
                    latency=out_latency_s,
                    extra_settings=extra_settings,
                )

            try:
                stream_ctx = open_stream(extra)
            except Exception as exc:
                if extra is not None:
                    self._push("log", f"WASAPI exclusive failed ({exc}); retrying without exclusive...")
                    stream_ctx = open_stream(None)
                else:
                    raise

            with stream_ctx as stream:
                try:
                    self._push("log", f"Audio stream latency (reported): {stream.latency}")
                except Exception:
                    pass

                # Warm-up: keep it tiny to preserve low latency
                if first_packet_event.wait(timeout=2.0):
                    warm_deadline = time.monotonic() + warmup_s
                    while not self.stop_event.is_set():
                        if buf_depth() >= target_frames:
                            break
                        if time.monotonic() >= warm_deadline:
                            break
                        time.sleep(0.002)

                while not self.stop_event.is_set():
                    now = time.monotonic()

                    if buf_depth() > (target_frames + 10):
                        with buf_lock:
                            while len(pcm_buf) > (target_frames + 6):
                                pcm_buf.popleft()
                                buf_drop_frames += 1

                    pcm16 = buf_pop()
                    if pcm16 is None:
                        # Short grace wait before PLC to absorb packet arrival jitter.
                        waited = 0
                        while waited < 3 and not self.stop_event.is_set():
                            time.sleep(0.001)
                            pcm16 = buf_pop()
                            if pcm16 is not None:
                                break
                            waited += 1

                        if pcm16 is None:
                            pcm16 = dec.decode(None, bit_depth=16)
                            plc_frames += 1

                    write_pcm(stream, pcm16)

                    s = array.array("h")
                    s.frombytes(pcm16)
                    if s:
                        p = max(abs(v) for v in s)
                        if p > audio_peak:
                            audio_peak = p

                    if now - last_stats > 0.5:
                        no_rx_ms = int((now - last_packet_ts) * 1000) if last_packet_ts > 0 else -1
                        self._push(
                            "stats",
                            {
                                "packets": packets,
                                "plc": plc_frames,
                                "bad": bad_packets,
                                "net_lost": net_lost_frames,
                                "buf_drop": buf_drop_frames,
                                "buf": buf_depth(),
                                "no_rx_ms": no_rx_ms,
                                "peak": audio_peak,
                            },
                        )
                        last_stats = now
                        audio_peak = 0

            try:
                sock.close()
            except OSError:
                pass
            self._push("state", "stopped")
            self._push("log", "Disconnected")

        except OSError as exc:
            self._push("state", "error")
            self._push(
                "log",
                "OS error while loading LC3/audio device: "
                f"{exc}. If this is WinError 5, set LC3 Lib Path to the actual lc3.dll file, not just a folder.",
            )
        except Exception as exc:
            self._push("state", "error")
            self._push("log", f"Error: {exc}")


class App:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("OpenALS LC3 UDP Receiver")
        self.root.geometry("800x560")

        self.event_q = queue.Queue()
        self.worker = None

        self.esp_ip = tk.StringVar(value="192.168.4.1")
        self.port = tk.StringVar(value="5000")
        self.room = tk.StringVar(value="0x7F")
        self.sample_rate = tk.StringVar(value=str(DEFAULT_SAMPLE_RATE))
        self.channels = tk.StringVar(value=str(DEFAULT_CHANNELS))
        self.frame_us = tk.StringVar(value=str(DEFAULT_FRAME_US))
        self.bytes_per_ch = tk.StringVar(value=str(DEFAULT_BYTES_PER_CH))
        self.profile_name = tk.StringVar(value="48k / 2ch / 7.5ms / 72B")
        self.latency_mode = tk.StringVar(value="Ultra Low")
        self.libpath = tk.StringVar(value="")
        self.lc3_python_dir = tk.StringVar(
            value=str(Path(__file__).resolve().parents[1] / "components" / "liblc3" / "python")
        )
        self.device = tk.StringVar(value="")

        self.status = tk.StringVar(value="Idle")
        self.stats = tk.StringVar(value="packets=0  plc=0  bad=0  last_rx=never")

        self._build_ui()
        self.apply_profile_from_name(self.profile_name.get())
        self.refresh_devices()
        self.root.after(100, self.process_events)
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    def _build_ui(self):
        frm = ttk.Frame(self.root, padding=10)
        frm.pack(fill=tk.BOTH, expand=True)

        cfg = ttk.LabelFrame(frm, text="Connection")
        cfg.pack(fill=tk.X, padx=2, pady=2)

        self._row(cfg, 0, "ESP IP", self.esp_ip)
        self._row(cfg, 1, "Port", self.port)
        self._row(cfg, 2, "Room", self.room)

        profile_row = ttk.Frame(cfg)
        profile_row.grid(row=3, column=0, columnspan=2, sticky="ew", padx=6, pady=2)
        ttk.Label(profile_row, text="LC3 Profile", width=16).pack(side=tk.LEFT)
        self.profile_box = ttk.Combobox(
            profile_row,
            textvariable=self.profile_name,
            values=list(LC3_PROFILES.keys()),
            state="readonly",
        )
        self.profile_box.pack(side=tk.LEFT, fill=tk.X, expand=True)
        self.profile_box.bind("<<ComboboxSelected>>", lambda _e: self.apply_profile_from_name(self.profile_name.get()))

        mode_row = ttk.Frame(cfg)
        mode_row.grid(row=4, column=0, columnspan=2, sticky="ew", padx=6, pady=2)
        ttk.Label(mode_row, text="Latency Mode", width=16).pack(side=tk.LEFT)
        self.mode_box = ttk.Combobox(
            mode_row,
            textvariable=self.latency_mode,
            values=LATENCY_MODES,
            state="readonly",
        )
        self.mode_box.pack(side=tk.LEFT, fill=tk.X, expand=True)

        self._row(cfg, 5, "Sample Rate", self.sample_rate)
        self._row(cfg, 6, "Channels", self.channels)
        self._row(cfg, 7, "Frame (us)", self.frame_us)
        self._row(cfg, 8, "Bytes/Ch", self.bytes_per_ch)

        self._row(cfg, 9, "LC3 Python Dir", self.lc3_python_dir)
        self._row(cfg, 10, "LC3 Lib Path", self.libpath)

        dev_row = ttk.Frame(cfg)
        dev_row.grid(row=11, column=0, columnspan=2, sticky="ew", padx=6, pady=4)
        ttk.Label(dev_row, text="Output Device", width=16).pack(side=tk.LEFT)
        self.device_box = ttk.Combobox(dev_row, textvariable=self.device, state="readonly")
        self.device_box.pack(side=tk.LEFT, fill=tk.X, expand=True)
        ttk.Button(dev_row, text="Refresh", command=self.refresh_devices).pack(side=tk.LEFT, padx=6)

        btns = ttk.Frame(frm)
        btns.pack(fill=tk.X, pady=6)
        self.connect_btn = ttk.Button(btns, text="Connect", command=self.connect)
        self.connect_btn.pack(side=tk.LEFT)
        self.disconnect_btn = ttk.Button(btns, text="Disconnect", command=self.disconnect, state=tk.DISABLED)
        self.disconnect_btn.pack(side=tk.LEFT, padx=8)
        self.test_btn = ttk.Button(btns, text="Test Tone", command=self.play_test_tone)
        self.test_btn.pack(side=tk.LEFT, padx=8)

        stat = ttk.LabelFrame(frm, text="Status")
        stat.pack(fill=tk.X, padx=2, pady=2)
        ttk.Label(stat, textvariable=self.status).pack(anchor="w", padx=8, pady=3)
        ttk.Label(stat, textvariable=self.stats).pack(anchor="w", padx=8, pady=3)

        logf = ttk.LabelFrame(frm, text="Log")
        logf.pack(fill=tk.BOTH, expand=True, padx=2, pady=2)
        self.log = tk.Text(logf, height=12)
        self.log.pack(fill=tk.BOTH, expand=True)

    def _row(self, parent, row, label, var):
        r = ttk.Frame(parent)
        r.grid(row=row, column=0, columnspan=2, sticky="ew", padx=6, pady=2)
        ttk.Label(r, text=label, width=16).pack(side=tk.LEFT)
        ttk.Entry(r, textvariable=var).pack(side=tk.LEFT, fill=tk.X, expand=True)

    def refresh_devices(self):
        if sd is None:
            self.device_box["values"] = ["sounddevice not installed"]
            self.device.set("sounddevice not installed")
            return
        vals = []
        try:
            hostapis = sd.query_hostapis()
        except Exception:
            hostapis = None

        for i, info in enumerate(sd.query_devices()):
            if info.get("max_output_channels", 0) > 0:
                api_name = ""
                try:
                    if hostapis is not None:
                        api_name = hostapis[info["hostapi"]]["name"]
                except Exception:
                    api_name = ""
                suffix = f" [{api_name}]" if api_name else ""
                vals.append(f"{i}: {info['name']}{suffix}")

        self.device_box["values"] = vals
        if vals and not self.device.get():
            self.device.set(vals[0])

    def _selected_device(self):
        text = self.device.get().strip()
        if not text:
            return None
        idx = text.split(":", 1)[0].strip()
        return int(idx) if idx.isdigit() else text

    def apply_profile_from_name(self, name: str):
        prof = LC3_PROFILES.get(name)
        if not prof:
            return
        sr, ch, frame, bpc = prof
        self.sample_rate.set(str(sr))
        self.channels.set(str(ch))
        self.frame_us.set(str(frame))
        self.bytes_per_ch.set(str(bpc))

    def add_log(self, msg: str):
        self.log.insert(tk.END, msg + "\n")
        self.log.see(tk.END)

    def connect(self):
        try:
            cfg = ReceiverConfig(
                esp_ip=self.esp_ip.get().strip(),
                port=int(self.port.get().strip()),
                room=int(self.room.get().strip(), 0),
                sample_rate=int(self.sample_rate.get().strip()),
                channels=int(self.channels.get().strip()),
                frame_us=int(self.frame_us.get().strip()),
                bytes_per_ch=int(self.bytes_per_ch.get().strip()),
                latency_mode=self.latency_mode.get().strip(),
                sound_device=self._selected_device(),
                libpath=self.libpath.get().strip() or None,
                lc3_python_dir=self.lc3_python_dir.get().strip(),
            )
            if cfg.sample_rate <= 0 or cfg.channels <= 0 or cfg.frame_us <= 0 or cfg.bytes_per_ch <= 0:
                raise ValueError("LC3 profile values must be positive integers")
            if cfg.latency_mode not in LATENCY_MODES:
                raise ValueError("Invalid latency mode")
        except Exception as exc:
            messagebox.showerror("Invalid settings", str(exc))
            return

        self.worker = ReceiverWorker(cfg, self.event_q)
        self.worker.start()
        self.connect_btn.configure(state=tk.DISABLED)
        self.disconnect_btn.configure(state=tk.NORMAL)
        self.status.set("Connecting...")
        self.add_log("Starting receiver...")

    def disconnect(self):
        if self.worker:
            self.worker.stop()
            self.worker = None
        self.connect_btn.configure(state=tk.NORMAL)
        self.disconnect_btn.configure(state=tk.DISABLED)
        self.status.set("Stopping...")

    def process_events(self):
        while True:
            try:
                kind, data = self.event_q.get_nowait()
            except queue.Empty:
                break

            if kind == "log":
                self.add_log(str(data))
            elif kind == "state":
                if data == "connected":
                    self.status.set("Connected and playing")
                elif data == "stopped":
                    self.status.set("Stopped")
                    self.connect_btn.configure(state=tk.NORMAL)
                    self.disconnect_btn.configure(state=tk.DISABLED)
                elif data == "error":
                    self.status.set("Error")
                    self.connect_btn.configure(state=tk.NORMAL)
                    self.disconnect_btn.configure(state=tk.DISABLED)
            elif kind == "stats":
                last_rx = "never" if data.get("no_rx_ms", -1) < 0 else f"{data['no_rx_ms']}ms"
                peak = data.get("peak", 0)
                buf = data.get("buf", 0)
                self.stats.set(
                    f"packets={data['packets']}  plc={data['plc']}  bad={data['bad']}  "
                    f"net_lost={data['net_lost']}  buf_drop={data['buf_drop']}  buf={buf}  last_rx={last_rx}  peak={peak}"
                )

        self.root.after(100, self.process_events)

    def on_close(self):
        self.disconnect()
        self.root.destroy()

    def play_test_tone(self):
        if sd is None:
            messagebox.showerror("Missing dependency", "Install sounddevice first")
            return
        try:
            dev = self._selected_device()
            sr = int(self.sample_rate.get().strip())
            ch = int(self.channels.get().strip())
            frame_us = int(self.frame_us.get().strip())
            frame_samples = max(1, int(sr * frame_us / 1_000_000))
            blocksize = max(64, frame_samples // 2)

            dur_s = 1.0
            freq = 440.0
            n = int(sr * dur_s)
            pcm = array.array("h")
            for i in range(n):
                v = int(0.2 * 32767.0 * math.sin(2.0 * math.pi * freq * (i / sr)))
                for _ in range(ch):
                    pcm.append(v)

            with sd.RawOutputStream(
                samplerate=sr,
                channels=ch,
                dtype="int16",
                device=dev,
                blocksize=blocksize,
                latency=0.03,
            ) as stream:
                stream.write(pcm.tobytes())
            self.add_log("Played 1s test tone")
        except Exception as exc:
            messagebox.showerror("Test tone failed", str(exc))


def main():
    root = tk.Tk()
    App(root)
    root.mainloop()


if __name__ == "__main__":
    main()