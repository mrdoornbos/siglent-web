# siglent-scope: USB mirror and control for the Siglent SDS1202X-E oscilloscope.
# Copyright (C) 2026 Michael Doornbos
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

"""Screen mirror, data capture, and remote control for the Siglent SDS1202X-E over USB.

The scope enumerates as a USBTMC device and speaks the SDS1000X-E SCPI dialect. This
talks to it through pyvisa's pure-python backend + libusb -- no Windows software.

All instrument access is serialized through one Scope lock because USBTMC is a single,
non-concurrent link: the screen grabber, measurements, waveform pulls and control
commands take turns on the same connection.

Subcommands:
    list                       discover scopes / show *IDN?
    shot   [-o out.png]        save one screenshot
    mirror                     local Tk window (s=save, q=quit) -- desktop use
    serve  [--host ...]        MJPEG web mirror + measurements + control (run on USB host)
    measure [-c C1 C2] [--json]   print live measurements
    wave   -c C1 -o out.csv    export real sample data (CSV, or .png for a plot)
    cmd    "C1:VDIV?"          send a raw SCPI command (query if it ends in '?')
"""

from __future__ import annotations

import argparse
import io
import re
import signal
import struct
import subprocess
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

import pyvisa

# Siglent USB vendor ids. pyvisa-py renders the VID in DECIMAL in the resource
# string (e.g. USB0::62701::...), so we match on the decimal values, not hex.
# 0xF4EC = 62700 (older units), 0xF4ED = 62701 (e.g. SDS1202X-E).
SIGLENT_VIDS = (62700, 62701)

# The SDS1000X-E display grid is 14 horizontal x 8 vertical divisions, and the
# 8-bit ADC uses 25 codes per vertical division. Both are needed to convert raw
# waveform bytes back into volts and seconds.
HGRID = 14
CODES_PER_DIV = 25

# A sensible default set of per-channel measurements (SCPI PAVA parameters).
DEFAULT_PARAMS = ("PKPK", "AMPL", "MAX", "MIN", "MEAN", "RMS", "FREQ", "PER", "DUTY")


# ---------------------------------------------------------------------------
# discovery + parsing helpers
# ---------------------------------------------------------------------------

def _is_siglent(resource: str) -> bool:
    """True if the USB resource string's vendor-id field is a known Siglent VID."""
    parts = resource.split("::")
    if len(parts) < 2 or not parts[0].upper().startswith("USB"):
        return False
    try:
        return int(parts[1]) in SIGLENT_VIDS
    except ValueError:
        return False


def find_scope_resource(rm: pyvisa.ResourceManager) -> str | None:
    """Return the first USB resource string that looks like a Siglent scope."""
    for res in rm.list_resources():
        if _is_siglent(res):
            return res
    # Fall back to any USBTMC instrument if no Siglent VID matched.
    for res in rm.list_resources():
        if res.upper().startswith("USB") and "INSTR" in res.upper():
            return res
    return None


_SI = {"G": 1e9, "M": 1e6, "k": 1e3, "m": 1e-3, "u": 1e-6, "n": 1e-9, "p": 1e-12}
_NUM_RE = re.compile(r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?")


def parse_value(token: str) -> float | None:
    """Pull a float (applying an SI suffix) out of a SCPI value token like '5.00E-08S'
    or '1.00GSa/s'. Returns None for unmeasurable readings (Siglent sends '****')."""
    if not token or "*" in token:
        return None
    m = _NUM_RE.search(token)
    if not m:
        return None
    val = float(m.group())
    unit = token[m.end():]
    # A leading SI prefix multiplies the value, but not 'Sa/s' (already absolute)
    # and not 's'/'S' seconds.
    if unit[:1] in _SI and not unit.startswith("Sa"):
        val *= _SI[unit[0]]
    return val


def query_number(resp: str) -> float | None:
    """Parse the numeric value out of a single-value query response.
    e.g. 'C1:VDIV 2.00E+00V' -> 2.0 , 'SARA 1.00E+09Sa/s' -> 1e9."""
    resp = resp.strip()
    if not resp:
        return None
    return parse_value(resp.split()[-1])


def parse_pava(resp: str) -> float | None:
    """Parse a PAVA? response like 'C1:PAVA PKPK,3.36E+00V' -> 3.36."""
    if "," not in resp:
        return None
    return parse_value(resp.rsplit(",", 1)[-1].strip())


def _after(resp: str) -> str:
    """Return the argument part of a SCPI reply: 'C1:CPL D1M' -> 'D1M'."""
    parts = resp.strip().split(None, 1)
    return parts[1].strip() if len(parts) > 1 else ""


def default_name(ext: str) -> str:
    return datetime.now().strftime(f"sds1202xe_%Y%m%d_%H%M%S.{ext}")


# ---------------------------------------------------------------------------
# Scope -- single locked USB connection shared by every feature
# ---------------------------------------------------------------------------

class Scope:
    def __init__(self, resource: str | None = None):
        self.requested = resource
        self.lock = threading.RLock()
        self.inst = None
        self.resource: str | None = None
        self._open()

    def _open(self):
        rm = pyvisa.ResourceManager("@py")
        res = self.requested or find_scope_resource(rm)
        if not res:
            raise RuntimeError(
                "No Siglent USBTMC scope found. Is it plugged in and powered on?\n"
                "Run 'scope list' to see all detected VISA resources."
            )
        inst = rm.open_resource(res)
        inst.timeout = 5000  # ms; SCDP transfer is ~770 KB
        inst.chunk_size = 1024 * 1024
        inst.encoding = "latin-1"  # tolerate stray bytes; we never crash on decode
        self.inst = inst
        self.resource = res
        self._drain()

    def _drain(self):
        """Flush any messages left queued in the instrument's output buffer by a
        previously interrupted transfer (e.g. a screen dump killed mid-read).
        USBTMC clear() doesn't always empty it, so we read-and-discard until idle."""
        try:
            self.inst.clear()
        except Exception:  # noqa: BLE001
            pass
        old = self.inst.timeout
        self.inst.timeout = 400
        try:
            for _ in range(10):
                try:
                    self.inst.read_raw()  # consume one stuck message
                except Exception:  # noqa: BLE001 -- timeout => buffer empty
                    break
        finally:
            self.inst.timeout = old

    def close(self):
        with self.lock:
            if self.inst is not None:
                try:
                    self.inst.close()
                except Exception:  # noqa: BLE001
                    pass
                self.inst = None

    def reconnect(self):
        """Close and reopen the link; used to recover from a dropped/wedged USB."""
        with self.lock:
            self.close()
            time.sleep(0.5)
            self._open()

    # -- primitives (lock held by callers via the public methods) --

    def query(self, cmd: str) -> str:
        with self.lock:
            try:
                return self.inst.query(cmd).strip()
            except Exception:  # noqa: BLE001 -- resync the USBTMC buffer after a failed read
                self._drain()
                raise

    def write(self, cmd: str):
        with self.lock:
            self.inst.write(cmd)

    def screen_bmp(self) -> bytes:
        with self.lock:
            self.inst.write("SCDP")
            data = self.inst.read_raw()
        if not data.startswith(b"BM"):
            raise RuntimeError(
                f"Unexpected screen-dump response (first bytes: {data[:8]!r})."
            )
        return data

    def measure(self, channels=("C1", "C2"), params=DEFAULT_PARAMS,
                only_displayed: bool = False) -> dict:
        """Measure each channel. With only_displayed=True, skip channels whose trace
        is off -- important because querying Cx:PAVA? auto-enables the channel, so
        polling must not touch channels the user has turned off."""
        out: dict[str, dict[str, float | None]] = {}
        with self.lock:
            for ch in channels:
                if only_displayed and not self.inst.query(f"{ch}:TRA?").strip().upper().endswith("ON"):
                    continue
                vals: dict[str, float | None] = {}
                for p in params:
                    try:
                        vals[p] = parse_pava(self.inst.query(f"{ch}:PAVA? {p}"))
                    except Exception:  # noqa: BLE001 -- resync on a timed-out read
                        self._drain()
                        vals[p] = None
                out[ch] = vals
        return out

    def state(self) -> dict:
        """Read back the full front-panel state: channels, timebase, trigger, acq."""
        def q(cmd: str) -> str:
            try:
                return self.inst.query(cmd).strip()
            except Exception:  # noqa: BLE001 -- a timed-out read orphans a response in the
                self._drain()  # output buffer, desyncing every later query; resync the link
                return ""

        chans: dict[str, dict] = {}
        with self.lock:
            for ch in ("C1", "C2"):
                chans[ch] = {
                    "on": q(f"{ch}:TRA?").upper().endswith("ON"),
                    "vdiv": query_number(q(f"{ch}:VDIV?")),
                    "ofst": query_number(q(f"{ch}:OFST?")),
                    "cpl": _after(q(f"{ch}:CPL?")),
                    "attn": query_number(q(f"{ch}:ATTN?")),
                    "invert": q(f"{ch}:INVS?").upper().endswith("ON"),
                    "unit": _after(q(f"{ch}:UNIT?")),
                }
            bwl_raw = _after(q("BWL?"))             # "C1,OFF,C2,ON"
            tdiv = query_number(q("TDIV?"))
            trdl = query_number(q("TRDL?"))
            trse = _after(q("TRSE?"))               # "EDGE,SR,C1,HT,OFF"
            trmd = _after(q("TRMD?"))
            acqw = _after(q("ACQW?"))
            sara = query_number(q("SARA?"))
            sast = _after(q("SAST?"))
            toks = [t for t in trse.split(",") if t]
            ttype = toks[0] if toks else ""
            src = toks[toks.index("SR") + 1] if "SR" in toks else "C1"
            tlev = query_number(q(f"{src}:TRLV?"))
            tslp = _after(q(f"{src}:TRSL?"))
            tcpl = _after(q(f"{src}:TRCP?"))

        bw = bwl_raw.split(",")
        for i in range(0, len(bw) - 1, 2):
            if bw[i] in chans:
                chans[bw[i]]["bwl"] = bw[i + 1].upper() == "ON"
        for ch in chans:
            chans[ch].setdefault("bwl", False)

        return {
            "channels": chans,
            "timebase": {"tdiv": tdiv, "trdl": trdl},
            "trigger": {"type": ttype, "source": src, "slope": tslp,
                        "level": tlev, "coupling": tcpl, "mode": trmd},
            "acq": {"mode": acqw, "sara": sara, "status": sast},
        }

    def waveform(self, channel: str = "C1", points: int = 0) -> dict:
        """Return real sample data: {'channel','t':[...],'v':[...],'meta':{...}}.

        Converts the raw 8-bit ADC bytes to volts via VDIV/OFST and to seconds via
        the sample rate and timebase, the standard SDS1000X-E procedure."""
        with self.lock:
            vdiv = query_number(self.inst.query(f"{channel}:VDIV?")) or 0.0
            ofst = query_number(self.inst.query(f"{channel}:OFST?")) or 0.0
            tdiv = query_number(self.inst.query("TDIV?")) or 0.0
            sara = query_number(self.inst.query("SARA?")) or 1.0
            if points > 0:
                self.inst.write(f"WFSU SP,0,NP,{points},FP,0")
            self.inst.write(f"{channel}:WF? DAT2")
            raw = self.inst.read_raw()

        # IEEE-488.2 definite-length block: ...#<n><n-digit length><data>
        hash_i = raw.find(b"#")
        if hash_i < 0:
            raise RuntimeError("waveform response has no data block")
        ndig = int(raw[hash_i + 1:hash_i + 2])
        nbytes = int(raw[hash_i + 2:hash_i + 2 + ndig])
        start = hash_i + 2 + ndig
        data = raw[start:start + nbytes]
        codes = struct.unpack(f"{len(data)}b", data)  # signed int8

        scale = vdiv / CODES_PER_DIV
        half = tdiv * HGRID / 2.0
        dt = 1.0 / sara if sara else 0.0
        volts = [c * scale - ofst for c in codes]
        times = [-half + i * dt for i in range(len(codes))]
        return {
            "channel": channel,
            "t": times,
            "v": volts,
            "meta": {"vdiv": vdiv, "ofst": ofst, "tdiv": tdiv, "sara": sara,
                     "points": len(codes)},
        }


    def cursor(self, src: str = "C1") -> dict:
        """Read cursor mode/type and the manual-cursor readout for one channel.
        HREL = [type, dT, 1/dT, X1, X2];  VREL = [type, dV, Y1, Y2]."""
        def q(cmd: str) -> str:
            try:
                return self.inst.query(cmd).strip()
            except Exception:  # noqa: BLE001 -- a timed-out read orphans a response in the
                self._drain()  # output buffer, desyncing every later query; resync the link
                return ""

        with self.lock:
            mode = _after(q("CRMS?"))
            typ = _after(q("CRTY?"))
            h = _after(q(f"{src}:CRVA? HREL")).split(",")
            v = _after(q(f"{src}:CRVA? VREL")).split(",")

        def g(arr, i):
            return parse_value(arr[i]) if len(arr) > i else None

        return {
            "mode": mode, "type": typ, "src": src,
            "dt": g(h, 1), "freq": g(h, 2), "x1": g(h, 3), "x2": g(h, 4),
            "dv": g(v, 1), "y1": g(v, 2), "y2": g(v, 3),
        }


def tailscale_ip() -> str | None:
    """Best-effort lookup of this machine's Tailscale IPv4 address."""
    for cli in ("tailscale", "/Applications/Tailscale.app/Contents/MacOS/Tailscale"):
        try:
            out = subprocess.run([cli, "ip", "-4"], capture_output=True, text=True, timeout=5)
        except (FileNotFoundError, subprocess.SubprocessError):
            continue
        lines = out.stdout.strip().splitlines()
        if lines and lines[0].strip():
            return lines[0].strip()
    return None


# ---------------------------------------------------------------------------
# image helpers
# ---------------------------------------------------------------------------

def bmp_to_png(bmp: bytes) -> bytes:
    from PIL import Image

    out = io.BytesIO()
    Image.open(io.BytesIO(bmp)).save(out, format="PNG")
    return out.getvalue()


def bmp_to_jpeg(bmp: bytes, quality: int = 85) -> bytes:
    from PIL import Image

    out = io.BytesIO()
    Image.open(io.BytesIO(bmp)).convert("RGB").save(out, format="JPEG", quality=quality)
    return out.getvalue()


# ---------------------------------------------------------------------------
# subcommands
# ---------------------------------------------------------------------------

def cmd_list(_args) -> int:
    rm = pyvisa.ResourceManager("@py")
    resources = rm.list_resources()
    if not resources:
        print("No VISA resources found. Plug in the scope, power it on, and try again.")
        return 1
    print("Detected VISA resources:")
    for r in resources:
        tag = "  <-- looks like Siglent" if _is_siglent(r) else ""
        print(f"  {r}{tag}")
    res = find_scope_resource(rm)
    if res:
        try:
            inst = rm.open_resource(res)
            inst.timeout = 3000
            print(f"\n*IDN? -> {inst.query('*IDN?').strip()}")
            inst.close()
        except Exception as e:  # noqa: BLE001
            print(f"\nFound {res} but could not query *IDN?: {e}")
    return 0


def cmd_shot(args) -> int:
    scope = Scope(args.resource)
    print(f"Connected: {scope.resource}")
    bmp = scope.screen_bmp()
    scope.close()
    out = Path(args.output) if args.output else Path(default_name("png"))
    out.write_bytes(bmp if out.suffix.lower() == ".bmp" else bmp_to_png(bmp))
    print(f"Saved {out}  ({out.stat().st_size} bytes)")
    return 0


def cmd_measure(args) -> int:
    scope = Scope(args.resource)
    # With no -c, measure only channels already displayed (querying PAVA on an off
    # channel would switch it on). An explicit -c means "measure this one regardless".
    channels = args.channel or ["C1", "C2"]
    data = scope.measure(channels=channels, only_displayed=not args.channel)
    scope.close()
    if args.json:
        import json
        print(json.dumps(data, indent=2))
        return 0
    for ch, vals in data.items():
        print(f"\n{ch}:")
        for p, v in vals.items():
            print(f"  {p:6} {'--' if v is None else f'{v:.6g}'}")
    return 0


def cmd_wave(args) -> int:
    scope = Scope(args.resource)
    print(f"Connected: {scope.resource}")
    wf = scope.waveform(channel=args.channel, points=args.points)
    scope.close()
    n = wf["meta"]["points"]
    print(f"{args.channel}: {n} samples  "
          f"(VDIV={wf['meta']['vdiv']:g}V  TDIV={wf['meta']['tdiv']:g}s  "
          f"SARA={wf['meta']['sara']:g}Sa/s)")
    out = Path(args.output) if args.output else Path(default_name("csv"))
    if out.suffix.lower() == ".png":
        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
        except ImportError:
            print("matplotlib not installed; run: uv add matplotlib", file=sys.stderr)
            return 1
        fig, ax = plt.subplots(figsize=(10, 5))
        ax.plot([t * 1e6 for t in wf["t"]], wf["v"], lw=0.8, color="#c8a000")
        ax.set_xlabel("time (us)")
        ax.set_ylabel("volts")
        ax.set_title(f"{args.channel}  {n} samples")
        ax.grid(alpha=0.3)
        fig.tight_layout()
        fig.savefig(out, dpi=120)
    else:
        with out.open("w") as f:
            f.write("time_s,volt_V\n")
            for t, v in zip(wf["t"], wf["v"]):
                f.write(f"{t:.9e},{v:.6e}\n")
    print(f"Saved {out}")
    return 0


def cmd_state(args) -> int:
    scope = Scope(args.resource)
    st = scope.state()
    scope.close()
    if args.json:
        import json
        print(json.dumps(st, indent=2))
        return 0
    for ch, d in st["channels"].items():
        on = "ON " if d["on"] else "off"
        print(f"{ch} [{on}] VDIV={d['vdiv']:g}V OFST={d['ofst']:g}V CPL={d['cpl']} "
              f"PROBE={d['attn']:g}x BWL={'on' if d['bwl'] else 'off'} "
              f"INV={'on' if d['invert'] else 'off'}")
    tb, tr, ac = st["timebase"], st["trigger"], st["acq"]
    print(f"TIMEBASE TDIV={tb['tdiv']:g}s DELAY={tb['trdl']:g}s")
    print(f"TRIGGER  {tr['type']} src={tr['source']} slope={tr['slope']} "
          f"level={tr['level']}V coupling={tr['coupling']} mode={tr['mode']}")
    print(f"ACQUIRE  mode={ac['mode']} rate={ac['sara']:g}Sa/s status={ac['status']}")
    return 0


def cmd_cmd(args) -> int:
    scope = Scope(args.resource)
    text = args.command.strip()
    if "?" in text:
        print(scope.query(text))
    else:
        scope.write(text)
        print("ok")
    scope.close()
    return 0


def cmd_mirror(args) -> int:
    import tkinter as tk
    from PIL import Image, ImageTk

    scope = Scope(args.resource)
    print(f"Connected: {scope.resource}")
    print("Live mirror:  s = save PNG   q / Esc = quit")

    root = tk.Tk()
    root.title("SDS1202X-E  (s=save  q=quit)")
    root.configure(bg="black")
    label = tk.Label(root, bg="black")
    label.pack()
    status = tk.Label(root, bg="black", fg="#39ff14", anchor="w", font=("Menlo", 11))
    status.pack(fill="x")

    state = {"bmp": None, "photo": None, "frames": 0, "running": True, "t0": time.time()}
    lock = threading.Lock()

    def fetch_loop():
        while state["running"]:
            try:
                bmp = scope.screen_bmp()
                with lock:
                    state["bmp"] = bmp
                    state["frames"] += 1
            except Exception as e:  # noqa: BLE001
                with lock:
                    state["error"] = str(e)
                time.sleep(0.5)

    threading.Thread(target=fetch_loop, daemon=True).start()

    def redraw():
        if not state["running"]:
            return
        with lock:
            bmp, frames, err = state["bmp"], state["frames"], state.get("error")
        if bmp:
            img = Image.open(io.BytesIO(bmp))
            photo = ImageTk.PhotoImage(img)
            label.configure(image=photo)
            state["photo"] = photo
            fps = frames / max(time.time() - state["t0"], 1e-6)
            status.configure(text=f" {img.width}x{img.height}   {fps:4.1f} fps   frame {frames}",
                             fg="#39ff14")
        elif err:
            status.configure(text=f" {err}", fg="#ff5555")
        root.after(150, redraw)

    def save(_evt=None):
        with lock:
            bmp = state["bmp"]
        if not bmp:
            return
        out = Path(default_name("png"))
        out.write_bytes(bmp_to_png(bmp))
        status.configure(text=f" saved {out}", fg="#ffff55")
        print(f"Saved {out}")

    def quit_(_evt=None):
        state["running"] = False
        root.after(50, root.destroy)

    root.bind("s", save)
    root.bind("q", quit_)
    root.bind("<Escape>", quit_)
    root.protocol("WM_DELETE_WINDOW", quit_)
    redraw()
    root.mainloop()
    state["running"] = False
    scope.close()
    return 0


# control actions exposed on the web page -> SCPI commands
CONTROL_ACTIONS = {
    "run": "TRMD AUTO",
    "stop": "STOP",
    "single": "TRMD SINGLE",
    "auto": "ASET",
    "force": "FRTR",
}

PAGE = b"""<!doctype html><html><head><meta charset=utf-8>
<meta name=viewport content="width=device-width,initial-scale=1">
<title>SDS1202X-E</title><style>
body{margin:0;background:#111;color:#ddd;font:13px Menlo,monospace;text-align:center}
#scr{max-width:100%;height:auto;image-rendering:pixelated;border-bottom:1px solid #333}
.bar{padding:6px}
button,a.btn{margin:2px;padding:6px 11px;background:#222;color:#39ff14;cursor:pointer;
text-decoration:none;border:1px solid #39ff14;border-radius:4px;font:13px Menlo,monospace}
button:hover,a.btn:hover{background:#39ff14;color:#111}
button.on{background:#39ff14;color:#111}
button.off{color:#666;border-color:#666}
select,input{background:#000;color:#39ff14;border:1px solid #444;border-radius:3px;
font:12px Menlo;padding:3px}
#meas{display:flex;flex-wrap:wrap;justify-content:center;gap:6px 16px;padding:6px;color:#9cf}
#meas .ch{border:1px solid #224;border-radius:4px;padding:4px 8px}
#panels{display:flex;flex-wrap:wrap;justify-content:center;gap:10px;padding:6px}
.panel{border:1px solid #333;border-radius:6px;padding:8px 10px;min-width:200px;text-align:left}
.panel h3{margin:0 0 6px;font-size:13px;color:#9cf}
.panel.c1 h3{color:#ffd24a}.panel.c2 h3{color:#4ad2ff}
.row{margin:3px 0}.row label{display:inline-block;width:58px;color:#aaa}
#scpi{width:55%;background:#000;color:#39ff14;border:1px solid #444;padding:6px;font:13px Menlo}
#out{color:#ff9;min-height:1em;padding:4px}
</style></head><body>
<img id=scr src="/stream" alt="scope">
<div class=bar>
<button onclick="act('run')">Run</button>
<button onclick="act('stop')">Stop</button>
<button onclick="act('single')">Single</button>
<button onclick="act('auto')">Auto Setup</button>
<button onclick="act('force')">Force Trig</button>
<a class=btn href="/snapshot.jpg" download>Save PNG</a>
</div>
<div id=meas></div>
<div id=panels></div>
<div class=bar>
<input id=scpi placeholder="raw SCPI e.g. C1:VDIV? or TDIV 1E-3" onkeydown="if(event.key=='Enter')scpi()">
<button onclick="scpi()">Send</button>
</div>
<div id=out></div>
<script>
var S={};  // latest state
// Track which control the user just touched, so background polling doesn't fight
// them by overwriting a selection mid-interaction or before the scope reflects it.
var touched={};
document.addEventListener('change',function(e){if(e.target&&e.target.id)touched[e.target.id]=Date.now()},true);
document.addEventListener('focus',function(e){if(e.target&&e.target.id)touched[e.target.id]=Date.now()},true);
function fresh(id){return touched[id]&&(Date.now()-touched[id]<2500)}
function post(cmd){return fetch('/scpi?cmd='+encodeURIComponent(cmd))
  .then(r=>r.text()).then(t=>{out.textContent=cmd+' -> '+t;return refresh()})}
function act(a){fetch('/control?action='+a).then(r=>r.text()).then(t=>{out.textContent=t;refresh()})}
function scpi(){var c=document.getElementById('scpi').value;if(!c)return;post(c)}

var VDIVS=[5e-4,1e-3,2e-3,5e-3,0.01,0.02,0.05,0.1,0.2,0.5,1,2,5,10];
function tdivs(){var r=[],m=[1,2,5];for(var e=-9;e<=1;e++)for(var b of m){var v=b*Math.pow(10,e);if(v<=50)r.push(+v.toPrecision(2))}return r}
var TDIVS=tdivs();
var CPLS=[['D1M','DC 1M'],['A1M','AC 1M'],['GND','GND']];
var ATTNS=[0.1,1,10,100,1000];
var TSRC=['C1','C2','EX','EX5','LINE'];
var TSLP=[['POS','Rising'],['NEG','Falling']];
var TMODE=[['AUTO','Auto'],['NORM','Normal'],['SINGLE','Single']];
var TCPL=['DC','AC','HFREJ','LFREJ'];
var ACQ=[['SAMPLING','Sample'],['PEAK_DETECT','Peak'],['AVERAGE','Average'],['HIGH_RES','HiRes']];

function engV(v){var a=Math.abs(v);if(a>=1)return v+' V';if(a>=1e-3)return +(v*1e3).toPrecision(3)+' mV';return +(v*1e6).toPrecision(3)+' uV'}
function engT(v){var a=Math.abs(v);if(a>=1)return v+' s';if(a>=1e-3)return +(v*1e3).toPrecision(3)+' ms';if(a>=1e-6)return +(v*1e6).toPrecision(3)+' us';return +(v*1e9).toPrecision(3)+' ns'}
function eng(v){return v==null?'--':(Math.abs(v)>=1e6?+(v/1e6).toPrecision(4)+'M':Math.abs(v)>=1e3?+(v/1e3).toPrecision(4)+'k':+v.toPrecision(4))}
function near(v,arr){var b=arr[0];for(var x of arr)if(Math.abs(x-v)<Math.abs(b-v))b=x;return b}
function opts(arr,sel){return arr.map(function(o){var v=Array.isArray(o)?o[0]:o,l=Array.isArray(o)?o[1]:o;
  return '<option value="'+v+'"'+(String(v)==String(sel)?' selected':'')+'>'+l+'</option>'}).join('')}

function chanPanel(ch){var c=ch.toLowerCase();return ''+
 '<div class="panel '+c+'"><h3>'+ch+' <button id="'+c+'_on" onclick="tra(\\''+ch+'\\')">--</button></h3>'+
 '<div class=row><label>V/div</label><select id="'+c+'_vdiv" onchange="post(\\''+ch+':VDIV \\'+this.value+\\'V\\')">'+
   opts(VDIVS.map(function(v){return [v,engV(v)]}))+'</select></div>'+
 '<div class=row><label>Offset</label><input id="'+c+'_ofst" size=6 onchange="post(\\''+ch+':OFST \\'+this.value+\\'V\\')"> V</div>'+
 '<div class=row><label>Couple</label><select id="'+c+'_cpl" onchange="post(\\''+ch+':CPL \\'+this.value)">'+opts(CPLS)+'</select></div>'+
 '<div class=row><label>Probe</label><select id="'+c+'_attn" onchange="post(\\''+ch+':ATTN \\'+this.value)">'+opts(ATTNS.map(function(v){return [v,v+'x']}))+'</select></div>'+
 '<div class=row><label>BW 20M</label><button id="'+c+'_bwl" onclick="bwl(\\''+ch+'\\')">--</button>'+
   ' <label style=width:auto> Inv</label> <button id="'+c+'_inv" onclick="inv(\\''+ch+'\\')">--</button></div>'+
 '</div>'}

function fixedPanels(){return ''+
 '<div class=panel><h3>Timebase</h3>'+
 '<div class=row><label>T/div</label><select id=tdiv onchange="post(\\'TDIV \\'+this.value+\\'S\\')">'+
   opts(TDIVS.map(function(v){return [v,engT(v)]}))+'</select></div>'+
 '<div class=row><label>Delay</label><input id=trdl size=8 onchange="post(\\'TRDL \\'+this.value+\\'S\\')"> s</div>'+
 '</div>'+
 '<div class=panel><h3>Trigger</h3>'+
 '<div class=row><label>Source</label><select id=tsrc onchange="post(\\'TRSE EDGE,SR,\\'+this.value+\\',HT,OFF\\')">'+opts(TSRC)+'</select></div>'+
 '<div class=row><label>Slope</label><select id=tslp onchange="post(S.trigger.source+\\':TRSL \\'+this.value)">'+opts(TSLP)+'</select></div>'+
 '<div class=row><label>Level</label><input id=tlev size=6 onchange="post(S.trigger.source+\\':TRLV \\'+this.value+\\'V\\')"> V</div>'+
 '<div class=row><label>Couple</label><select id=tcpl onchange="post(S.trigger.source+\\':TRCP \\'+this.value)">'+opts(TCPL)+'</select></div>'+
 '<div class=row><label>Mode</label><select id=tmode onchange="post(\\'TRMD \\'+this.value)">'+opts(TMODE)+'</select></div>'+
 '</div>'+
 '<div class=panel><h3>Acquire</h3>'+
 '<div class=row><label>Mode</label><select id=acqw onchange="post(\\'ACQW \\'+this.value)">'+opts(ACQ)+'</select></div>'+
 '<div class=row><label>Rate</label><span id=sara>--</span></div>'+
 '<div class=row><label>Status</label><span id=sast>--</span></div>'+
 '</div>'+
 '<div class=panel><h3>Cursors</h3>'+
 '<div class=row><label>Mode</label><select id=crms onchange="post(\\'CRMS \\'+this.value)">'+opts([['OFF','Off'],['MANUAL','Manual'],['TRACK','Track']])+'</select>'+
   ' <select id=crsrc onchange="cursorSrc=this.value;pollCursor1()">'+opts(['C1','C2'])+'</select></div>'+
 '<div class=row><label>Type</label><select id=crty onchange="post(\\'CRTY \\'+this.value)">'+opts([['X','Vertical (time)'],['Y','Horizontal (volts)'],['X-Y','Both']])+'</select></div>'+
 '<div class=row><label>X1</label><input id=crx1 size=7 onchange="post(cursorSrc+\\':CRST TREF,\\'+this.value+\\'US\\')"> &micro;s'+
   ' <label style=width:auto>X2</label> <input id=crx2 size=7 onchange="post(cursorSrc+\\':CRST TDIF,\\'+this.value+\\'US\\')"> &micro;s</div>'+
 '<div class=row><label>Y1</label><input id=cry1 size=7 onchange="post(cursorSrc+\\':CRST VREF,\\'+this.value+\\'V\\')"> V'+
   ' <label style=width:auto>Y2</label> <input id=cry2 size=7 onchange="post(cursorSrc+\\':CRST VDIF,\\'+this.value+\\'V\\')"> V</div>'+
 '<div class=row id=crrd style=color:#9cf>&Delta;--</div>'+
 '</div>'+
 '<div class=panel><h3>Setups</h3>'+
 '<div class=row><label>Slot</label><select id=slot>'+opts([1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20])+'</select></div>'+
 '<div class=row><button onclick="post(\\'*SAV \\'+gv(\\'slot\\'))">Save</button>'+
   '<button onclick="post(\\'*RCL \\'+gv(\\'slot\\'))">Recall</button>'+
   '<button onclick="if(confirm(\\'Reset scope to factory defaults?\\'))post(\\'*RST\\')">Reset</button></div>'+
 '</div>'}

function build(){document.getElementById('panels').innerHTML=chanPanel('C1')+chanPanel('C2')+fixedPanels()}

function tog(id,on){var b=document.getElementById(id);if(!b)return;
  b.textContent=on?'ON':'OFF';b.className=on?'on':'off'}
function setSel(id,v){var e=document.getElementById(id);if(!e||v==null)return;
  if(document.activeElement===e||fresh(id))return;
  if(String(e.value)!=String(v))e.value=v}
function gv(id){return document.getElementById(id).value}

function apply(s){S=s;
 ['C1','C2'].forEach(function(ch){var c=ch.toLowerCase(),d=s.channels[ch];if(!d)return;
   tog(c+'_on',d.on);tog(c+'_bwl',d.bwl);tog(c+'_inv',d.invert);
   if(d.vdiv!=null)setSel(c+'_vdiv',near(d.vdiv,VDIVS));
   var o=document.getElementById(c+'_ofst');if(o&&document.activeElement!=o)o.value=d.ofst;
   setSel(c+'_cpl',d.cpl);if(d.attn!=null)setSel(c+'_attn',near(d.attn,ATTNS))});
 if(s.timebase.tdiv!=null)setSel('tdiv',near(s.timebase.tdiv,TDIVS));
 var td=document.getElementById('trdl');if(td&&document.activeElement!=td)td.value=s.timebase.trdl;
 setSel('tsrc',s.trigger.source);setSel('tslp',s.trigger.slope);setSel('tcpl',s.trigger.coupling);
 setSel('tmode',s.trigger.mode);
 var tl=document.getElementById('tlev');if(tl&&document.activeElement!=tl)tl.value=s.trigger.level;
 setSel('acqw',(s.acq.mode||'').split(',')[0]);
 document.getElementById('sara').textContent=eng(s.acq.sara)+'Sa/s';
 document.getElementById('sast').textContent=s.acq.status||'--'}

function refresh(){return fetch('/state').then(r=>r.json()).then(apply).catch(function(){})}
function tra(ch){post(ch+':TRA '+(S.channels[ch]&&S.channels[ch].on?'OFF':'ON'))}
function bwl(ch){post('BWL '+ch+','+(S.channels[ch]&&S.channels[ch].bwl?'OFF':'ON'))}
function inv(ch){post(ch+':INVS '+(S.channels[ch]&&S.channels[ch].invert?'OFF':'ON'))}

async function pollMeas(){try{var d=await (await fetch('/measure')).json();var h='';
 for(var ch in d){h+='<div class=ch><b>'+ch+'</b> ';
  for(var p in d[ch]){var v=d[ch][p];h+=p+'='+(v==null?'--':(+v).toPrecision(4))+' '}h+='</div>'}
 document.getElementById('meas').innerHTML=h}catch(e){}setTimeout(pollMeas,1500)}

var cursorSrc='C1';
function setIf(id,v){var e=document.getElementById(id);if(!e||v==null)return;
  if(document.activeElement===e||fresh(id))return;e.value=v}
function setUs(id,v){setIf(id,v==null?null:+(v*1e6).toPrecision(6))}
async function pollCursor1(){try{var d=await (await fetch('/cursor?src='+cursorSrc)).json();
 setSel('crms',d.mode=='MANU'?'MANUAL':d.mode);setSel('crty',d.type);
 setUs('crx1',d.x1);setUs('crx2',d.x2);setIf('cry1',d.y1);setIf('cry2',d.y2);
 var tt=d.dt==null?'--':engT(d.dt),ff=eng(d.freq),vv=d.dv==null?'--':(+d.dv).toPrecision(4)+'V';
 document.getElementById('crrd').innerHTML='&Delta;T='+tt+'&nbsp; 1/&Delta;T='+ff+'Hz&nbsp; &Delta;V='+vv}catch(e){}}
function pollCursor(){pollCursor1();setTimeout(pollCursor,1500)}

build();refresh();pollMeas();pollCursor();setInterval(refresh,5000);
</script>
</body></html>"""


def cmd_serve(args) -> int:
    from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
    from urllib.parse import urlparse, parse_qs

    scope = Scope(args.resource)
    print(f"Connected: {scope.resource}")

    frame = {"jpeg": None, "n": 0, "err": None}
    cond = threading.Condition()
    interval = max(args.interval, 0.0)
    stop = threading.Event()

    def grabber():
        fails = 0
        while not stop.is_set():
            t = time.time()
            try:
                jpeg = bmp_to_jpeg(scope.screen_bmp(), quality=args.quality)
                with cond:
                    frame["jpeg"], frame["err"] = jpeg, None
                    frame["n"] += 1
                    cond.notify_all()
                fails = 0
            except Exception as e:  # noqa: BLE001
                with cond:
                    frame["err"] = str(e)
                fails += 1
                if fails >= 3:  # link likely dropped; try to recover
                    try:
                        scope.reconnect()
                        fails = 0
                    except Exception:  # noqa: BLE001
                        pass
                time.sleep(0.5)
            dt = interval - (time.time() - t)
            if dt > 0:
                stop.wait(dt)

    threading.Thread(target=grabber, daemon=True).start()

    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, *a):
            pass

        def _send(self, body: bytes, ctype: str, extra: dict | None = None):
            self.send_response(200)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            for k, v in (extra or {}).items():
                self.send_header(k, v)
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            u = urlparse(self.path)
            q = parse_qs(u.query)
            path = u.path

            if path in ("/", "/index.html"):
                return self._send(PAGE, "text/html; charset=utf-8")

            if path == "/favicon.ico":
                self.send_response(204)
                self.end_headers()
                return

            if path.startswith("/snapshot"):
                with cond:
                    jpeg = frame["jpeg"]
                if not jpeg:
                    return self.send_error(503, "no frame yet")
                return self._send(jpeg, "image/jpeg",
                                  {"Content-Disposition": f'attachment; filename="{default_name("jpg")}"'})

            if path == "/measure":
                import json
                try:
                    data = scope.measure(only_displayed=True)
                    return self._send(json.dumps(data).encode(), "application/json")
                except Exception as e:  # noqa: BLE001
                    return self._send(json.dumps({"error": str(e)}).encode(),
                                      "application/json")

            if path == "/cursor":
                import json
                src = (q.get("src") or ["C1"])[0]
                try:
                    return self._send(json.dumps(scope.cursor(src)).encode(), "application/json")
                except Exception as e:  # noqa: BLE001
                    return self._send(json.dumps({"error": str(e)}).encode(),
                                      "application/json")

            if path == "/state":
                import json
                try:
                    return self._send(json.dumps(scope.state()).encode(), "application/json")
                except Exception as e:  # noqa: BLE001
                    return self._send(json.dumps({"error": str(e)}).encode(),
                                      "application/json")

            if path == "/control":
                action = (q.get("action") or [""])[0]
                cmd = CONTROL_ACTIONS.get(action)
                if not cmd:
                    return self._send(b"unknown action", "text/plain")
                try:
                    scope.write(cmd)
                    return self._send(f"{action} -> {cmd}".encode(), "text/plain")
                except Exception as e:  # noqa: BLE001
                    return self._send(f"error: {e}".encode(), "text/plain")

            if path == "/scpi":
                cmd = (q.get("cmd") or [""])[0].strip()
                if not cmd:
                    return self._send(b"empty", "text/plain")
                try:
                    if "?" in cmd:
                        return self._send(scope.query(cmd).encode(), "text/plain")
                    scope.write(cmd)
                    return self._send(f"sent: {cmd}".encode(), "text/plain")
                except Exception as e:  # noqa: BLE001
                    return self._send(f"error: {e}".encode(), "text/plain")

            if path.startswith("/stream"):
                self.send_response(200)
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.send_header("Cache-Control", "no-cache")
                self.end_headers()
                last_n = -1
                try:
                    while not stop.is_set():
                        with cond:
                            cond.wait_for(lambda: frame["n"] != last_n, timeout=10)
                            jpeg, last_n = frame["jpeg"], frame["n"]
                        if not jpeg:
                            continue
                        self.wfile.write(b"--frame\r\n")
                        self.wfile.write(b"Content-Type: image/jpeg\r\n")
                        self.wfile.write(f"Content-Length: {len(jpeg)}\r\n\r\n".encode())
                        self.wfile.write(jpeg)
                        self.wfile.write(b"\r\n")
                except (BrokenPipeError, ConnectionResetError):
                    return
                return

            self.send_error(404)

    host = args.host
    if host is None:
        host = tailscale_ip()
        if not host:
            print("Could not detect a Tailscale IP; falling back to 127.0.0.1.\n"
                  "Pass --host to override (e.g. --host 0.0.0.0).", file=sys.stderr)
            host = "127.0.0.1"

    srv = ThreadingHTTPServer((host, args.port), Handler)
    srv.daemon_threads = True

    def shutdown(_sig=None, _frm=None):
        stop.set()
        threading.Thread(target=srv.shutdown, daemon=True).start()

    signal.signal(signal.SIGTERM, shutdown)
    signal.signal(signal.SIGINT, shutdown)

    print(f"Serving scope mirror at  http://{host}:{args.port}/")
    print("  live stream + measurements + Run/Stop/Auto/Force + SCPI box")
    print("Open that in a browser on the tailnet.  Ctrl-C (or SIGTERM) to stop.")
    srv.serve_forever()
    print("stopping; closing USB link cleanly.")
    scope.close()
    return 0


def main() -> int:
    p = argparse.ArgumentParser(description="Siglent SDS1202X-E USB tool for macOS")
    p.add_argument("-r", "--resource", help="explicit VISA resource (default: auto-detect)")
    sub = p.add_subparsers(dest="cmd", required=True)

    sub.add_parser("list", help="list detected VISA resources and identify the scope")

    s = sub.add_parser("shot", help="save one screenshot (PNG, or .bmp for raw)")
    s.add_argument("-o", "--output", help="output file (default: timestamped PNG)")

    sub.add_parser("mirror", help="local Tk mirror window (s=save, q=quit)")

    sv = sub.add_parser("serve", help="MJPEG web mirror + measurements + control")
    sv.add_argument("--host", help="bind address (default: this machine's Tailscale IP)")
    sv.add_argument("--port", type=int, default=8088, help="port (default: 8088)")
    sv.add_argument("--interval", type=float, default=0.3,
                    help="min seconds between frames (default: 0.3 ~= 3 fps)")
    sv.add_argument("--quality", type=int, default=85, help="JPEG quality 1-95 (default: 85)")

    m = sub.add_parser("measure", help="print live measurements")
    m.add_argument("-c", "--channel", action="append", help="channel(s), e.g. -c C1 -c C2")
    m.add_argument("--json", action="store_true", help="output JSON")

    w = sub.add_parser("wave", help="export real sample data (CSV, or .png for a plot)")
    w.add_argument("-c", "--channel", default="C1", help="channel (default: C1)")
    w.add_argument("-o", "--output", help="output .csv or .png (default: timestamped CSV)")
    w.add_argument("--points", type=int, default=0, help="cap sample count (0 = scope default)")

    st = sub.add_parser("state", help="print full front-panel state (channels/timebase/trigger)")
    st.add_argument("--json", action="store_true", help="output JSON")

    c = sub.add_parser("cmd", help="send a raw SCPI command (query if it contains '?')")
    c.add_argument("command", help='e.g. "C1:VDIV?" or "TDIV 1E-3"')

    args = p.parse_args()
    handlers = {
        "list": cmd_list, "shot": cmd_shot, "mirror": cmd_mirror, "serve": cmd_serve,
        "measure": cmd_measure, "wave": cmd_wave, "state": cmd_state, "cmd": cmd_cmd,
    }
    try:
        return handlers[args.cmd](args)
    except KeyboardInterrupt:
        return 130
    except RuntimeError as e:
        print(f"Error: {e}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
