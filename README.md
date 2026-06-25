# siglent-scope

Screen mirror, data capture, and remote control for the Siglent SDS1202X-E
oscilloscope over USB, from macOS or Linux. No vendor software required.

The scope enumerates as a USBTMC device and speaks the SDS1000X-E SCPI dialect.
This tool talks to it through PyVISA's pure-Python backend and libusb, so nothing
in the path is Windows-only. It can grab the screen, pull real sample data, read
measurements, and drive the front-panel controls remotely.

## Features

- Live screen mirror in a browser (MJPEG), served over the network
- One-shot screenshots to PNG
- Waveform export to CSV, or a rendered plot (PNG)
- Live measurements (Vpp, frequency, RMS, and more)
- Remote control of channels, timebase, trigger, acquisition, and cursors
- Save and recall instrument setups
- A raw SCPI box for anything not exposed in the UI

## Requirements

- Python 3.11 or newer
- [uv](https://docs.astral.sh/uv/) for environment management
- libusb (`brew install libusb` on macOS, or your distribution's package)
- A Siglent SDS1202X-E (other SDS1000X-E models will likely work)

## Install

```
uv sync
```

## Usage

Connect the scope by USB, then:

```
uv run scope list                 # find the scope and print *IDN?
uv run scope shot -o screen.png   # save a screenshot
uv run scope measure              # measurements for the displayed channels
uv run scope wave -c C1 -o w.csv  # export channel 1 samples to CSV
uv run scope wave -c C1 -o w.png  # or render a plot
uv run scope state                # dump channel/timebase/trigger settings
uv run scope cmd "C1:VDIV?"       # send a raw SCPI command
uv run scope mirror               # local desktop mirror window (Tk)
```

### Web interface

```
uv run scope serve
```

This starts an HTTP server (default port 8088) that mirrors the screen live and
exposes the control panels. Open `http://<host>:8088/` in a browser. By default
it binds to the machine's Tailscale IP if one is found; pass `--host` to override
(for example `--host 0.0.0.0` for all interfaces, or `--host 127.0.0.1` for local
only).

```
--host HOST       bind address (default: Tailscale IP, else 127.0.0.1)
--port PORT       port (default: 8088)
--interval SEC    minimum seconds between frames (default: 0.3)
--quality N       JPEG quality 1-95 (default: 85)
```

The page has the live stream, a measurements strip, Run/Stop/Single/Auto/Force
buttons, a Save-PNG link, panels for each channel and for timebase, trigger,
acquisition, cursors, and save/recall, plus a raw SCPI input.

## Notes and limitations

- USBTMC allows only one connection at a time. While `serve` is running it holds
  the USB link, so the other CLI subcommands cannot run at the same time. Use the
  web SCPI box, or stop the server, for ad-hoc commands.
- Querying a channel's measurement (`Cx:PAVA?`) turns that channel's trace on, so
  measurement polling only touches channels that are already displayed.
- This firmware has no SCPI command to hide the math trace. You can define a math
  or FFT function from the SCPI box, but turning math off needs the front-panel
  Math key, so the web UI does not include a math panel.
- FFT window names on this model are abbreviated: RECT, HANN, HAMM, FLATTOP.
