#!/usr/bin/env python3
"""
tune_gui.py  --  live slider panel for the strategy brain.

A small Tkinter window (Python stdlib only). Each slider edits strategy_gains.json
in place; strategy_full.py hot-reloads that file, so moving a slider changes the
running sim within one control loop. The JSON stays the single source of truth and
ships to the real sub unchanged -- this panel is just a convenient way to write it.

Run it in a second terminal, alongside the sim:
    python tune_gui.py

macOS (Homebrew Python) note: if the import fails with "No module named '_tkinter'",
install Tk once ->  brew install python-tk@3.13   (match your Python version)
Linux note: sudo apt install python3-tk
"""

import json
import os
import tkinter as tk
from tkinter import ttk

GAINS_FILE = "strategy_gains.json"
GAINS_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), GAINS_FILE)

# key, label, min, max, resolution, description
PARAMS = [
    (
        "yaw_kp",
        "Yaw kp (turn)",
        0.0,
        0.01,
        0.00005,
        "How hard it turns to center the target left/right (higher = snappier).",
    ),
    (
        "heave_kp",
        "Heave kp (up/down)",
        0.0,
        0.01,
        0.00005,
        "How hard it dives/climbs to center the target up/down (higher = faster).",
    ),
    (
        "advance_surge",
        "Advance surge",
        0.0,
        1.0,
        0.01,
        "Forward speed while chasing the target (0-1).",
    ),
    (
        "orbit_strafe",
        "Orbit strafe",
        0.0,
        1.0,
        0.01,
        "Sideways speed while circling the target (0-1).",
    ),
    (
        "max_yaw_error_for_strafe",
        "Max yaw err for strafe",
        0.0,
        320.0,
        1.0,
        "Circling strafe ramps in as it gets within this many px of center.",
    ),
    (
        "orbit_enter_frac",
        "Orbit-enter (frac)",
        0.0,
        0.5,
        0.005,
        "Switch to orbiting once the target fills this share of the frame.",
    ),
    (
        "orbit_exit_ratio",
        "Orbit-exit (x enter)",
        0.0,
        1.0,
        0.01,
        "Go back to chasing if the target shrinks below this x the enter size.",
    ),
    (
        "orbit_to_win_s",
        "Orbit -> win (s)",
        0.0,
        30.0,
        0.5,
        "Seconds of continuous orbiting that count as a win.",
    ),
    (
        "celebrate_s",
        "Celebrate (s)",
        0.0,
        15.0,
        0.5,
        "Seconds spent flashing the light after a win.",
    ),
    (
        "grace_s",
        "Grace (s)",
        0.0,
        3.0,
        0.1,
        "Seconds to keep going after losing sight of the target before searching.",
    ),
    (
        "search_yaw_command",
        "Search spin rate",
        0.0,
        0.4,
        0.005,
        "Speed of the full-circle spin while searching. Raise for a faster sweep; "
        "lower if it whips past the target without locking on (esp. real-sub YOLO).",
    ),
]

DEFAULTS = {
    "yaw_kp": 0.0003,
    "heave_kp": 0.0003,
    "advance_surge": 0.8,
    "orbit_strafe": 0.8,
    "max_yaw_error_for_strafe": 80.0,
    "orbit_enter_frac": 0.05,
    "orbit_exit_ratio": 0.75,
    "orbit_to_win_s": 10.0,
    "celebrate_s": 5.0,
    "grace_s": 0.5,
    "search_yaw_command": 0.1,
}


def load_gains():
    try:
        with open(GAINS_PATH) as f:
            g = json.load(f)
    except (OSError, ValueError):
        g = {}
    out = dict(DEFAULTS)
    for k in out:
        v = g.get(k, out[k])
        if isinstance(v, (int, float)):
            out[k] = float(v)
    # legacy single-gain file: seed both kp from centering_kp
    if (
        "yaw_kp" not in g
        and "heave_kp" not in g
        and isinstance(g.get("centering_kp"), (int, float))
    ):
        out["yaw_kp"] = out["heave_kp"] = float(g["centering_kp"])
    return out


def fmt(key, v):
    if key in ("yaw_kp", "heave_kp"):
        return f"{v:.5f}"
    if key in ("search_yaw_command", "orbit_enter_frac"):
        return f"{v:.3f}"
    if key in ("advance_surge", "orbit_strafe", "orbit_exit_ratio", "grace_s"):
        return f"{v:.2f}"
    return f"{v:.1f}"


class TunerApp:
    def __init__(self, root):
        self.root = root
        root.title("Strategy gains")
        self.vars = {}
        self.value_labels = {}
        self._write_job = None
        self._ready = False

        vals = load_gains()
        frm = ttk.Frame(root, padding=10)
        frm.grid(sticky="nsew")

        row = 0
        for key, label, lo, hi, res, desc in PARAMS:
            ttk.Label(frm, text=label).grid(row=row, column=0, sticky="w", pady=(8, 0))
            var = tk.DoubleVar(value=vals[key])
            self.vars[key] = var
            tk.Scale(
                frm,
                variable=var,
                from_=lo,
                to=hi,
                resolution=res,
                orient="horizontal",
                length=240,
                showvalue=False,
                command=lambda _v, k=key: self._on_change(k),
            ).grid(row=row, column=1, padx=8, pady=(8, 0))
            vlab = ttk.Label(frm, text=fmt(key, vals[key]), width=8, anchor="e")
            vlab.grid(row=row, column=2, sticky="e", pady=(8, 0))
            self.value_labels[key] = vlab
            # description line underneath, spanning all columns so it can't skew widths
            ttk.Label(frm, text=desc, foreground="gray", wraplength=520).grid(
                row=row + 1, column=0, columnspan=3, sticky="w"
            )
            row += 2

        bar = ttk.Frame(frm, padding=(0, 12, 0, 0))
        bar.grid(row=row, column=0, columnspan=3, sticky="ew")
        ttk.Button(bar, text="Reload from file", command=self._reload).pack(side="left")
        ttk.Button(bar, text="Reset to defaults", command=self._reset).pack(
            side="left", padx=6
        )
        self.status = ttk.Label(frm, text=f"editing {GAINS_FILE}")
        self.status.grid(row=row + 1, column=0, columnspan=3, sticky="w", pady=(6, 0))

        self._ready = True

    def _on_change(self, key):
        if not self._ready:
            return
        self.value_labels[key].config(text=fmt(key, self.vars[key].get()))
        # debounce: collapse a slider drag into a single write ~150 ms after it stops
        if self._write_job is not None:
            self.root.after_cancel(self._write_job)
        self._write_job = self.root.after(150, self._write)

    def _write(self):
        self._write_job = None
        data = {k: round(v.get(), 6) for k, v in self.vars.items()}
        try:
            with open(GAINS_PATH, "w") as f:
                json.dump(data, f, indent=2)
            self.status.config(
                text=f"saved  yaw_kp={data['yaw_kp']:.5f}  heave_kp={data['heave_kp']:.5f}"
            )
        except OSError as e:
            self.status.config(text=f"write failed: {e}")

    def _reload(self):
        for k, v in load_gains().items():
            if k in self.vars:
                self.vars[k].set(v)
                self.value_labels[k].config(text=fmt(k, v))
        self.status.config(text="reloaded from file")

    def _reset(self):
        for k, v in DEFAULTS.items():
            self.vars[k].set(v)
            self.value_labels[k].config(text=fmt(k, v))
        self._write()
        self.status.config(text="reset to defaults")


if __name__ == "__main__":
    root = tk.Tk()
    TunerApp(root)
    root.mainloop()
