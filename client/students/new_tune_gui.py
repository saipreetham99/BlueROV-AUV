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
        "approach_offset_px",
        "Approach offset (px)",
        -250.0,
        250.0,
        5.0,
        "While closing in, hold the target this many px off-centre instead of "
        "dead-centre, so you arrive at an angle rather than nose-to-nose. "
        "+ = target held right of centre (approach from its left). 0 = centred.",
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
        "orbit_hold_frac",
        "Orbit hold (frac)",
        0.0,
        0.5,
        0.005,
        "Box size (frac of frame) the orbit holds -- this sets the orbit radius. "
        "Keep it above orbit-exit or it'll bounce back to chasing.",
    ),
    (
        "orbit_surge_kp",
        "Orbit radius kp",
        0.0,
        40.0,
        0.5,
        "How hard it corrects distance to the hold size (higher = tighter radius). "
        "0 = no radius hold, so the orbit slowly spirals outward.",
    ),
    (
        "orbit_flip_s",
        "Orbit flip (s)",
        0.0,
        30.0,
        0.5,
        "Seconds of circling with no back in view before reversing orbit direction.",
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
    (
        "search_heave_command",
        "Search depth sweep",
        0.0,
        1.0,
        0.01,
        "Up/down bob amplitude while searching (0-1). 0 = spin flat, no depth sweep.",
    ),
    (
        "search_heave_period_s",
        "Search sweep period (s)",
        1.0,
        12.0,
        0.5,
        "Seconds for one full up-down depth-sweep cycle while searching.",
    ),
    # --- anti-stall: dash, reset maneuver, search timeout ---
    (
        "dash_s",
        "Dash time (s)",
        0.0,
        4.0,
        0.1,
        "After each orbit-direction flip, whip around at full strafe for this long "
        "to try to swing behind the target. 0 = no dash.",
    ),
    (
        "dash_strafe",
        "Dash strafe",
        0.0,
        1.0,
        0.01,
        "Strafe speed during that dash (0-1). Full speed = fastest way around.",
    ),
    (
        "stuck_reset_s",
        "Stuck-reset (s)",
        0.0,
        30.0,
        0.5,
        "Total time circling with no back in view (across both flips) before giving "
        "up and running the rise/forward/turn reset maneuver. Keep it well above "
        "orbit-flip so at least one direction flip happens first.",
    ),
    (
        "reset_up_s",
        "Reset: rise time (s)",
        0.0,
        5.0,
        0.1,
        "Reset phase 1 -- seconds spent rising (also lifts off the floor).",
    ),
    (
        "reset_heave",
        "Reset: rise speed",
        0.0,
        1.0,
        0.01,
        "Upward speed during the rise phase (0-1).",
    ),
    (
        "reset_fwd_s",
        "Reset: forward time (s)",
        0.0,
        5.0,
        0.1,
        "Reset phase 2 -- seconds driving forward (breaks a vertical stack).",
    ),
    (
        "reset_fwd_surge",
        "Reset: forward speed",
        0.0,
        1.0,
        0.01,
        "Forward speed during the forward phase (0-1).",
    ),
    (
        "reset_turn_s",
        "Reset: turn time (s)",
        0.0,
        5.0,
        0.1,
        "Reset phase 3 -- seconds spinning around to bring the target back in view.",
    ),
    (
        "reset_turn_yaw",
        "Reset: turn rate",
        0.0,
        1.0,
        0.01,
        "Turn rate during the spin-around (0-1). Higher = sharper turn.",
    ),
    (
        "search_timeout_s",
        "Search timeout (s)",
        0.0,
        20.0,
        0.5,
        "If searching drags on this long AFTER first contact, re-run the reset "
        "maneuver so it can't spin forever. The initial hunt is unaffected.",
    ),
]

DEFAULTS = {
    "yaw_kp": 0.0003,
    "heave_kp": 0.0003,
    "advance_surge": 0.8,
    "approach_offset_px": 80.0,
    "orbit_strafe": 0.8,
    "max_yaw_error_for_strafe": 80.0,
    "orbit_enter_frac": 0.05,
    "orbit_exit_ratio": 0.75,
    "orbit_hold_frac": 0.06,
    "orbit_surge_kp": 10.0,
    "orbit_flip_s": 6.0,
    "grace_s": 0.5,
    "search_yaw_command": 0.1,
    "search_heave_command": 0.2,
    "search_heave_period_s": 4.0,
    # anti-stall
    "dash_s": 1.5,
    "dash_strafe": 1.0,
    "stuck_reset_s": 14.0,
    "reset_up_s": 1.0,
    "reset_heave": 0.5,
    "reset_fwd_s": 1.0,
    "reset_fwd_surge": 0.6,
    "reset_turn_s": 1.0,
    "reset_turn_yaw": 0.8,
    "search_timeout_s": 8.0,
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
    if key in ("search_yaw_command", "orbit_enter_frac", "orbit_hold_frac"):
        return f"{v:.3f}"
    if key in (
        "advance_surge",
        "orbit_strafe",
        "orbit_exit_ratio",
        "grace_s",
        "search_heave_command",
        "dash_strafe",
        "reset_heave",
        "reset_fwd_surge",
        "reset_turn_yaw",
    ):
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

        # scrollable body: there are a lot of sliders now, so wrap the form in a
        # canvas + scrollbar and cap the window height so it never runs off-screen.
        outer = ttk.Frame(root)
        outer.grid(sticky="nsew")
        root.rowconfigure(0, weight=1)
        root.columnconfigure(0, weight=1)
        canvas = tk.Canvas(outer, borderwidth=0, highlightthickness=0)
        vsb = ttk.Scrollbar(outer, orient="vertical", command=canvas.yview)
        canvas.configure(yscrollcommand=vsb.set)
        vsb.pack(side="right", fill="y")
        canvas.pack(side="left", fill="both", expand=True)

        frm = ttk.Frame(canvas, padding=10)
        canvas.create_window((0, 0), window=frm, anchor="nw")
        frm.bind(
            "<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all")),
        )

        def _wheel(e):
            up = getattr(e, "delta", 0) > 0 or getattr(e, "num", 0) == 4
            canvas.yview_scroll(-1 if up else 1, "units")

        canvas.bind_all("<MouseWheel>", _wheel)  # Windows / macOS
        canvas.bind_all("<Button-4>", _wheel)  # Linux scroll up
        canvas.bind_all("<Button-5>", _wheel)  # Linux scroll down

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

        # size the viewport to the content, but never taller than the screen
        self.root.update_idletasks()
        want_w = frm.winfo_reqwidth() + vsb.winfo_reqwidth() + 4
        want_h = frm.winfo_reqheight() + 4
        max_h = self.root.winfo_screenheight() - 120
        canvas.configure(width=frm.winfo_reqwidth())
        self.root.geometry(f"{want_w}x{min(want_h, max_h)}")

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
