#!/usr/bin/env python3
"""
lap_optimizer.py — minimise lap time by tuning PID + SpeedController.

Usage:
    python3 lap_optimizer.py                              # CMA-ES, bane_fase2 (default)
    python3 lap_optimizer.py --track suzuka.png --mode cmaes --iterations 80
    python3 lap_optimizer.py --mode random --iterations 40
    python3 lap_optimizer.py --mode grid
"""

import os
import json
import copy
import numpy as np
from datetime import datetime
from PIL import ImageFilter

from track.image_loader import load_track_image
from sensors.qtr_array import QTRArray
from physics.robot_model import Robot
from control.pid_controller import PID, SpeedController
from config import *

_HERE       = os.path.dirname(__file__)
_ASSETS_DIR = os.path.normpath(os.path.join(_HERE, '..', 'assets'))
_OUTPUT_DIR = os.path.normpath(os.path.join(_HERE, '..', 'output'))
os.makedirs(_OUTPUT_DIR, exist_ok=True)

DNF_PENALTY  = MAX_LAP_TIME * 10
RENDER_EVERY = 8

# ── Parameter search space ────────────────────────────────────────────────────
# Each entry: (min, max, initial_guess)
# PID gains are calibrated for normalised error e_norm = e_y / QTR_HALF_SPAN (±1).
# Conversion: Kp_sim = Kp_phys × SCALE,  SCALE = 4000 × (2×2.20/255/0.165) = 418.28
# turn_speed is capped below straight_speed max — robot must slow in corners.
PARAM_SPACE = {
    "kp":               (1.0,   80.0,  PID_KP),
    "ki":               (0.0,    0.5,  PID_KI),
    "kd":               (5.0,  500.0,  PID_KD),
    "pid_limit":        (1.0,   27.0,  PID_LIMIT),
    "integral_limit":   (0.01,  0.50,  PID_INTEGRAL_LIMIT),
    "deriv_filter":     (0.05,  0.60,  PID_DERIV_FILTER),

    "straight_speed":        (0.40, 1.20, SC_STRAIGHT_SPEED),
    "turn_speed":            (0.35, 0.90, SC_TURN_SPEED),       # max < straight_speed max
    "error_threshold":       (0.05, 0.80, SC_ERROR_THRESHOLD),
    "smoothing":             (0.03, 0.30, SC_SMOOTHING),
    "turn_factor_threshold": (0.10, 0.80, SC_TURN_FACTOR_THRESHOLD),
    "min_speed_factor":      (0.10, 0.60, SC_MIN_SPEED_FACTOR),
}


# ── Track cache ───────────────────────────────────────────────────────────────
_track_cache: dict = {}

def _load_track(track_filename: str):
    key = os.path.basename(track_filename)
    if key not in _track_cache:
        path    = os.path.join(_ASSETS_DIR, key)
        track   = load_track_image(path)
        blurred = track.filter(ImageFilter.GaussianBlur(radius=2))
        _track_cache[key] = np.array(blurred, dtype=np.float32)
    return _track_cache[key]


def _get_spawn(track_filename: str):
    key = os.path.basename(track_filename)
    if key not in SPAWN_REGISTRY:
        W, H = MAP_SIZE_M
        print(f"  [warn] No spawn entry for '{key}' — using centre.")
        return {"x": W / 2, "y": H / 2, "theta": 0.0}
    return SPAWN_REGISTRY[key]


# ── Core simulation ───────────────────────────────────────────────────────────

def run_lap(track_filename: str, params: dict, show_viz: bool = False):
    """
    Simulate one lap and return (lap_time, valid, max_error).
    lap_time is None on DNF. valid is False only if the line is lost.
    """
    blur_arr = _load_track(track_filename)
    spawn    = _get_spawn(track_filename)

    pid = PID(kp=params["kp"], ki=params["ki"], kd=params["kd"],
              limit=params["pid_limit"],
              integral_limit=params["integral_limit"],
              derivative_filter=params["deriv_filter"])
    pid.reset()

    sc = SpeedController(
        straight_speed=params["straight_speed"],
        turn_speed=params["turn_speed"],
        error_threshold=params["error_threshold"],
        smoothing=params["smoothing"],
        turn_factor_threshold=params["turn_factor_threshold"],
        min_speed_factor=params["min_speed_factor"])
    sc.reset()

    robot          = Robot()
    robot.position = np.array([spawn["x"], spawn["y"]])
    robot.theta    = spawn["theta"]
    robot.vL = robot.vR = robot.omega = 0.0
    sensors = QTRArray()

    np.random.seed(NOISE_SEED)

    update_plot = None
    if show_viz:
        from visualization.plots import setup_realtime_plot
        update_plot, _ = setup_realtime_plot(
            blur_arr, spawn=(spawn["x"], spawn["y"]), sf_radius=START_FINISH_RADIUS)

    sx, sy         = spawn["x"], spawn["y"]
    departed       = False
    depart_t       = 0.0
    lap_time       = None
    max_error      = 0.0
    valid          = True
    last_e_y       = 0.0
    line_loss_t    = 0.0
    in_finish_zone = False

    _track_key = os.path.basename(track_filename)
    checkpoints = CHECKPOINT_REGISTRY.get(_track_key, [])
    cp_next  = 0
    cp_total = len(checkpoints)

    t = 0.0
    step = 0

    while t < MAX_LAP_TIME:
        readings = sensors.read(robot, blur_arr)
        weights  = readings ** 2
        pos_y    = sensors.sensor_pos_body[:, 1]
        total_w  = weights.sum()

        if total_w > LINE_THRESH:
            e_y = last_e_y = float(np.dot(weights, pos_y) / total_w)
        else:
            e_y = last_e_y

        e_norm = e_y / QTR_HALF_SPAN

        w_cmd = pid.compute(e_norm, DT)
        v_cmd = sc.update(e_norm, w_cmd, pid.limit)

        # Clamp steering so neither wheel reverses (matches physical robot behaviour)
        w_max  = 2.0 * v_cmd / WHEEL_BASE if v_cmd > 0 else 0.0
        w_cmd  = float(np.clip(w_cmd, -w_max, w_max))

        vL_cmd = v_cmd - w_cmd * WHEEL_BASE / 2
        vR_cmd = v_cmd + w_cmd * WHEEL_BASE / 2

        robot.update(vL_cmd, vR_cmd)

        rx, ry        = robot.position
        dist_to_start = np.hypot(rx - sx, ry - sy)

        if cp_next < cp_total:
            cx, cy = checkpoints[cp_next]
            if np.hypot(rx - cx, ry - cy) < CHECKPOINT_RADIUS:
                cp_next += 1

        if not departed and dist_to_start > MIN_DEPARTURE_DIST:
            departed       = True
            depart_t       = t
            in_finish_zone = False

        if departed and lap_time is None:
            if dist_to_start < START_FINISH_RADIUS:
                if not in_finish_zone:
                    in_finish_zone = True
                    elapsed = t - depart_t
                    if elapsed >= MIN_LAP_TIME and cp_next >= cp_total:
                        lap_time = elapsed
                        break
            else:
                in_finish_zone = False

        line_loss_t = line_loss_t + DT if total_w <= LINE_THRESH else 0.0
        if line_loss_t > MAX_LINE_LOSS_TIME:
            valid = False
            break

        abs_e = abs(e_y)
        if abs_e > max_error:
            max_error = abs_e

        if show_viz and update_plot is not None and step % RENDER_EVERY == 0:
            update_plot(robot, readings, e_y, robot.vL, robot.vR, t,
                        lap_time=lap_time, elapsed=t)

        t    += DT
        step += 1

    return lap_time, valid, max_error


def _score(track_filename: str, params: dict) -> tuple:
    """Return (score, lap_time, valid, max_error). Lower score = better."""
    lap_time, valid, max_error = run_lap(track_filename, params)
    if not valid or lap_time is None:
        return DNF_PENALTY, lap_time, valid, max_error
    return lap_time, lap_time, valid, max_error


# ── Helpers ───────────────────────────────────────────────────────────────────

def _clip(params: dict) -> dict:
    out = {}
    for k, v in params.items():
        lo, hi, _ = PARAM_SPACE[k]
        out[k] = float(np.clip(v, lo, hi))
    return out

def _vec_to_params(vec: np.ndarray, keys: list) -> dict:
    return _clip({k: float(v) for k, v in zip(keys, vec)})

def _params_to_vec(params: dict, keys: list) -> np.ndarray:
    return np.array([params[k] for k in keys], dtype=float)

def _initial_params() -> dict:
    return {k: v[2] for k, v in PARAM_SPACE.items()}


# ── CMA-ES ────────────────────────────────────────────────────────────────────

class _CMAES:
    """Minimal CMA-ES — no external dependencies."""

    def __init__(self, x0: np.ndarray, sigma0: float = 0.3, population: int = None):
        self.n   = len(x0)
        self.mu  = population or max(4, self.n)
        self.lam = self.mu * 2
        self.x0    = x0.copy()
        self.sigma = sigma0
        self.mean  = x0.copy()

        self.weights = np.log(self.mu + 0.5) - np.log(np.arange(1, self.mu + 1))
        self.weights /= self.weights.sum()
        self.mueff   = 1.0 / (self.weights ** 2).sum()

        self.cc   = (4 + self.mueff / self.n) / (self.n + 4 + 2 * self.mueff / self.n)
        self.cs   = (self.mueff + 2) / (self.n + self.mueff + 5)
        self.c1   = 2.0 / ((self.n + 1.3) ** 2 + self.mueff)
        self.cmu  = min(1 - self.c1,
                        2 * (self.mueff - 2 + 1 / self.mueff) /
                        ((self.n + 2) ** 2 + self.mueff))
        self.damps = 1 + 2 * max(0, np.sqrt((self.mueff - 1) / (self.n + 1)) - 1) + self.cs

        self.pc = np.zeros(self.n)
        self.ps = np.zeros(self.n)
        self.B  = np.eye(self.n)
        self.D  = np.ones(self.n)
        self.C  = np.eye(self.n)
        self.invsqrtC = np.eye(self.n)
        self.eigeneval = 0
        self.chiN = self.n ** 0.5 * (1 - 1 / (4 * self.n) + 1 / (21 * self.n ** 2))
        self.gen  = 0

    def ask(self) -> list:
        self._update_eigen()
        samples = []
        for _ in range(self.lam):
            z = np.random.randn(self.n)
            x = self.mean + self.sigma * (self.B @ (self.D * z))
            samples.append(x)
        return samples

    def tell(self, xs: list, fs: list):
        self.gen += 1
        order     = np.argsort(fs)
        xs_sorted = [xs[i] for i in order]

        old_mean  = self.mean.copy()
        self.mean = sum(self.weights[i] * xs_sorted[i] for i in range(self.mu))

        self.ps = ((1 - self.cs) * self.ps +
                   np.sqrt(self.cs * (2 - self.cs) * self.mueff) *
                   self.invsqrtC @ (self.mean - old_mean) / self.sigma)

        hsig = (np.linalg.norm(self.ps) /
                np.sqrt(1 - (1 - self.cs) ** (2 * (self.gen + 1))) /
                self.chiN < 1.4 + 2 / (self.n + 1))

        self.pc = ((1 - self.cc) * self.pc +
                   hsig * np.sqrt(self.cc * (2 - self.cc) * self.mueff) *
                   (self.mean - old_mean) / self.sigma)

        artmp  = (1 / self.sigma) * np.array([xs_sorted[i] - old_mean for i in range(self.mu)])
        self.C = ((1 - self.c1 - self.cmu) * self.C +
                  self.c1 * (np.outer(self.pc, self.pc) +
                              (1 - hsig) * self.cc * (2 - self.cc) * self.C) +
                  self.cmu * sum(self.weights[i] * np.outer(artmp[i], artmp[i])
                                 for i in range(self.mu)))

        self.sigma *= np.exp((self.cs / self.damps) *
                             (np.linalg.norm(self.ps) / self.chiN - 1))

    def _update_eigen(self):
        if self.gen - self.eigeneval > self.lam / (self.c1 + self.cmu) / self.n / 10:
            self.eigeneval = self.gen
            self.C = np.triu(self.C) + np.triu(self.C, 1).T
            self.D, self.B = np.linalg.eigh(self.C)
            self.D = np.sqrt(np.maximum(self.D, 1e-20))
            self.invsqrtC = self.B @ np.diag(1.0 / self.D) @ self.B.T


# ── LapOptimizer ──────────────────────────────────────────────────────────────

class LapOptimizer:
    """Minimise lap time on a given track by tuning PID + SpeedController."""

    def __init__(self, track_filename: str = "bane_fase2.png", param_space: dict = None):
        self.track     = os.path.basename(track_filename)
        self.space     = param_space or PARAM_SPACE
        self.keys      = list(self.space.keys())
        self.lo        = np.array([self.space[k][0] for k in self.keys])
        self.hi        = np.array([self.space[k][1] for k in self.keys])
        self.x0        = np.array([self.space[k][2] for k in self.keys])
        self.results   = []
        self.best      = None
        self.iteration = 0
        self.start_time = None

    def _evaluate(self, params: dict):
        s, lap_time, valid, max_error = _score(self.track, params)
        tag = ""
        if not valid:
            result_str = "INVALID"
        elif lap_time is None:
            result_str = "DNF"
        else:
            result_str = f"{lap_time:.3f}s"
            if self.best is None or s < self.best["score"]:
                self.best = {"params": copy.deepcopy(params), "score": s,
                             "lap_time": lap_time, "max_error": max_error}
                tag = "  <- NEW BEST"
        self.results.append({"iteration": self.iteration, "params": params,
                              "score": s, "lap_time": lap_time,
                              "valid": valid, "max_error": max_error})
        return s, result_str, tag

    def _header(self, mode: str, total: str):
        sp = _get_spawn(self.track)
        print("\n" + "="*65)
        print(f"  LAP OPTIMIZER [{mode}]  --  {self.track}")
        print("="*65)
        print(f"  Combinations : {total}")
        print(f"  Spawn        : ({sp['x']:.2f}, {sp['y']:.2f})  "
              f"theta={np.degrees(sp['theta']):.0f} deg")
        print(f"  S/F radius   : {START_FINISH_RADIUS*100:.0f} cm")
        print(f"  DNF if       : line lost > {MAX_LINE_LOSS_TIME:.1f} s")
        print(f"  Max lap time : {MAX_LAP_TIME:.0f} s")
        print("="*65 + "\n")

    def cmaes(self, n_generations: int = 30):
        """CMA-ES — recommended for continuous parameter spaces."""
        self.start_time = datetime.now()
        cma = _CMAES(self.x0, sigma0=0.25)
        total_evals = n_generations * cma.lam
        self._header("CMA-ES", f"~{total_evals} evals ({n_generations} generations)")

        for gen in range(n_generations):
            xs = cma.ask()
            fs = []
            for x in xs:
                self.iteration += 1
                x_clipped = np.clip(x, self.lo, self.hi)
                params = {k: float(v) for k, v in zip(self.keys, x_clipped)}
                s, result_str, tag = self._evaluate(params)
                fs.append(s)
                p = params
                print(f"  [{self.iteration:>4}] gen={gen+1:>3} "
                      f"Kp={p['kp']:.1f}  Ki={p['ki']:.3f}  Kd={p['kd']:.1f}  "
                      f"lim={p['pid_limit']:.1f}  "
                      f"v_str={p['straight_speed']:.3f}  v_trn={p['turn_speed']:.3f}  "
                      f"e_th={p['error_threshold']:.3f}  smth={p['smoothing']:.3f}"
                      f"  ->  {result_str}{tag}")
            cma.tell(xs, fs)
            best_gen = min(fs)
            print(f"  --- gen {gen+1:>3} best: "
                  f"{'DNF' if best_gen >= DNF_PENALTY else f'{best_gen:.3f}s'} "
                  f"| sigma={cma.sigma:.4f}\n")

        self._print_summary()

    def random_search(self, n: int = 40):
        """Uniform random sampling — fast baseline."""
        self.start_time = datetime.now()
        self._header("RANDOM", str(n))
        for _ in range(n):
            self.iteration += 1
            params = {k: float(np.random.uniform(self.space[k][0], self.space[k][1]))
                      for k in self.keys}
            s, result_str, tag = self._evaluate(params)
            p = params
            print(f"  [{self.iteration:>4}/{n}] "
                  f"Kp={p['kp']:.1f}  Ki={p['ki']:.3f}  Kd={p['kd']:.1f}  "
                  f"lim={p['pid_limit']:.1f}  "
                  f"v_str={p['straight_speed']:.3f}  v_trn={p['turn_speed']:.3f}  "
                  f"e_th={p['error_threshold']:.3f}  smth={p['smoothing']:.3f}"
                  f"  ->  {result_str}{tag}")
        self._print_summary()

    def grid_search(self, points_per_param: int = 3):
        """Exhaustive grid search — use only for small spaces."""
        self.start_time = datetime.now()
        ranges = [np.linspace(self.space[k][0], self.space[k][1], points_per_param)
                  for k in self.keys]
        total = points_per_param ** len(self.keys)
        self._header("GRID", str(total))
        for combo in np.ndindex(tuple(points_per_param for _ in self.keys)):
            self.iteration += 1
            params = {self.keys[i]: float(ranges[i][combo[i]])
                      for i in range(len(self.keys))}
            s, result_str, tag = self._evaluate(params)
            p = params
            print(f"  [{self.iteration:>5}/{total}] "
                  f"Kp={p['kp']:.1f}  Ki={p['ki']:.3f}  Kd={p['kd']:.1f}  "
                  f"lim={p['pid_limit']:.1f}  "
                  f"v_str={p['straight_speed']:.3f}  v_trn={p['turn_speed']:.3f}  "
                  f"e_th={p['error_threshold']:.3f}  smth={p['smoothing']:.3f}"
                  f"  ->  {result_str}{tag}")
        self._print_summary()

    def _print_summary(self):
        elapsed = datetime.now() - self.start_time
        print("\n" + "="*65)
        print("  OPTIMIZATION COMPLETE")
        print("="*65)
        print(f"  Evaluations : {self.iteration}  |  Wall time: {elapsed}")

        if self.best is None:
            print("  No valid laps found.")
        else:
            p = self.best["params"]
            print(f"\n  BEST LAP TIME : {self.best['lap_time']:.3f} s")
            print(f"  Peak error    : {self.best['max_error']*1000:.1f} mm")
            print(f"\n  ── Paste into config.py ──────────────────────────────")
            print(f"  PID_KP                   = {p['kp']:.1f}")
            print(f"  PID_KI                   = {p['ki']:.4f}")
            print(f"  PID_KD                   = {p['kd']:.1f}")
            print(f"  PID_LIMIT                = {p['pid_limit']:.1f}")
            print(f"  PID_INTEGRAL_LIMIT       = {p['integral_limit']:.4f}")
            print(f"  PID_DERIV_FILTER         = {p['deriv_filter']:.3f}")
            print(f"")
            print(f"  SC_STRAIGHT_SPEED        = {p['straight_speed']:.4f}")
            print(f"  SC_TURN_SPEED            = {p['turn_speed']:.4f}")
            print(f"  SC_ERROR_THRESHOLD       = {p['error_threshold']:.4f}")
            print(f"  SC_SMOOTHING             = {p['smoothing']:.4f}")
            print(f"  SC_TURN_FACTOR_THRESHOLD = {p['turn_factor_threshold']:.3f}")
            print(f"  SC_MIN_SPEED_FACTOR      = {p['min_speed_factor']:.3f}")
            print(f"  ─────────────────────────────────────────────────────")

        self.save_results()

        top = self.get_top_n(5)
        if top:
            print(f"\n  Top {len(top)} valid laps:")
            print(f"  {'#':<3} {'Time':>7}  {'Kp':>6} {'Ki':>6} {'Kd':>5} "
                  f"{'lim':>5}  {'v_str':>6} {'v_trn':>6} {'e_th':>6} {'smth':>5}  {'err(mm)':>7}")
            print(f"  " + "-"*80)
            for i, r in enumerate(top, 1):
                p = r["params"]
                print(f"  {i:<3} {r['lap_time']:>7.3f}s "
                      f"{p['kp']:>6.1f} {p['ki']:>6.3f} {p['kd']:>5.1f} "
                      f"{p['pid_limit']:>5.1f}  "
                      f"{p['straight_speed']:>6.3f} {p['turn_speed']:>6.3f} "
                      f"{p['error_threshold']:>6.3f} {p['smoothing']:>5.3f}  "
                      f"{r['max_error']*1000:>7.1f}")

    def get_top_n(self, n: int = 5):
        valid = [r for r in self.results if r["valid"] and r["lap_time"] is not None]
        return sorted(valid, key=lambda r: r["lap_time"])[:n]

    def save_results(self):
        ts   = datetime.now().strftime("%Y%m%d_%H%M%S")
        name = os.path.splitext(self.track)[0]
        path = os.path.join(_OUTPUT_DIR, f"lap_{name}_{ts}.json")
        data = {
            "timestamp":   datetime.now().isoformat(),
            "track":       self.track,
            "evaluations": self.iteration,
            "best":        self.best,
            "all_results": [
                {k: (float(v) if isinstance(v, (np.floating, float)) else v)
                 for k, v in r.items() if k != "params"}
                | {"params": {k2: float(v2) for k2, v2 in r["params"].items()}}
                for r in self.results
            ],
        }
        with open(path, "w") as f:
            json.dump(data, f, indent=2)
        print(f"\n  Results saved -> output/lap_{name}_{ts}.json")


# ── CLI ───────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(
        description="Lap-time PID optimizer",
        formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument("--track", default="bane_fase2.png",
                        help="Track image filename in assets/")
    parser.add_argument("--mode", default="cmaes",
                        choices=["cmaes", "random", "grid"],
                        help="cmaes (recommended) | random | grid")
    parser.add_argument("--iterations", type=int, default=30,
                        help="Generations (cmaes) / samples (random) / pts-per-param (grid)")
    args = parser.parse_args()

    opt = LapOptimizer(track_filename=args.track)

    if args.mode == "cmaes":
        opt.cmaes(n_generations=args.iterations)
    elif args.mode == "random":
        opt.random_search(n=args.iterations)
    elif args.mode == "grid":
        opt.grid_search(points_per_param=args.iterations)
