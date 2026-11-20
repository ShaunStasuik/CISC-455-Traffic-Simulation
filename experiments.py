"""Repeatable behaviour experiments for the report. Run: python experiments.py

Each experiment changes ONE setting, runs the EA with several EA seeds, and is
compared with pure random search given the same number of designs. Results go
to experiments/ as a CSV per run plus two charts (needs matplotlib).

Quick check (about 15 minutes on 2 cores):  python experiments.py --quick
Full set (about 35 minutes on 2 cores):     python experiments.py
"""
import argparse
import contextlib
import csv
import io
import json
import math
import random
import shutil
import statistics
import tempfile
from multiprocessing import Pool
from pathlib import Path

import intersection_ea as ea
import sumo_simulation as sim

# Hypervolume box: delay 0..REF_DELAY seconds, cost $0..REF_COST.
# A front's hypervolume is the share of this box its designs dominate (0..1, higher = better).
REF_DELAY = 60.0
REF_COST = 3_000_000.0

# (label, kind, population, generations, mutation probability, tournament size)
EXPERIMENTS = [
    ("A defaults", "ea", 8, 5, 1 / len(ea.GENE_VALUES), 2),
    ("B random search", "random", 8, 5, None, None),
    ("C bigger budget", "ea", 16, 10, 1 / len(ea.GENE_VALUES), 2),
    ("D mutation 0.25", "ea", 8, 5, 0.25, 2),
    ("E tournament 4", "ea", 8, 5, 1 / len(ea.GENE_VALUES), 4),
]


def hypervolume(front):
    points = sorted((p["cost"], p["delay"]) for p in front
                    if p["violation"] == 0 and p["delay"] < REF_DELAY and p["cost"] < REF_COST)
    area, best = 0.0, REF_DELAY
    for i, (cost, delay) in enumerate(points):
        best = min(best, delay)
        next_cost = points[i + 1][0] if i + 1 < len(points) else REF_COST
        area += (next_cost - cost) * (REF_DELAY - best)
    return area / (REF_DELAY * REF_COST)


def random_search(evaluate, designs, seed):
    """Same starting designs as the EA, then purely random plans. No selection, no variation."""
    rng = random.Random(seed)
    seen, items = set(), []
    for plan in [ea.minimum_design(), dict(ea.BASELINE)]:
        seen.add(ea.plan_key(plan)); items.append(evaluate(plan))
    while len(seen) < designs:
        plan = ea.random_individual(rng)
        if ea.plan_key(plan) not in seen:
            seen.add(ea.plan_key(plan)); items.append(evaluate(plan))
    return ea.update_archive([], items)


def run_one(job):
    label, kind, population, generations, mutation, tournament, seed, match_designs = job
    folder = Path(tempfile.mkdtemp(prefix="exp_"))
    try:
        runs = [(sim.make_traffic(folder / f"train_{s}.rou.xml", s), s) for s in sim.TRAIN_SEEDS]
        evaluate, _, cache, counters = sim.make_evaluator(folder, runs)
        with contextlib.redirect_stdout(io.StringIO()):
            if kind == "ea":
                ea.MUTATION_PROBABILITY, ea.TOURNAMENT_SIZE = mutation, tournament
                front = ea.evolve(evaluate, population, generations, seed=seed)["front"]
            else:
                front = random_search(evaluate, match_designs, seed)
        feasible = [item for item in cache.values() if item["violation"] == 0]
        pick = ea.recommend(front, cost_weight=0.5)
        test = sim.run_simulation(pick["plan"], folder / "fresh",
                                  sim.make_traffic(folder / "test.rou.xml", sim.TEST_SEED), sim.TEST_SEED)
        return dict(experiment=label, seed=seed, designs=len(cache), infeasible=counters["simulation_failures"],
                    lowest_delay=min(i["delay"] for i in feasible), front_size=len(front),
                    hypervolume=hypervolume(front), pick_cost=pick["cost"], pick_delay=pick["delay"],
                    pick_fresh_delay=test["score"], front=[(p["cost"], p["delay"]) for p in front])
    finally:
        shutil.rmtree(folder, ignore_errors=True)


def charts(rows, out):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    labels = [e[0] for e in EXPERIMENTS]
    colours = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#6250d6"]
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.4), dpi=150)
    for ax, key, title in [(axes[0], "lowest_delay", "Lowest delay found (s per car, lower = better)"),
                           (axes[1], "hypervolume", "Trade-off quality, hypervolume (higher = better)")]:
        for i, label in enumerate(labels):
            values = [r[key] for r in rows if r["experiment"] == label]
            xs = [i + (j - (len(values) - 1) / 2) * 0.09 for j in range(len(values))]
            ax.scatter(xs, values, s=40, color=colours[i], edgecolor="white", zorder=3)
            mean = statistics.mean(values)
            ax.plot([i - 0.28, i + 0.28], [mean, mean], color="#2c2c2a", lw=2, zorder=4)
            ax.annotate(f"{mean:.3g}", (i + 0.3, mean), va="center", fontsize=9)
        ax.set_xticks(range(len(labels)), [l.replace(" ", "\n", 1) for l in labels], fontsize=9)
        ax.set_title(title, loc="left", fontsize=11)
        ax.spines[["top", "right"]].set_visible(False); ax.yaxis.grid(True, color="#e8e6e0")
    fig.text(0.01, 0.01, "Each dot = one EA seed. Bar = average.", fontsize=8.5, color="#5f5e5a")
    fig.tight_layout(rect=(0, 0.03, 1, 1)); fig.savefig(out / "summary.png", facecolor="white")

    fig, ax = plt.subplots(figsize=(9, 5), dpi=150)
    for i, label in enumerate(labels[:3]):
        for n, row in enumerate(r for r in rows if r["experiment"] == label):
            pts = sorted(row["front"])
            ax.step([c / 1e6 for c, _ in pts], [d for _, d in pts], where="post", color=colours[i],
                    alpha=0.8, label=label if n == 0 else None)
            ax.scatter([c / 1e6 for c, _ in pts], [d for _, d in pts], s=12, color=colours[i])
    ax.set_xlabel("Added infrastructure cost ($ millions)"); ax.set_ylabel("Delay per car (s)")
    ax.set_title("Trade-off curves, every seed (down and left = better)", loc="left", fontsize=11)
    ax.legend(frameon=False); ax.spines[["top", "right"]].set_visible(False)
    fig.tight_layout(); fig.savefig(out / "fronts.png", facecolor="white")


def main():
    parser = argparse.ArgumentParser(description="Run the behaviour experiments")
    parser.add_argument("--quick", action="store_true", help="2 seeds per experiment instead of 5")
    parser.add_argument("--workers", type=int, default=2, help="parallel processes (CPU cores)")
    args = parser.parse_args()
    seeds = [1, 2] if args.quick else [1, 2, 3, 4, 5]
    out = Path("experiments"); out.mkdir(exist_ok=True)

    ea_jobs = [(label, kind, pop, gens, mut, tour, seed, None)
               for label, kind, pop, gens, mut, tour in EXPERIMENTS if kind == "ea" for seed in seeds]
    with Pool(args.workers) as pool:
        rows = pool.map(run_one, ea_jobs)
        # Random search gets exactly as many designs as the default EA used with the same seed.
        budget = {r["seed"]: r["designs"] for r in rows if r["experiment"] == "A defaults"}
        rs_jobs = [(label, kind, pop, gens, mut, tour, seed, budget[seed])
                   for label, kind, pop, gens, mut, tour in EXPERIMENTS if kind == "random" for seed in seeds]
        rows += pool.map(run_one, rs_jobs)

    fields = [k for k in rows[0] if k != "front"]
    with (out / "runs.csv").open("w", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=fields)
        writer.writeheader()
        writer.writerows({k: r[k] for k in fields} for r in rows)
    (out / "fronts.json").write_text(json.dumps([{k: r[k] for k in ("experiment", "seed", "front")} for r in rows]))

    print(f"{'experiment':18s} {'lowest delay':>14s} {'hypervolume':>12s} {'designs':>8s} {'pick@0.5':>22s}")
    for label, *_ in EXPERIMENTS:
        group = [r for r in rows if r["experiment"] == label]
        mean = lambda key: statistics.mean(r[key] for r in group)
        print(f"{label:18s} {mean('lowest_delay'):11.2f} s  {mean('hypervolume'):12.3f} {mean('designs'):8.0f}"
              f"   ${mean('pick_cost'):>9,.0f} / {mean('pick_fresh_delay'):.1f} s fresh")
    try:
        charts(rows, out)
        print("Charts: experiments/summary.png, experiments/fronts.png")
    except ImportError:
        print("Install matplotlib for charts: python -m pip install matplotlib")
    print("Per-run results: experiments/runs.csv")


if __name__ == "__main__":
    main()
