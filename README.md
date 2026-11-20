# Intersection design with NSGA-II: delay vs. cost

An evolutionary algorithm (NSGA-II) designs one signalized four-way intersection.
It searches two goals at once: **delay per car** (from SUMO) and **added
construction cost**. A budget can cap the cost; a preference weight then picks
one design from the trade-offs that were found.

Two production files: `intersection_ea.py` (the EA) and `sumo_simulation.py`
(roads, cost, SUMO). `experiments.py` reruns the behaviour experiments for the
report. `test_multiobjective.py` holds the automated checks.

## What this branch (`Tirths-branch`) changes, compared with `updated_code-1`

| # | Change | Why |
|---|---|---|
| 1 | Score is **delay** (`timeLoss + departDelay`, seconds lost per car), not total journey time. | Journey time included about 216 s of plain driving on long roads, the same for every design, which hid the real differences. |
| 2 | Approach roads are **400 m** (was 1,500 m). Left-turn pockets are **0 or 15–150 m** in 5 m steps (was 12–1,000 m). | Realistic pockets; simulations about 3x faster; widening one axis now costs $896,000 instead of $3,360,000, so the cost curve no longer jumps straight from $0 to $3.36M. |
| 3 | **Yellow is no longer a gene** (12 genes, was 14). It is computed from the speed limit. | The EA was pushing yellow to the maximum and using it as extra green. Yellow length is a safety rule, not something to optimize. |
| 4 | **Creep mutation** replaces random reset. | Numbers move by small steps (up to 15 m or 5 s), so good designs can be fine-tuned. Optional features also switch on/off. |
| 5 | Each design is scored on **3 traffic samples** (seeds 42, 43, 44) and the delays are averaged. | The same design's delay can differ a lot between samples of drivers; averaging stops lucky designs from winning. |
| 6 | Preference scale is **60 s of delay** (was 300 s of journey). | Matches the new, smaller delay numbers. Both preference functions now share one set of defaults. |
| 7 | Added `experiments.py`, 4 new tests, `.gitignore`. Removed the old `intersection_design.py`, `signal_timing_ea.py` and committed `__pycache__` files. | The old files belonged to the earlier version on `main` and crashed here. They are still on `main` and in git history. |

Unchanged: the NSGA-II code (dominance, sorting, crowding, survivor selection,
archive), uniform crossover, tournament selection, the cost model and its prices,
the budget logic, and the road/signal builder.

## Install and run

Python 3.10+ is required. In this folder:

```sh
python -m pip install -r requirements.txt
python -m unittest -v test_multiobjective          # 15 checks, no SUMO needed, < 1 s
python intersection_ea.py                           # one search, about 1.5 minutes
python intersection_ea.py --population 16 --generations 10 --budget 250000 --cost-weight 0.5
python experiments.py --quick                       # behaviour experiments, about 15 minutes
```

On Windows, `py` can replace `python`. SUMO 1.28.0 is installed by pip; an
existing SUMO on PATH or in SUMO_HOME also works. Add `--view` to watch the
selected design in the SUMO GUI (needs a desktop).

## Representation: one dictionary is one individual (12 genes)

| Gene names | Values | Meaning |
|---|---|---|
| `pocket_ns`, `pocket_ew` | 0, or 15–150 m in 5 m steps | Left-turn lane length on both approaches of that axis. 0 = no turn lane. |
| `left_green_ns`, `left_green_ew` | 0, or 5–30 s | Protected left-arrow time. 0 = left turns yield during the main green. Ignored (inactive gene) when the pocket is 0. |
| `lanes_ns`, `lanes_ew` | 1–2 | General lanes per direction, not counting the left pocket. |
| `bypass_n`, `bypass_e`, `bypass_s`, `bypass_w` | True/False | Right-turn slip roads N→W, E→N, S→E, W→S that skip the light and yield when merging. |
| `green_ns`, `green_ew` | 10–60 s | Main green time for each axis. |

Yellow = `max(3, 1 + v / (2 × 3))` seconds, where v is the speed limit in m/s
(1 s reaction, 3 m/s² braking). At 50 km/h: 1 + 13.89 / 6 = **3.31 s**. Every
phase is followed by yellow and a 3 s all-red, so the cycle length is derived.

## Evolution in plain language

1. **Initialize:** a zero-added-cost design, the baseline, and random designs.
2. **Evaluate:** cost is computed first. Over-budget designs are not simulated.
   Others run in SUMO on 3 traffic samples; delay is the average. A gridlock,
   collision or unfinished car on any sample makes the design infeasible.
3. **Rank (NSGA-II):** feasible beats infeasible; then Pareto dominance (no worse
   on both goals, better on one) sorts designs into fronts. Crowding distance
   prefers designs in empty parts of the trade-off curve.
4. **Parent selection:** binary tournament on (front rank, then crowding).
5. **Crossover:** uniform; each gene comes from either parent with probability 0.5.
6. **Mutation:** each gene mutates with probability 1/12.
   - slip roads: bit-flip;
   - pocket/arrow switched off: switches on at a random legal size;
   - pocket/arrow switched on: 20% chance to switch off, otherwise creep;
   - lanes and greens: creep (lanes ±1, greens ±1–5 s, pockets ±5–15 m, arrows ±1–3 s), clipped to the legal range.
7. **Survivor selection:** parents + children compete; best fronts survive
   (elitism), the last front is cut by crowding distance.
8. **Archive and choice:** every feasible non-dominated design ever found is kept.
   At the end, the preference picks one:
   `(1 − w) × delay / 60 s + w × cost / $1,000,000` (lower is better).

## Traffic, score and roads

600 synthetic cars arrive over 30 minutes; 70% from E/W; left/straight/right =
15:70:15; 50 km/h everywhere. Training traffic uses seeds 42, 43, 44. The final
chosen design and the baseline are re-checked on unseen traffic (seed 314).

**Delay per car** = SUMO `timeLoss` (time lost compared with driving at the speed
limit: waiting, slowing, queueing) + `departDelay` (waiting to enter a congested
road). Both are per-car averages. Every car must arrive with no collisions or
teleports.

Approach roads are 400 m (`ARM_LENGTH` in `sumo_simulation.py`). If you raise the
pocket range above 150 m, also raise `ARM_LENGTH`; the code refuses a pocket that
does not fit on the road.

## Choose again without rerunning SUMO

```sh
python intersection_ea.py --select-from results/pareto_front.json --cost-weight 0
python intersection_ea.py --select-from results/pareto_front.json --cost-weight 0.8 --budget 100000
```

## Experiments for the report

`python experiments.py` runs five experiments with 5 EA seeds each, changing one
setting at a time: A defaults (population 8, 5 generations), B random search with
the same number of designs, C bigger budget (16 × 10), D mutation 0.25, E
tournament 4. It writes `experiments/runs.csv`, `experiments/summary.png`
(lowest delay and hypervolume per run) and `experiments/fronts.png` (about 35 minutes on 2 cores). Hypervolume
is the share of the box delay 0–60 s × cost $0–3M covered by a run's trade-off
curve; higher means cheaper-and-faster designs were found across the range.
Results from this branch are in `VALIDATION.md`.

## Scope and limitations

- Costs are illustrative incremental infrastructure costs (see `COST_MODEL.md`),
  not municipal quotes. Land, utilities, junction pavement, maintenance and
  demolition are not priced. Lane widening is priced over the 400 m simulated
  roads, so changing `ARM_LENGTH` also changes lane prices.
- Traffic is synthetic. Pedestrians, U-turns, adaptive signals, overpasses and
  other control types (stop signs, roundabouts) are not modelled.
- Some slip-road layouts produce collisions at the merge; these designs are
  rejected as infeasible, not repaired.
- Results depend on random seeds; report several seeds, not one run.

SUMO references:
- https://sumo.dlr.de/docs/Networks/PlainXML.html
- https://sumo.dlr.de/docs/Simulation/Traffic_Lights.html
- https://sumo.dlr.de/docs/Simulation/Output/StatisticOutput.html
