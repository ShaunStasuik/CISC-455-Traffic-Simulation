# Validation and experiment results (Tirths-branch)

Run on 2026-11-20 with Python 3.13 and eclipse-sumo 1.28.0. These checks show the
prototype works and how the EA behaves. They are not proof of global optimality
or of real-world engineering validity.

## 1. Automated tests

```sh
python -m unittest -v test_multiobjective
```

15 tests pass in under a second (no SUMO needed). They cover Pareto dominance and
sorting, constraint priority, crowding distance, survivor selection, the archive,
preference endpoints and budgets, cost examples, dormant arrows, lane widening on
400 m roads, refusal of pockets that do not fit, creep mutation staying legal and
small, features switching on/off, yellow being a rule rather than a gene,
averaging over traffic samples with caching, and failure handling.

## 2. Reference designs

Delay = seconds lost per car (`timeLoss + departDelay`). Training = mean of traffic
seeds 42, 43, 44. Fresh = unseen traffic seed 314.

| Design | Added cost | Training delay | Fresh delay |
|---|---:|---:|---:|
| Baseline (50 m pockets, 8 s arrows, 30/30 greens) | $172,000 | 52.8 s | 51.6 s |
| Do-nothing (no pockets, no arrows, 30/30 greens) | $0 | 30.0 s | 35.7 s |

The baseline is worse than doing nothing: its protected left arrows add four
extra phases and lengthen the cycle. The same design's delay varies between
traffic samples (do-nothing: 27.6 / 36.1 / 26.5 s), which is why designs are
averaged over three samples.

## 3. Behaviour experiments

`python experiments.py`: each experiment changes one setting; 5 EA seeds each;
random search gets the same number of designs as the default EA with the same
seed. 1,979 s on 2 CPU cores.

| Experiment | Lowest delay found (mean of 5) | Hypervolume (mean of 5) | Designs per run | Pick at weight 0.5: cost / fresh delay (mean) |
|---|---:|---:|---:|---:|
| A defaults (population 8, 5 generations, mutation 1/12, tournament 2) | 16.6 s | 0.703 | 43 | $16,800 / 27.8 s |
| B random search (same number of designs) | 15.1 s | 0.668 | 43 | $0 / 35.7 s |
| C bigger budget (population 16, 10 generations) | **13.9 s** | **0.752** | 156 | $31,360 / **19.6 s** |
| D mutation 0.25 | 14.7 s | 0.723 | 47 | $48,438 / 25.8 s |
| E tournament size 4 | 15.9 s | 0.713 | 40 | $57,120 / 23.8 s |

Hypervolume = share of the box delay 0–60 s × cost $0–3M that a run's trade-off
curve covers (higher = cheaper-and-faster designs found across the range).
Charts: `figures/summary.png` (every run as a dot) and `figures/fronts.png`
(trade-off curves of A, B and C). Raw numbers: `figures/runs.csv`.

## 4. What the results say

- **Evolution beats random search on the trade-off, not on the single fastest
  design.** The EA's hypervolume was higher in 4 of 5 paired seeds (0.703 vs
  0.668). Random search sometimes stumbles on a very fast but expensive design
  (lowest delay 15.1 s vs 16.6 s), but it never found cheap improvements: its
  recommended design was the $0 do-nothing plan in all 5 runs (35.7 s fresh).
  The EA's recommendations cost about $17k and cut fresh delay to 27.8 s on average.
- **More search is better and more consistent.** Population 16 × 10 generations
  gave the best results on every measure (13.9 s lowest delay, hypervolume 0.752,
  recommended design 19.6 s fresh for about $31k), at about 3.5x the simulations.
- **More mutation helped with creep mutation.** Mutation 0.25 raised hypervolume
  from 0.703 to 0.723. (In the earlier version with random-reset mutation, the
  same change made no difference.)
- **Tournament size 4 made little difference** (0.713 vs 0.703); with 5 seeds
  and this spread, that is not a clear effect.
- **Luck still matters at the default budget.** Lowest delay with the defaults
  ranged from 12.6 s to 18.1 s across seeds; report several seeds.
- **The cost curve is now continuous.** With 400 m roads, widening one axis
  costs $896,000 instead of $3.36M, so the trade-off curves fill the $0–2.5M
  range rather than jumping from $0 to $3.36M.
- **Training results hold on fresh traffic.** Recommended designs were usually
  within about 1–6 s of their training delay on the unseen traffic sample.
- **Some designs are infeasible.** 0–6 designs per run produced collisions or
  unfinished cars (mostly slip-road merges) and were rejected.

## 5. Comparison with the earlier version (updated_code-1)

The earlier version scored journey time on 1,500 m roads (about 216 s of every
journey was plain driving), so its numbers are not directly comparable. The same
style of experiment on it showed: the EA beat random search on hypervolume
(0.666 vs 0.575), random-reset mutation 0.25 changed nothing, the cost curve
jumped from about $0.4M straight to $3.36M, and the recommended design at weight
0.5 was almost always the $0 one. With this branch, recommendations are real
low-cost designs, the cost curve is continuous, and mutation rate now matters.

## Limits

Five seeds per experiment is a small sample. Traffic is synthetic. Costs are
illustrative (`COST_MODEL.md`). Slip-road merge collisions are rejected, not
fixed. Only the recommended design and the baseline are checked on fresh traffic;
the whole front is not.
