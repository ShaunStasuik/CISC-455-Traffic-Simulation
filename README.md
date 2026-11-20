# Simple intersection EA: cost and journey time

A small **NSGA-II** prototype with the original 14 intersection genes. The two
production Python files remain separate: `intersection_ea.py` handles evolution
and user preferences; `sumo_simulation.py` handles geometry, cost and SUMO.
Control type stays fixed to traffic signals. Mutation and uniform recombination
are unchanged from the simple baseline.

## Install and run

Python 3.10+ is required. In this folder:

```sh
python -m pip install -r requirements.txt
python intersection_ea.py
python intersection_ea.py --population 8 --generations 5 --budget 250000 --cost-weight 0.5
```

On Windows, `py` can replace `python`. SUMO 1.28.0 is pinned; an existing SUMO
installation on PATH or in SUMO_HOME also works. Add `--view` to a new search to
watch the selected plan (requires a desktop).

The default is population 8, five reproduction generations, seed 17, no budget,
and cost weight 0.5. These are small demonstration settings, not tuned optima.

## Three separate concepts

1. **Objectives:** minimize mean journey time and added infrastructure cost.
   NSGA-II does not combine these into a weighted fitness.
2. **Budget:** an optional hard maximum on added infrastructure cost. Over-budget
   candidates are not simulated and cannot be recommended. Zero is a valid cap;
   omit `--budget` for no cap.
3. **Preference:** after evolution, choose from the feasible non-dominated archive
   using `(1-w)*time/time_reference + w*cost/cost_reference`. Lower is better.
   `--cost-weight 0` selects the fastest; `1` the cheapest (ties choose faster).

The default references are **300 seconds** and **1,000,000 CAD**. They are fixed,
explicit modelling scales, not data-dependent minima/maxima or a claim about the
value of time. Set them with `--time-reference` and `--cost-reference`. A weight
of 0.5 means equal coefficients on these normalized quantities, NOT equal value
for one dollar and one second. References matter; keep them fixed when comparing
preferences. Zero/negative references, NaN, infinity and invalid weights are rejected.

Budget and preference are inputs, not genes. A low weight does not relax a budget.
The recommended cost model is explained, with sources, in `COST_MODEL.md`.
Change its prices using `--road-cost 175 --arrow-cost 15000`.

## Choose again without rerunning SUMO

```sh
python intersection_ea.py --select-from results/pareto_front.json --cost-weight 0
python intersection_ea.py --select-from results/pareto_front.json --cost-weight 0.8 --budget 100000
```

This saves a new `best_plan.json` and `recommendation.json` beside the supplied
archive. It does not run SUMO. If omitted, the budget and normalization references
are inherited from the archive (the cost weight defaults to 0.5). A lower cap
filters the archive; raising a cap does not recover designs excluded during the
original search. An empty eligible set produces an explicit error, without
claiming that no feasible design exists anywhere. Changing unit prices requires
a new search; this re-ranking command intentionally rejects price overrides.

Replay independently:

```sh
python sumo_simulation.py --plan results/best_plan.json --view
python sumo_simulation.py
```

The second command runs the original baseline. Re-ranking does not validate the
newly selected plan on fresh traffic; use standalone replay to do that.

## Output files

Each search creates `results/experiment_.../` containing:

- `pareto_front.json`: feasible non-dominated training designs discovered during
  the entire search, costs/breakdowns, history, input settings and counters.
- `pareto_front.csv`: the same trade-offs in a compact table with all 14 genes.
- `best_plan.json`: plain genome for the selected design, compatible with replay.
- `recommendation.json` and `comparison.json`: initial selection preferences,
  baseline and selected-plan fresh-traffic checks. The baseline is only a reference
  and can exceed the budget; it is explicitly labelled.
- `evaluations.json`: all unique requested training candidates, including rejection
  reasons. Infinite/unavailable scores are written as JSON null.
- Individual SUMO networks, signal plans, metrics and replay configurations.

Copies of the Pareto archive, CSV and selected-plan files are placed directly in
`results/` for convenience. Earlier experiment directories remain intact. A search
with no feasible result saves diagnostics and clears current selected-plan pointers
so an old result cannot be mistaken for a new one. Preference re-ranking preserves
the original experiment comparison report.

The archive is an approximation, not a proof of global Pareto optimality. One
representative is retained for identical objective pairs. The archive may contain
more designs than the final population. Only the selected design and baseline are
checked on fresh traffic after a search; the whole front is not independently
validated. A fresh-traffic failure is clearly reported rather than concealed.

## Read the code

In `intersection_ea.py`, read representation, mutation/recombination, `dominates`,
`non_dominated_sort`, `crowding_distance`, `evolve`, then `recommend`.

In `sumo_simulation.py`, read `cost_breakdown`, `make_evaluator`, `run_experiment`,
and `run_simulation`. XML and geometry details follow. `test_multiobjective.py`
is an independent test file, not another runtime dependency.

## Representation: one dictionary is one individual

`BASELINE` shows the whole representation with a comment beside each feature.
For example, `plan["green_ns"] = 40` means 40 seconds of main green for both
north and south approaches. NS/EW pairing limits the number of genes; traffic
from the four approaches can still differ.

| Gene names | Values | Effect |
|---|---|---|
| `pocket_ns`, `pocket_ew` | 0 or integer 12–1000 m | Add one dedicated inner left-turn lane on each approach of that axis. Length changes its queue storage. Zero removes it. |
| `left_green_ns`, `left_green_ew` | 0 or integer 5–30 s | Protected left-arrow duration. Zero allows yielding left turns during the main green instead. Inactive when the pocket is absent. |
| `lanes_ns`, `lanes_ew` | Integer 1–2 | General lanes per direction, not counting the additional left pocket. |
| `bypass_n`, `bypass_e`, `bypass_s`, `bypass_w` | True/False | Separate right-turn slip roads: N→W, E→N, S→E, W→S. They bypass the central light and yield when merging. |
| `green_ns`, `green_ew` | Integer 10–60 s | Main green durations for the two axes. |
| `yellow_ns`, `yellow_ew` | Integer 3–6 s | Requested yellow durations, raised if needed to meet the simulation's speed-derived minimum. |

There are 14 genes: 2 + 2 + 2 + 4 + 2 + 2. The right-turn booleans use the
earlier slip-road interpretation; they are not additional variable-length
storage pockets. Lane bounds mean up to two general lanes plus one left pocket
per incoming direction, not two lanes total across both directions.

Pocket length and arrow duration stay independent. Storage determines how many
turners can wait out of the through lane; green duration determines service
time. A long pocket need not get a proportionally long arrow. If a pocket is
zero, its arrow gene stays in the dictionary but is ignored by the simulator.
It can become active if mutation or recombination later introduces a pocket.

## Evolution in plain language

1. Initialize with a zero-added-cost design, the original baseline, and random
   plans. Under a budget, bounded cheap sampling seeks affordable random plans;
   if unsuccessful, use minimum infrastructure with randomized timings.
2. Evaluate `(journey_time, cost)`. Exact repeated genomes reuse cached results.
   Budget excess is checked before SUMO. Invalid simulations remain infeasible.
3. Sort into fronts using dominance. Feasible designs outrank infeasible designs;
   among infeasible ones, smaller budget excess wins, while simulation failures
   have infinite constraint violation. They never enter the returned archive.
4. Select parents using binary tournaments: lower front rank, then larger crowding
   distance. Crowding is normalized separately along each front's objectives.
5. Apply the original uniform crossover and per-gene `1/14` mutation (Boolean
   flip or numeric random reset). These remain easy to replace later.
6. Combine parents and offspring; fill the next generation front by front. If a
   front will not fit, preserve its less crowded members (NSGA-II elitism).
7. Update an archive of feasible non-dominated results ever found. At the end,
   apply the user's preference to that archive.

Progress prints feasible count, unique genomes, fastest feasible journey, and
archive size. History also records median feasible journey time. The weight does
not affect search: runs with the same search inputs and seed but different weights
produce the same training archive. Budget does affect the search.

## Traffic and simulation objective

The synthetic demand is unchanged: 600 passenger cars departing over 1800 seconds,
70% probability of entering from E/W, with left/straight/right weights 15:70:15.
Approach speeds are 50 km/h. Training seed 42 is shared across candidates;
validation seed 314 is separate. These are settings, not intersection genes.

Journey time is `(total travel time + departure insertion delay) / 600`. Every car
must arrive, without collisions or teleports, or the design is infeasible.
Installation/process failures are reported as errors. Exact-genome caching is
per run; geometry is still rebuilt for every new simulated design.

## What the SUMO file builds

All designs are signalized four-arm intersections with 1500 m approach arms.
The pocket lengths change actual simulated lane storage. SUMO's explicit edge
lengths specify storage; the graphical layout includes junction allowances and
is an abstract drawing rather than an engineering plan. Nearby slip-road gates
are moved when needed to leave enough separation from a pocket entrance.

Main greens alternate NS and EW. Left turns without a protected phase use
SUMO's yielding `g`; other permitted movements use priority `G`. Protected
lefts run before the axis's main green and remain red during that main phase.
If opposing protected turns conflict in the compiled geometry, they run in
separate phases, each using that axis's arrow duration. Every phase is followed
by yellow and a fixed 3-second all-red. The complete cycle length is therefore
derived, not another gene. `phase_audit.json` shows the actual plan.

The illustrative yellow floor is `max(3, 1 + v/(2*3))`, where `v` is the faster
approach speed on that axis in m/s. It assumes a 1-second reaction time and
3 m/s² deceleration on level ground. This and the all-red setting are classroom
simulation assumptions, not certified road timing rules. SUMO runs with a
0.2-second time step and checks junction collisions. No pedestrians, U-turns,
adaptive signals, overpasses or alternative intersection controls are modelled.

## Scope and limitations

- Cost is an illustrative incremental infrastructure proxy, not a municipal quote.
  One-lane roads/basic signals form a common reference; land, utilities, detailed
  junction pavement, tapers, islands, maintenance and demolition are not priced.
- Lanes widen all 1500 m arms in both directions. One axis changing from one to
  two general lanes adds 19,200 m2 and costs $3,360,000 at default prices. A budget
  of $250,000 therefore excludes that change. Restricting construction to a short
  intersection zone would also require changing the simulated roads.
- The original model has shown bypass-gate collision failures for some designs.
  These are rejected, not fixed by this update. Results are conditional on the
  simplified network and should not be presented as engineering-ready designs.
- Extra cost objectives do not guarantee better search or realism. Use several
  seeds and a small price-sensitivity comparison for the course project.

## Tests

```sh
python -m unittest -v test_multiobjective
```

These tests require no SUMO. `VALIDATION.md` records real SUMO smoke runs and
remaining limitations. `sample_results/` contains compact results from checked
runs; full generated simulation folders are omitted from the distribution.

SUMO format references:
- https://sumo.dlr.de/docs/Networks/PlainXML.html
- https://sumo.dlr.de/docs/Simulation/Traffic_Lights.html
- https://sumo.dlr.de/docs/Definition_of_Vehicles,_Vehicle_Types,_and_Routes.html
