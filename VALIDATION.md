# Validation of cost, NSGA-II, preference and budget update

Validation performed 2026-11-19 with Python 3 and eclipse-sumo 1.28.0.
The original 14 genes, road geometry, signal decoder and random-reset mutation
were retained. These checks establish prototype correctness, not engineering
validity, statistical superiority, or optimality.

## Automated checks

`python -m unittest -v test_multiobjective`

12 deterministic tests cover known Pareto fronts, equality, feasibility priority,
crowding normalization/endpoints, survivor truncation, archive maintenance,
preference endpoints, exact budget boundaries, invalid settings, worked cost
examples, dormant arrows, complete incoming/outgoing widening, pocket extremes,
bypass geometry costs, zero-price handling, pre-SUMO budget screening, caching,
failed simulations, zero-budget evolution, strict JSON, and failure output.

CLI checks additionally exercised saved-archive selection with weights 0 and 1,
filtering at 250,000 CAD, saved-reference inheritance, and rejection of negative
budgets/prices, NaN weights, out-of-range weights, zero references and re-ranking
with new unit-price overrides. Re-ranking ran without a SUMO runtime on its path.

## Real SUMO runs

All runs used 600 cars, 1800 s arrival window, default 70% E/W traffic, training
seed 42 and fresh-traffic seed 314. Unit costs were 175 CAD/m2 and 15,000
CAD/approach. Cost references were 1,000,000 CAD and 300 seconds.

- Unconstrained: population 8, five reproduction generations, EA seed 17,
  preference weight 0.1, no budget.
- Budget 250000: population 6, three reproduction generations, EA seed 17,
  preference weight 0.5, budget 250,000 CAD.
- Zero budget: population 3, one reproduction generation, EA seed 17,
  preference weight 0.5, budget 0 CAD.

| Run | Archive designs | Training SUMO runs | Budget skips | Infeasible simulations | Selected cost CAD | Training seconds | Fresh seconds |
|---|---:|---:|---:|---:|---:|---:|---:|
| unconstrained | 8 | 43 | 0 | 0 | 0.00 | 252.5286 | 255.4573 |
| budget_250000 | 2 | 14 | 3 | 1 | 0.00 | 268.6909 | 272.2676 |
| zero_budget | 1 | 3 | 2 | 0 | 0.00 | 260.9939 | 262.0536 |

All selected designs and the baseline completed 600/600 fresh-traffic trips
without collisions or teleports. Every archived design was checked to be
training-feasible, within its budget, and not dominated by another archive entry.
The baseline fresh-traffic mean journey was 295.53395 seconds in each run.

The unrestricted archive spans zero added cost (252.53 s) to 4,818,610.28 CAD
(236.40 s). Saved-archive re-ranking selected the fastest endpoint at weight 0,
the cheapest endpoint at weight 1, and 204,960 CAD / 248.99 s at weight 0 with
a 250,000 CAD filter. These re-ranked alternatives were NOT all freshly validated.

The budget-250000 run encountered one infeasible simulation, which was excluded.
The pre-existing bypass/merge model can produce collision failures; this update
does not resolve that modelling limitation. Passing selected designs does not
prove every generated network is valid.

Compact archives, CSVs and initial recommendation reports are in `sample_results/`.
The example plan is the selected design from the unconstrained run. No full SUMO
run folders, absolute-path replay files, caches or installed binaries are shipped.

## Limits

Only three small smoke searches were performed, not a multi-seed comparative
study. A small archive is not the entire Pareto front. Timing/traffic assumptions,
collision handling and the cost omissions in COST_MODEL.md remain limitations.
GUI playback was not manually inspected. Unit prices are historical modelling
proxies, not quotations. Both normalization scales and unit prices affect final
recommendations and should be documented in any experiment.
