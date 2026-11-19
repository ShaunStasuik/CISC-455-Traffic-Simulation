# CISC 455 Traffic Simulation
Optimizing traffic flow using Evolutionary Optimization.

## Running

```
py signal_timing_ea.py [--value-weight 0.5] [--budget 2500000] [--view]
```

Requires SUMO (`sumo`, `netconvert`, and `sumo-gui` for `--view`) on PATH.

## What the EA evolves

One individual is a full intersection design (`intersection_design.py`):
control type (stop / signal / roundabout), lanes per axis, left-turn lane length,
a protected left-turn arrow, right-turn pockets per approach, and the green and
yellow timers. Every feature has a price; all prices live at the top of
`intersection_design.py`.

Fitness (lower is better) balances two goals:

```
score = (1 - value_weight) * delay / baseline_delay
      +      value_weight  * |cost - budget| / budget
```

- `value_weight` 0 cares only about traffic, 1 only about cost, 0.5 splits evenly.
- The cost term rewards getting *close* to the budget, so the EA doesn't leave
  money unspent when spending it would noticeably improve traffic.
- Yellow is repaired up to the minimum safe time for the speed limit (4 s at 50 km/h).
