"""CISC 455: evolutionary search for a four-way intersection design within a budget.

Keep sumo_simulation.py and intersection_design.py beside this file.
Run: py signal_timing_ea.py [--value-weight 0.5] [--budget 2500000] [--view]

Read evolve() first. One individual is a Design (see intersection_design.py):
control type, lanes, turning lanes, pockets, and signal timers.
The EA asks fitness(design) for a number. Its only job is finding a smaller one.
That number blends traffic delay and distance from the budget; VALUE_WEIGHT
decides which goal matters more (0 = traffic only, 1 = cost only).
"""
import argparse
import random

from intersection_design import (ARMS, CONTROL_TYPES, MAX_ARROW, MAX_GREEN, MAX_LANES,
                                 MAX_TURN_LANE_M, MAX_YELLOW, MIN_ARROW, MIN_GREEN,
                                 MIN_LANES, MIN_SAFE_YELLOW, TURN_LANE_STEP, Design,
                                 describe, repair, total_cost)
from sumo_simulation import run_experiment


BASELINE = Design()         # one-lane roads, a 30/30 signal, no extras
VALUE_WEIGHT = 0.5          # 0 = traffic only, 1 = cost only
BUDGET = 2_500_000          # dollars
POPULATION = 16
GENERATIONS = 12
MUTATION_PROBABILITY = 0.2  # per gene
MUTATION_STEP = 6           # maximum seconds added or removed from a green
EA_SEED = 17


def random_design(rng):
    return repair(Design(
        control=rng.choice(CONTROL_TYPES),
        ns_lanes=rng.randint(MIN_LANES, MAX_LANES),
        ew_lanes=rng.randint(MIN_LANES, MAX_LANES),
        turn_lane_m=rng.choice(range(0, MAX_TURN_LANE_M + 1, TURN_LANE_STEP)),
        turn_arrow=rng.random() < 0.5,
        right_pockets=tuple(rng.random() < 0.5 for _ in ARMS),
        ns_green=rng.randint(MIN_GREEN, MAX_GREEN),
        ew_green=rng.randint(MIN_GREEN, MAX_GREEN),
        arrow_green=rng.randint(MIN_ARROW, MAX_ARROW),
        yellow=rng.randint(MIN_SAFE_YELLOW, MAX_YELLOW),
    ))


def mutate(design, rng):
    """Give each gene an independent chance to change; repair() keeps the result legal."""
    genes = design._asdict()

    def chance():
        return rng.random() < MUTATION_PROBABILITY

    def nudge(step):
        return rng.choice([-1, 1]) * rng.randint(1, step)

    if chance():
        genes["control"] = rng.choice([c for c in CONTROL_TYPES if c != design.control])
    for key in ("ns_lanes", "ew_lanes"):
        if chance():
            genes[key] += nudge(1)
    if chance():
        genes["turn_lane_m"] += nudge(2) * TURN_LANE_STEP
    if chance():
        genes["turn_arrow"] = not design.turn_arrow
    genes["right_pockets"] = tuple(not on if chance() else on for on in design.right_pockets)
    for key, step in (("ns_green", MUTATION_STEP), ("ew_green", MUTATION_STEP),
                      ("arrow_green", 3), ("yellow", 1)):
        if chance():
            genes[key] += nudge(step)
    return repair(Design(**genes))


def crossover(parent_a, parent_b, rng):
    """Uniform crossover: each gene comes from either parent with equal chance."""
    return repair(Design(*(rng.choice(pair) for pair in zip(parent_a, parent_b))))


def evolve(fitness):
    """Return the best design. Calling fitness(design) gives its blended score."""
    rng = random.Random(EA_SEED)
    # Initialization: include one known design, then fill the population at random.
    population = [BASELINE]
    while len(population) < POPULATION:
        population.append(random_design(rng))

    def select_parent():
        contestants = rng.sample(population, 3)
        return min(contestants, key=fitness)  # tournament of three

    for generation in range(GENERATIONS + 1):
        # Rank candidates by their simulated score. Lower is better.
        best = min(population, key=fitness)
        print(f"Generation {generation:2d} | score {fitness(best):6.3f} | "
              f"${total_cost(best):>10,.0f} | {describe(best)}", flush=True)
        if generation == GENERATIONS:
            return best
        offspring = [best]  # elitism: retain the current best
        while len(offspring) < POPULATION:
            child = crossover(select_parent(), select_parent(), rng)
            offspring.append(mutate(child, rng))
        population = offspring


def weight(text):
    value = float(text)
    if not 0 <= value <= 1:
        raise argparse.ArgumentTypeError("must be between 0 and 1")
    return value


def main():
    parser = argparse.ArgumentParser(description="Evolve an intersection design within a budget.")
    parser.add_argument("--value-weight", type=weight, default=VALUE_WEIGHT,
                        help="0 = traffic only, 1 = cost only, 0.5 = even split")
    parser.add_argument("--budget", type=float, default=BUDGET, help="Target spend in dollars")
    parser.add_argument("--view", action="store_true", help="Watch the winner after evaluation")
    args = parser.parse_args()
    if args.budget <= 0:
        parser.error("--budget must be positive")
    run_experiment(evolve, BASELINE, value_weight=args.value_weight, budget=args.budget,
                   view=args.view)


if __name__ == "__main__":
    main()
