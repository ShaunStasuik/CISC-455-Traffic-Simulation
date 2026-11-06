"""CISC 455: evolutionary search for green times at a four-way signal.

Keep sumo_simulation.py beside this file. Run: py signal_timing_ea.py
Add --view to watch the winning signal after the search.

Read evolve() first. A plan is (north_south_seconds, east_west_seconds).
The EA asks fitness(plan) for a number. Its only job is finding a smaller one.
"""
import argparse
import random

from sumo_simulation import run_experiment


# One individual is (north_south_green_seconds, east_west_green_seconds).
BASELINE = (30, 30)
MIN_GREEN, MAX_GREEN = 10, 60
POPULATION = 10
GENERATIONS = 8
MUTATION_PROBABILITY = 0.4  # per gene
MUTATION_STEP = 6           # maximum seconds added or removed by a mutation
EA_SEED = 17


def mutate(plan, rng):
    """Give each green time an independent chance to change."""
    child = list(plan)
    for i, seconds in enumerate(child):
        if rng.random() < MUTATION_PROBABILITY:
            delta = rng.choice([-1, 1]) * rng.randint(1, MUTATION_STEP)
            child[i] = max(MIN_GREEN, min(MAX_GREEN, seconds + delta))
    return tuple(child)


def evolve(fitness):
    """Return the best timing pair. Calling fitness(plan) gives seconds per car."""
    rng = random.Random(EA_SEED)
    # Initialization: include one known plan, then fill the population at random.
    population = [BASELINE]
    for _ in range(POPULATION - 1):
        population.append((rng.randint(MIN_GREEN, MAX_GREEN), rng.randint(MIN_GREEN, MAX_GREEN)))

    def select_parent():
        contestants = rng.sample(population, 3)
        return min(contestants, key=fitness)  # tournament of three

    for generation in range(GENERATIONS + 1):
        # Rank candidates by their simulated score. Lower is better.
        best = min(population, key=fitness)
        print(f"Generation {generation:2d} | N/S {best[0]:2d}s, E/W {best[1]:2d}s"
              f" | mean journey {fitness(best):.2f}s", flush=True)
        if generation == GENERATIONS:
            return best
        offspring = [best]  # elitism: retain the current best
        while len(offspring) < POPULATION:
            parent_a, parent_b = select_parent(), select_parent()
            child = (parent_a[0], parent_b[1])  # one-point crossover
            offspring.append(mutate(child, rng))
        population = offspring


def main():
    parser = argparse.ArgumentParser(description="Evolve green times at a four-way signal.")
    parser.add_argument("--view", action="store_true", help="Watch the winner after evaluation")
    args = parser.parse_args()
    run_experiment(evolve, BASELINE, view=args.view)


if __name__ == "__main__":
    main()
