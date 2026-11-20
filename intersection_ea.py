"""A small, readable EA. Start with BASELINE, then read evolve().

One individual is a dictionary describing an intersection.
evaluate(plan) returns journey time, infrastructure cost and constraint status.
NSGA-II searches both objectives; a user weight chooses a result afterwards.
Run: python intersection_ea.py
"""

import argparse
import random
import json
import math
from pathlib import Path
from statistics import median

from sumo_simulation import run_experiment, write_json, COST_PER_M2, COST_PER_ARROW


# 1. REPRESENTATION: all 14 genes are here.
# NS values apply to both north and south. EW values apply to east and west.
BASELINE = {
    "pocket_ns": 50,       # left-turn lane storage length, metres; 0 = absent
    "pocket_ew": 50,
    "left_green_ns": 8,    # protected left-arrow seconds; 0 = no protected phase
    "left_green_ew": 8,
    "lanes_ns": 1,         # general lanes PER DIRECTION, excluding left pocket
    "lanes_ew": 1,
    "bypass_n": False,     # right-turn slip road from north to west
    "bypass_e": False,     # east to north
    "bypass_s": False,     # south to east
    "bypass_w": False,     # west to south
    "green_ns": 30,        # main green seconds
    "green_ew": 30,
    "yellow_ns": 4,        # requested yellow seconds (speed minimum also applies)
    "yellow_ew": 4,
}

# Legal values. Change the ranges here when experimenting.
# Pocket lengths and arrow times allow zero, followed by a positive range.
GENE_VALUES = {
    "pocket_ns": [0] + list(range(12, 1001)),
    "pocket_ew": [0] + list(range(12, 1001)),
    "left_green_ns": [0] + list(range(5, 31)),
    "left_green_ew": [0] + list(range(5, 31)),
    "lanes_ns": [1, 2],
    "lanes_ew": [1, 2],
    "bypass_n": [False, True],
    "bypass_e": [False, True],
    "bypass_s": [False, True],
    "bypass_w": [False, True],
    "green_ns": list(range(10, 61)),
    "green_ew": list(range(10, 61)),
    "yellow_ns": list(range(3, 7)),
    "yellow_ew": list(range(3, 7)),
}

POPULATION_SIZE = 8
GENERATIONS = 5
MUTATION_PROBABILITY = 1 / 14  # independently applied to each gene
TOURNAMENT_SIZE = 2
EA_SEED = 17


def validate(plan):
    """Catch misspelled genes and illegal values before running SUMO."""
    if set(plan) != set(GENE_VALUES):
        raise ValueError("A plan must contain exactly the 14 genes in BASELINE.")
    for gene, value in plan.items():
        expected_type = bool if gene.startswith("bypass_") else int
        if type(value) is not expected_type or value not in GENE_VALUES[gene]:
            raise ValueError(f"Invalid value for {gene}: {value!r}")
    return dict(plan)


def random_value(gene, rng):
    """Pick a legal value. Give 'feature absent' a reasonable chance to occur."""
    values = GENE_VALUES[gene]
    if gene.startswith("pocket_") or gene.startswith("left_green_"):
        if rng.random() < 0.2:
            return 0
        return rng.choice(values[1:])
    return rng.choice(values)


def random_individual(rng):
    """Initialization: choose a value for every gene."""
    plan = {}
    for gene in GENE_VALUES:
        plan[gene] = random_value(gene, rng)
    return plan


# 2. RECOMBINATION: uniform crossover.
def recombine(parent_a, parent_b, rng):
    """For each gene, choose either parent's value with equal probability."""
    child = {}
    for gene in GENE_VALUES:
        if rng.random() < 0.5:
            child[gene] = parent_a[gene]
        else:
            child[gene] = parent_b[gene]
    return child


# 3. MUTATION: boolean flip or numeric random reset.
def mutate(plan, rng):
    """Return a changed copy; never edit a parent or the elite in place."""
    child = dict(plan)
    for gene in GENE_VALUES:
        if rng.random() < MUTATION_PROBABILITY:
            if gene.startswith("bypass_"):
                child[gene] = not child[gene]
            else:
                child[gene] = random_value(gene, rng)
    return child


# 4. MULTI-OBJECTIVE SELECTION: minimize journey_time AND cost.
def plan_key(plan):
    return tuple(plan[gene] for gene in GENE_VALUES)


def dominates(a, b):
    """Feasible beats infeasible; otherwise use Pareto dominance.

    Over-budget candidates have violation = dollars over budget. Simulation
    failures have infinite violation. Neither can enter the returned archive.
    """
    if a["violation"] != b["violation"]:
        return a["violation"] < b["violation"]
    if a["violation"] > 0:
        return False
    return (a["journey_time"] <= b["journey_time"] and a["cost"] <= b["cost"]
            and (a["journey_time"] < b["journey_time"] or a["cost"] < b["cost"]))


def non_dominated_sort(population):
    """Return fronts of population indices; front 0 has no dominators."""
    defeated = [[] for _ in population]
    counts = [0] * len(population)
    for i in range(len(population)):
        for j in range(i + 1, len(population)):
            if dominates(population[i], population[j]):
                defeated[i].append(j)
                counts[j] += 1
            elif dominates(population[j], population[i]):
                defeated[j].append(i)
                counts[i] += 1
    front = [i for i, count in enumerate(counts) if count == 0]
    fronts = []
    while front:
        fronts.append(front)
        following = []
        for i in front:
            for j in defeated[i]:
                counts[j] -= 1
                if counts[j] == 0:
                    following.append(j)
        front = following
    return fronts


def crowding_distance(population, front):
    """Large distance = less crowded in objective space.

    Each objective is normalized by its range WITHIN this front. This is
    separate from the fixed reference values used for user preferences.
    """
    distance = {i: 0.0 for i in front}
    for objective in ("journey_time", "cost"):
        ordered = sorted(front, key=lambda i: population[i][objective])
        if not ordered:
            continue
        low, high = population[ordered[0]][objective], population[ordered[-1]][objective]
        if not math.isfinite(low) or not math.isfinite(high) or high == low:
            continue  # constant or unavailable objective conveys no spacing
        distance[ordered[0]] = distance[ordered[-1]] = math.inf
        for position in range(1, len(ordered) - 1):
            previous = population[ordered[position - 1]][objective]
            following = population[ordered[position + 1]][objective]
            distance[ordered[position]] += (following - previous) / (high - low)
    return distance


def rank_and_distance(population):
    ranks, distances = {}, {}
    for rank, front in enumerate(non_dominated_sort(population)):
        ranks.update({i: rank for i in front})
        distances.update(crowding_distance(population, front))
    return ranks, distances


def select_parent(population, ranks, distances, rng):
    contestants = rng.choices(range(len(population)), k=TOURNAMENT_SIZE)
    # Random contestant order also breaks exact ties without index preference.
    winner = min(contestants, key=lambda i: (ranks[i], -distances[i]))
    return population[winner]["plan"]


def select_survivors(combined, size, rng):
    """NSGA-II elitism: fill from parent+offspring fronts, then crowding."""
    survivors = []
    for front in non_dominated_sort(combined):
        if len(survivors) + len(front) <= size:
            survivors.extend(combined[i] for i in front)
        else:
            distances = crowding_distance(combined, front)
            shuffled = list(front)
            rng.shuffle(shuffled)
            shuffled.sort(key=lambda i: distances[i], reverse=True)
            survivors.extend(combined[i] for i in shuffled[:size - len(survivors)])
            break
    return survivors


def update_archive(archive, candidates):
    """Keep feasible non-dominated results ever seen, one per objective pair."""
    unique = {}
    for item in archive + candidates:
        if item["violation"] == 0:
            unique.setdefault((item["journey_time"], item["cost"]), item)
    items = list(unique.values())
    if not items:
        return []
    front = non_dominated_sort(items)[0]
    return sorted((items[i] for i in front), key=lambda item: (item["cost"], item["journey_time"]))


def minimum_design():
    """Zero ADDED infrastructure cost; baseline timing, one lane/direction."""
    plan = dict(BASELINE)
    for gene in plan:
        if gene.startswith(("pocket_", "left_green_")):
            plan[gene] = 0
        elif gene.startswith("bypass_"):
            plan[gene] = False
        elif gene.startswith("lanes_"):
            plan[gene] = 1
    return plan


# 5. THE EA. Mutation and uniform recombination remain the simple baseline ones.
def evolve(evaluate, population_size=POPULATION_SIZE, generations=GENERATIONS,
           *, seed=EA_SEED, within_budget=lambda plan: True):
    if population_size < 3 or generations < 0:
        raise ValueError("Use population >= 3 and generations >= 0.")
    rng = random.Random(seed)
    plans = [minimum_design(), dict(BASELINE)]
    while len(plans) < population_size:
        # Bound cheap initialization retries: tight budgets must not hang.
        for _ in range(40):
            candidate = random_individual(rng)
            if within_budget(candidate):
                break
        else:
            candidate = minimum_design()
            for gene in ("green_ns", "green_ew", "yellow_ns", "yellow_ew"):
                candidate[gene] = random_value(gene, rng)
        plans.append(candidate)
    population = [evaluate(plan) for plan in plans]
    archive = update_archive([], population)
    history = []
    for generation in range(generations + 1):
        feasible = [item for item in population if item["violation"] == 0]
        times = [item["journey_time"] for item in feasible]
        row = dict(generation=generation, feasible=len(feasible),
                   unique_plans=len({plan_key(item["plan"]) for item in population}),
                   best_journey=min(times) if times else None,
                   median_journey=median(times) if times else None,
                   archive_size=len(archive))
        history.append(row)
        best = f"{row['best_journey']:.2f}s" if times else "none"
        print(f"Generation {generation:2d} | fastest feasible {best}"
              f" | feasible {len(feasible)}/{population_size}"
              f" | unique {row['unique_plans']} | trade-offs {len(archive)}", flush=True)
        if generation == generations:
            return dict(front=archive, history=history)
        ranks, distances = rank_and_distance(population)
        offspring = []
        for _ in range(population_size):
            a = select_parent(population, ranks, distances, rng)
            b = select_parent(population, ranks, distances, rng)
            child = validate(mutate(recombine(a, b, rng), rng))
            offspring.append(evaluate(child))
        archive = update_archive(archive, offspring)
        population = select_survivors(population + offspring, population_size, rng)


# 6. USER DECISION. Weight is NOT used in NSGA-II ranking or variation.
def preference_score(item, cost_weight=0.5, time_reference=20.0, cost_reference=500_000.0):
    return ((1 - cost_weight) * item["journey_time"] / time_reference
            + cost_weight * item["cost"] / cost_reference)


def recommend(front, *, cost_weight=0.5, budget=None,
              time_reference=300.0, cost_reference=1_000_000.0):
    if not math.isfinite(cost_weight) or not 0 <= cost_weight <= 1:
        raise ValueError("Cost weight must be finite and in [0, 1].")
    if any(not math.isfinite(v) or v <= 0 for v in (time_reference, cost_reference)):
        raise ValueError("Normalization references must be finite and positive.")
    if budget is not None and (not math.isfinite(budget) or budget < 0):
        raise ValueError("Budget must be finite and non-negative.")
    eligible = [item for item in front if item["violation"] == 0
                and (budget is None or item["cost"] <= budget)]
    if not eligible:
        raise ValueError("No feasible discovered design fits this budget. This is not proof that none exists.")
    # Tie-break by the other objective at either endpoint; deterministic otherwise.
    return min(eligible, key=lambda item: (
        preference_score(item, cost_weight, time_reference, cost_reference),
        item["cost"], item["journey_time"], plan_key(item["plan"])))


def main():
    parser = argparse.ArgumentParser(description="14-gene NSGA-II: journey time and infrastructure cost")
    parser.add_argument("--population", type=int, default=POPULATION_SIZE)
    parser.add_argument("--generations", type=int, default=GENERATIONS)
    parser.add_argument("--seed", type=int, default=EA_SEED)
    parser.add_argument("--cost-weight", type=float, default=0.5,
                        help="0=fastest, 1=cheapest; chooses from the discovered Pareto archive")
    parser.add_argument("--budget", type=float, help="Hard cap on ADDED infrastructure cost; omit for no cap")
    parser.add_argument("--road-cost", type=float, default=None, help=f"CAD/m2 (default {COST_PER_M2})")
    parser.add_argument("--arrow-cost", type=float, default=None, help=f"CAD/approach (default {COST_PER_ARROW})")
    parser.add_argument("--time-reference", type=float, help="Fixed time scale: default 300 seconds; re-ranking inherits saved value")
    parser.add_argument("--cost-reference", type=float, help="Fixed cost scale: default 1000000 CAD; re-ranking inherits saved value")
    parser.add_argument("--select-from", type=Path, help="Re-rank saved pareto_front.json without SUMO")
    parser.add_argument("--view", action="store_true", help="Watch selected design after a new search")
    args = parser.parse_args()
    if args.population < 3 or args.generations < 0:
        parser.error("Use population >= 3 and generations >= 0.")
    preference = dict(cost_weight=args.cost_weight, budget=args.budget,
                      time_reference=300.0 if args.time_reference is None else args.time_reference,
                      cost_reference=1_000_000.0 if args.cost_reference is None else args.cost_reference)
    # Validate settings before starting SUMO, even if the resulting archive is empty.
    dummy = dict(plan=minimum_design(), violation=0, cost=0, journey_time=1)
    try:
        recommend([dummy], **preference)
        for value in (args.road_cost, args.arrow_cost):
            if value is not None and (not math.isfinite(value) or value < 0):
                raise ValueError("Unit costs must be finite and non-negative.")
        if args.select_from:
            if args.view or args.road_cost is not None or args.arrow_cost is not None:
                raise ValueError("Re-ranking supports preference/budget changes only. Replay via sumo_simulation.py; new prices require a new search.")
            data = json.loads(args.select_from.read_text())
            if data.get("schema_version") != 1:
                raise ValueError("Unsupported Pareto archive format.")
            # An omitted budget retains the original search cap.
            if args.budget is None:
                preference["budget"] = data["search_budget"]
            for name in ("time_reference", "cost_reference"):
                if getattr(args, name) is None:
                    preference[name] = data.get("normalization", {}).get(name, preference[name])
            selected = recommend(data["front"], **preference)
            destination = args.select_from.resolve().parent
            write_json(destination / "best_plan.json", selected["plan"])
            write_json(destination / "recommendation.json", dict(
                preferences=preference, selected=selected,
                score=preference_score(selected, args.cost_weight, preference["time_reference"], preference["cost_reference"]),
                source_archive=str(args.select_from.resolve()),
                note="Chosen using training results. Re-ranking does not run fresh validation."))
            print(f"Selected ${selected['cost']:,.2f}, {selected['journey_time']:.2f}s (training).")
            print("Saved:", destination / "best_plan.json")
            print("Only designs in this archive were considered; raising the cap cannot recover unexplored designs.")
            return
        run_experiment(
            lambda evaluate, affordable: evolve(evaluate, args.population, args.generations,
                                                seed=args.seed, within_budget=affordable),
            BASELINE, recommend=recommend, preferences=preference,
            road_cost=COST_PER_M2 if args.road_cost is None else args.road_cost,
            arrow_cost=COST_PER_ARROW if args.arrow_cost is None else args.arrow_cost,
            ea_settings=dict(population=args.population, generations=args.generations, seed=args.seed),
            view=args.view,
        )
    except (ValueError, RuntimeError, OSError) as error:
        parser.exit(1, f"Error: {error}\n")


if __name__ == "__main__":
    main()


