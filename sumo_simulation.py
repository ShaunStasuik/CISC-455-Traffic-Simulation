"""All SUMO work lives here. The evolutionary algorithm lives in intersection_ea.py.

Read run_experiment() first, then run_simulation(). Geometry/XML helpers follow.
Run a saved design separately: python sumo_simulation.py --plan results/best_plan.json
Add --view to watch it. Without --plan, this file runs the baseline only.
"""

import argparse
import csv
import time
import json
import math
import os
from pathlib import Path
import random
import shutil
import subprocess
import tempfile
import xml.etree.ElementTree as ET

# Experiment inputs, not genes. All candidates face the same synthetic traffic.
CAR_COUNT = 600
ARRIVAL_WINDOW = 1800          # seconds: departures spread over 30 minutes
EAST_WEST_SHARE = 0.70         # probability an arrival comes from E or W
LEFT_STRAIGHT_RIGHT = [15, 70, 15]
# Every design is scored on SEVERAL training traffic samples and the delays are
# averaged, so a design cannot win by getting lucky on one sample of drivers.
TRAIN_SEEDS = (42, 43, 44)
TEST_SEED = 314                # fresh traffic, used only to check the final pick
SPEED_LIMITS_KMH = {"N": 50, "E": 50, "S": 50, "W": 50}
DRAIN_SECONDS = 3600           # extra time for the final queue to clear
ARM_LENGTH = 400               # metres from the centre to the end of each approach road
ARMS = "NESW"

# Historical, illustrative CAD inputs; see COST_MODEL.md for sources/assumptions.
COST_PER_M2 = 175.0
COST_PER_ARROW = 15_000.0


def cost_breakdown(plan, road_cost=COST_PER_M2, arrow_cost=COST_PER_ARROW):
    """Cost above one general lane/direction, no pockets/bypasses/left arrows.

    This is a lane-area proxy, not a surveyed pavement footprint. General
    widening uses road-shape lengths in both directions. Added pocket storage
    uses explicit simulated lengths (excludes the 30 m drawing allowance).
    Bypasses use their complete polyline lengths, including relocated gates.
    """
    if any(not math.isfinite(v) or v < 0 for v in (road_cost, arrow_cost)):
        raise ValueError("Unit costs must be finite and non-negative.")
    graph = build_roads(plan)
    areas = dict(extra_lanes=0.0, pockets=0.0, bypasses=0.0)
    for name, road in graph["roads"].items():
        shape = road["shape"]
        length = sum(math.dist(a, b) for a, b in zip(shape, shape[1:]))
        width = road["width"]
        if name.endswith("_bypass"):
            areas["bypasses"] += length * width * road["lanes"]
        else:
            axis = "ns" if road["arm"] in "NS" else "ew"
            general_lanes = plan["lanes_" + axis]
            areas["extra_lanes"] += length * width * (general_lanes - 1)
            pocket_lanes = road["lanes"] - general_lanes
            areas["pockets"] += road.get("length", length) * width * pocket_lanes
    approaches = sum(2 for axis in ("ns", "ew")
                     if graph["signals"]["centre"][axis]["left"] > 0)
    costs = {name: round(area * road_cost, 2) for name, area in areas.items()}
    costs["left_arrow_equipment"] = round(approaches * arrow_cost, 2)
    if not math.isfinite(sum(costs.values())):
        raise ValueError("Unit costs are too large to calculate a finite total.")
    return dict(areas_m2=areas, protected_left_approaches=approaches,
                unit_costs=dict(road_per_m2=road_cost, arrow_per_approach=arrow_cost),
                costs=costs, total=round(sum(costs.values()), 2))


def write_json(path, data):
    """Save portable JSON, replacing non-finite diagnostics with null."""
    def clean(value):
        if isinstance(value, dict):
            return {key: clean(item) for key, item in value.items()}
        if isinstance(value, (list, tuple)):
            return [clean(item) for item in value]
        if isinstance(value, float) and not math.isfinite(value):
            return None
        return value
    Path(path).write_text(json.dumps(clean(data), indent=2, allow_nan=False), encoding="utf-8")


def make_evaluator(folder, traffic_runs, *, budget=None, road_cost=COST_PER_M2,
                   arrow_cost=COST_PER_ARROW):
    """Cached two-objective evaluation; check cost BEFORE expensive SUMO.

    traffic_runs is a list of (route_file, seed) pairs. A plan's delay is the
    MEAN over all of them; if any sample fails (gridlock, collision), the plan
    is infeasible.
    """
    if budget is not None and (not math.isfinite(budget) or budget < 0):
        raise ValueError("Budget must be finite and non-negative.")
    cache = {}
    counters = dict(sumo_evaluations=0, over_budget_skipped=0, simulation_failures=0)

    def evaluate(plan):
        key = tuple(sorted(plan.items()))
        if key in cache:
            return cache[key]
        breakdown = cost_breakdown(plan, road_cost, arrow_cost)
        cost = breakdown["total"]
        if budget is not None and cost > budget:
            counters["over_budget_skipped"] += 1
            item = dict(plan=dict(plan), delay=math.inf, cost=cost,
                        violation=cost - budget, reason="over budget", breakdown=breakdown)
        else:
            run_folder = folder / f"training_{counters['sumo_evaluations']:03d}"
            counters["sumo_evaluations"] += 1
            results = []
            for traffic, seed in traffic_runs:
                results.append(run_simulation(plan, run_folder / f"seed_{seed}", traffic, seed))
                if not math.isfinite(results[-1]["score"]):
                    break  # one failed sample makes the plan infeasible; skip the rest
            feasible = all(math.isfinite(r["score"]) for r in results)
            if not feasible:
                counters["simulation_failures"] += 1
            delay = sum(r["score"] for r in results) / len(results) if feasible else math.inf
            reason = next((r["reason"] for r in results if r["reason"]), "")
            item = dict(plan=dict(plan), delay=delay, cost=cost,
                        violation=0.0 if feasible else math.inf,
                        reason=reason, breakdown=breakdown,
                        simulation=[saved_metrics(r) for r in results])
        cache[key] = item
        return item

    def within_budget(plan):
        return budget is None or cost_breakdown(plan, road_cost, arrow_cost)["total"] <= budget

    return evaluate, within_budget, cache, counters


def run_experiment(search, baseline, *, recommend, preferences, road_cost=COST_PER_M2,
                   arrow_cost=COST_PER_ARROW, ea_settings=None, view=False):
    """Search, save the Pareto archive, choose by weight, then validate selection."""
    output = Path("results").resolve()
    output.mkdir(exist_ok=True)
    folder = Path(tempfile.mkdtemp(prefix="experiment_", dir=output))
    for name in ("sumo", "netconvert"):
        executable(name)
    started = time.perf_counter()
    train_runs = [(make_traffic(folder / f"train_{seed}.rou.xml", seed), seed) for seed in TRAIN_SEEDS]
    test_traffic = make_traffic(folder / "test.rou.xml", TEST_SEED)
    budget = preferences["budget"]
    evaluate, affordable, cache, counters = make_evaluator(
        folder, train_runs, budget=budget, road_cost=road_cost, arrow_cost=arrow_cost)
    print(f"{CAR_COUNT} synthetic cars x {len(TRAIN_SEEDS)} traffic samples; "
          "minimize mean delay per car AND added infrastructure cost.")
    print("Budget:", "none" if budget is None else f"${budget:,.2f}", "| Results:", folder, flush=True)
    outcome = search(evaluate, affordable)
    data = dict(schema_version=1, front=outcome["front"], search_budget=budget,
                cost_model=dict(road_per_m2=road_cost, arrow_per_approach=arrow_cost,
                                basis="Illustrative historical CAD; incremental infrastructure above minimum design"),
                settings=experiment_settings(), ea_settings=ea_settings,
                normalization={key: preferences[key] for key in ("time_reference", "cost_reference")},
                history=outcome["history"], counters=counters,
                search_seconds=time.perf_counter() - started)
    write_json(folder / "evaluations.json", list(cache.values()))
    for destination in (folder, output):
        write_json(destination / "pareto_front.json", data)
        with (destination / "pareto_front.csv").open("w", newline="", encoding="utf-8") as file:
            fields = ["id", "cost", "delay"] + list(baseline)
            writer = csv.DictWriter(file, fieldnames=fields)
            writer.writeheader()
            for i, item in enumerate(data["front"]):
                writer.writerow(dict(id=i, cost=item["cost"], delay=item["delay"], **item["plan"]))
        # Clear only the previous current-result pointers, never experiment history.
        # An unsuccessful run must not leave an old plan looking like its result.
        for name in ("best_plan.json", "recommendation.json"):
            (destination / name).unlink(missing_ok=True)
    if not data["front"]:
        raise RuntimeError(f"No feasible design discovered under this budget; diagnostics saved in {folder}. "
                           "This is not proof that no feasible design exists.")
    selected = recommend(data["front"], **preferences)
    for i, item in enumerate(data["front"]):
        mark = " <-- selected" if item is selected else ""
        print(f"  {i:2d} | ${item['cost']:12,.2f} | {item['delay']:8.2f}s{mark}")

    # Fresh traffic is validation only, never used to re-rank the training archive.
    comparison = {}
    for label, plan in (("Baseline", baseline), ("Selected", selected["plan"])):
        test = run_simulation(plan, folder / label.lower(), test_traffic, TEST_SEED)
        breakdown = cost_breakdown(plan, road_cost, arrow_cost)
        comparison[label] = dict(genes=plan, cost=breakdown["total"],
                                 within_budget=budget is None or breakdown["total"] <= budget,
                                 fresh_traffic=saved_metrics(test))
        print(f"{label:10s} | fresh traffic {test['score']:.2f}s | {test['arrived']}/{CAR_COUNT} arrived"
              f" | {test['reason'] or 'complete'}", flush=True)
    if comparison["Selected"]["fresh_traffic"]["score"] is None:
        print("WARNING: selected training design failed fresh-traffic validation; inspect comparison.json.")
    report = dict(preferences=preferences, selected=selected, comparison=comparison,
                  counters=counters, source_archive=str(folder / "pareto_front.json"),
                  note="Preference uses training results; only baseline and selected design receive fresh-traffic validation.")
    for destination in (folder, output):
        write_json(destination / "best_plan.json", selected["plan"])
        write_json(destination / "recommendation.json", report)
    write_json(folder / "comparison.json", report)
    print("Training evaluations:", counters)
    print("Saved selected plan:", output / "best_plan.json")
    if view and (folder / "selected" / "scenario.sumocfg").exists():
        show_simulation(folder / "selected" / "scenario.sumocfg")
    return selected["plan"]



def run_simulation(plan, folder, traffic, seed):
    """Build one design, run SUMO, and measure all scheduled cars.

    This is the boundary between the EA and SUMO. It does no selection,
    recombination or mutation. It only evaluates the plan it was given.
    """
    folder = Path(folder).resolve()
    folder.mkdir(parents=True, exist_ok=True)
    try:
        graph = build_roads(plan)
        network = compile_network(graph, folder, executable("netconvert"))
        signal = write_signals(graph, network, folder)
    except InvalidDesign as error:
        result = dict(score=float("inf"), arrived=0, collisions=0, teleports=0,
                      reason=str(error))
    else:
        config = write_configuration(network, signal, traffic, seed, folder)
        run_program([
            executable("sumo"), "-c", str(config), "--no-step-log",
            "--duration-log.statistics", "--statistic-output", str(folder / "statistics.xml"),
        ], folder / "simulation.log")

        stats = ET.parse(folder / "statistics.xml")
        trips = stats.find("vehicleTripStatistics")
        arrived = int(trips.get("count"))
        collisions = int(stats.find("safety").get("collisions"))
        teleports = int(stats.find("teleports").get("total"))
        # DELAY per car = time lost compared with driving at the speed limit
        # (SUMO's timeLoss: waiting at lights, slowing, queueing) plus time spent
        # waiting to even enter a congested road (departDelay). Both are already
        # per-car averages. Plain driving time, the same for every design, is excluded.
        delay = float(trips.get("timeLoss")) + float(trips.get("departDelay"))
        if arrived != CAR_COUNT or collisions or teleports:
            score = float("inf")
            reason = f"{CAR_COUNT-arrived} unfinished; {collisions} collisions; {teleports} teleports"
        else:
            score = delay
            reason = ""
        result = dict(score=score, arrived=arrived, collisions=collisions,
                      teleports=teleports, reason=reason)
    (folder / "metrics.json").write_text(json.dumps(saved_metrics(result), indent=2), encoding="utf-8")
    return result


def make_traffic(path, seed):
    """Create synthetic (departure time, origin, destination) trips, once per seed."""
    if CAR_COUNT < 1 or ARRIVAL_WINDOW <= 0 or not 0 <= EAST_WEST_SHARE <= 1:
        raise ValueError("Use positive car count/window and an E/W share in [0,1].")
    rng = random.Random(seed)
    trips = ET.Element("routes")
    ET.SubElement(trips, "vType", id="car", vClass="passenger", length="5",
                  minGap="2.5", accel="2.6", decel="4.5", sigma="0.5")
    times = sorted(rng.uniform(0, ARRIVAL_WINDOW) for _ in range(CAR_COUNT))
    for i, depart in enumerate(times):
        origin = rng.choice("EW" if rng.random() < EAST_WEST_SHARE else "NS")
        turn = rng.choices(["left", "straight", "right"], weights=LEFT_STRAIGHT_RIGHT)[0]
        exits = {"left": LEFT_EXIT, "straight": OPPOSITE, "right": RIGHT_EXIT}
        destination = exits[turn][origin]
        ET.SubElement(trips, "trip", id=f"car{i}", type="car", depart=f"{depart:.3f}",
                      **{"from": origin + "_in", "to": destination + "_out", "departLane": "best"})
    write_xml(trips, path)
    return Path(path).resolve()


def write_configuration(network, signal, traffic, seed, folder):
    """Save a replayable SUMO configuration, including incident checks."""
    root = ET.Element("configuration")
    inputs = ET.SubElement(root, "input")
    for name, path in [("net-file", network), ("additional-files", signal), ("route-files", traffic)]:
        ET.SubElement(inputs, name, value=str(Path(path).resolve()))
    timing = ET.SubElement(root, "time")
    ET.SubElement(timing, "end", value=str(ARRIVAL_WINDOW + DRAIN_SECONDS))
    ET.SubElement(timing, "step-length", value="0.2")
    processing = ET.SubElement(root, "processing")
    ET.SubElement(processing, "time-to-teleport", value="-1")
    ET.SubElement(processing, "collision.check-junctions", value="true")
    randomness = ET.SubElement(root, "random_number")
    ET.SubElement(randomness, "seed", value=str(seed))
    path = folder / "scenario.sumocfg"
    write_xml(root, path)
    return path


def saved_metrics(result):
    """Use JSON null for infeasible scores; JSON does not support infinity."""
    return {**result, "score": result["score"] if math.isfinite(result["score"]) else None}


def experiment_settings():
    return dict(cars=CAR_COUNT, arrival_window=ARRIVAL_WINDOW, east_west_share=EAST_WEST_SHARE,
                left_straight_right=LEFT_STRAIGHT_RIGHT, speed_limits_kmh=SPEED_LIMITS_KMH,
                train_seeds=list(TRAIN_SEEDS), test_seed=TEST_SEED, drain_seconds=DRAIN_SECONDS,
                arm_length_m=ARM_LENGTH)


def show_simulation(config):
    subprocess.run([executable("sumo-gui"), "-c", str(config), "--start", "--delay", "30"], check=True)


# --- Road geometry (SUMO implementation details) ---
RIGHT_EXIT = dict(N="W", E="N", S="E", W="S")
LEFT_EXIT = {value: key for key, value in RIGHT_EXIT.items()}
OPPOSITE = dict(N="S", E="W", S="N", W="E")
VECTORS = dict(N=(0, 1), E=(1, 0), S=(0, -1), W=(-1, 0))


class InvalidDesign(ValueError):
    pass


def build_roads(plan):
    """Translate the 12 genes into roads, lane connections and timing settings."""
    speeds = SPEED_LIMITS_KMH
    if set(speeds) != set(ARMS) or any(not 20 <= v <= 80 for v in speeds.values()):
        raise ValueError("Provide a speed in 20..80 km/h for every origin arm")
    nodes = {"centre": dict(position=[0, 0], control="signal", radius=6)}
    roads, movements, chains = {}, [], {}

    def road(name, start, end, lanes, arm, curve=False, length=None, extra_offset=0):
        first, last = nodes[start]["position"], nodes[end]["position"]
        if curve:
            control = [0.55 * (a + b) for a, b in zip(first, last)]
            shape = [
                [
                    (1 - t) ** 2 * first[i]
                    + 2 * (1 - t) * t * control[i]
                    + t * t * last[i]
                    for i in range(2)
                ]
                for t in [j / 12 for j in range(13)]
            ]
        else:
            dx, dy = last[0] - first[0], last[1] - first[1]
            d = math.hypot(dx, dy)
            offset = 2 + lanes * 3.2 / 2 + extra_offset
            shift = (offset * dy / d, -offset * dx / d)
            shape = [[p[i] + shift[i] for i in range(2)] for p in (first, last)]
        roads[name] = dict(
            start=start,
            end=end,
            lanes=lanes,
            width=3.2,
            speed=min(8.3, speeds[arm] / 3.6) if curve else speeds[arm] / 3.6,
            priority=1 if curve else 3,
            shape=shape,
            arm=arm,
        )
        if length is not None:
            roads[name]["length"] = length

    def connect(first, second, pairs):
        for a, b in pairs:
            movements.append(
                dict(from_edge=first, to_edge=second, from_lane=a, to_lane=b)
            )

    for arm in ARMS:
        axis = "ns" if arm in "NS" else "ew"
        pocket = plan["pocket_" + axis]
        # 30 m graphical allowance for the central junction. Explicit edge
        # lengths below make the added storage sum to the requested metres.
        start = pocket + 30
        gate = 150 if not pocket or abs(start - 150) >= 70 else start + 70
        if max(gate, start) + 50 > ARM_LENGTH:
            raise ValueError(f"ARM_LENGTH {ARM_LENGTH} m is too short for a {pocket} m pocket; "
                             "lengthen the approach roads or shorten the pocket range.")
        points = [(ARM_LENGTH, arm), (gate, arm + "_gate"), (0, "centre")]
        if pocket:
            points.append((start, arm + "_pocket"))
        points.sort(reverse=True)
        x, y = VECTORS[arm]
        for distance, name in points[:-1]:
            nodes[name] = dict(
                position=[distance * x, distance * y],
                control="priority",
                radius=(
                    15
                    if name.endswith("_pocket")
                    else 5 if name.endswith("_gate") else 0
                ),
            )
        chains[arm] = points

    for arm in ARMS:
        axis = "ns" if arm in "NS" else "ew"
        n, pocket = plan["lanes_" + axis], plan["pocket_" + axis]
        points = chains[arm]
        incoming, outgoing = [], []
        for i, ((d1, a), (d2, b)) in enumerate(zip(points, points[1:])):
            inside = pocket > 0 and d1 <= pocket + 30
            name = (
                arm + "_in"
                if i == 0
                else (arm + "_to_centre" if b == "centre" else arm + f"_in_{i}")
            )
            exit_name = (
                arm + "_out"
                if i == 0
                else (arm + "_from_centre" if b == "centre" else arm + f"_out_{i}")
            )
            # 'length' is a documented SUMO simulation length, distinct from drawing.
            length = d1 - d2 - (30 if b == "centre" else 0) if inside else None
            road(
                name,
                a,
                b,
                n + int(inside),
                arm,
                length=length,
                extra_offset=3.2 if pocket and not inside else 0,
            )
            road(exit_name, b, a, n, OPPOSITE[arm])
            incoming.append(name)
            outgoing.append(exit_name)
        for i in range(len(incoming) - 1):
            na, nb = roads[incoming[i]]["lanes"], roads[incoming[i + 1]]["lanes"]
            pairs = [(j, j) for j in range(na)]
            if nb > na:
                pairs.append((na - 1, nb - 1))
            connect(incoming[i], incoming[i + 1], pairs)
            connect(outgoing[i + 1], outgoing[i], [(j, j) for j in range(n)])
        gate_index = next(
            i for i, (_, node) in enumerate(points) if node == arm + "_gate"
        )
        chains[arm] = dict(incoming=incoming, outgoing=outgoing, gate_index=gate_index)

    for arm in ARMS:
        chain = chains[arm]
        if plan["bypass_" + arm.lower()]:
            dest = RIGHT_EXIT[arm]
            name = arm + "_bypass"
            road(name, arm + "_gate", dest + "_gate", 1, arm, curve=True)
            connect(chain["incoming"][chain["gate_index"] - 1], name, [(0, 0)])
            other = chains[dest]
            connect(name, other["outgoing"][other["gate_index"] - 1], [(0, 0)])
        axis = "ns" if arm in "NS" else "ew"
        n = plan["lanes_" + axis]
        inc = chain["incoming"][-1]
        for turn, dest in [
            ("right", RIGHT_EXIT[arm]),
            ("straight", OPPOSITE[arm]),
            ("left", LEFT_EXIT[arm]),
        ]:
            out = chains[dest]["outgoing"][-1]
            pairs = (
                [(0, 0)]
                if turn == "right"
                else (
                    [(roads[inc]["lanes"] - 1, roads[out]["lanes"] - 1)]
                    if turn == "left"
                    else [(j, j) for j in range(n)]
                )
            )
            connect(inc, out, pairs)
            for movement in movements[-len(pairs) :]:
                movement["turn"] = turn
    # Dormant arrow genes do not change the phenotype when a pocket is absent.
    settings = {}
    for axis in ("ns", "ew"):
        floor = max(3, 1 + max(speeds[a] for a in axis.upper()) / 3.6 / (2 * 3))
        settings[axis] = dict(
            green=plan["green_" + axis],
            left=plan["left_green_" + axis] if plan["pocket_" + axis] else 0,
            yellow=round(floor, 3),  # safety rule from the speed limit, not a gene
        )
    return dict(
        nodes=nodes, roads=roads, movements=movements, signals={"centre": settings}
    )

# --- XML and network compilation ---
def write_xml(root, path):
    ET.indent(root)
    ET.ElementTree(root).write(path, encoding="utf-8", xml_declaration=True)


def executable(name):
    found = shutil.which(name)
    suffix = ".exe" if os.name == "nt" else ""
    candidate = Path(os.environ.get("SUMO_HOME", "")) / "bin" / (name + suffix)
    if candidate.is_file():
        return str(candidate.resolve())
    try:
        import sumo

        candidate = Path(sumo.__file__).parent / "bin" / (name + suffix)
        if candidate.is_file():
            return str(candidate)
    except ImportError:
        pass
    if found:
        return found
    raise RuntimeError(
        f"Cannot find {name}. Add SUMO's bin folder to PATH and restart VS Code."
    )


def run_program(command, log, timeout=120):
    """Save diagnostics; infrastructure errors should be visible to the user."""
    result = subprocess.run(command, capture_output=True, text=True, timeout=timeout)
    log.write_text(result.stdout + result.stderr, encoding="utf-8")
    if result.returncode:
        raise RuntimeError(
            f"{Path(command[0]).name} failed. See {log}\n{result.stderr[-1500:]}"
        )


def compile_network(graph, folder, netconvert):
    folder.mkdir(parents=True, exist_ok=True)
    nodes, roads = graph["nodes"], graph["roads"]
    xml_nodes, xml_roads, xml_connections = (
        ET.Element("nodes"),
        ET.Element("edges"),
        ET.Element("connections"),
    )
    for name, node in nodes.items():
        x, y = node["position"]
        control = {
            "signal": "traffic_light",
            "priority": "priority",
            "stop": "allway_stop",
        }[node["control"]]
        ET.SubElement(
            xml_nodes,
            "node",
            id=name,
            x=str(x),
            y=str(y),
            type=control,
            radius=str(node["radius"]),
        )
    for name, road in roads.items():
        ET.SubElement(
            xml_roads,
            "edge",
            {
                **({"length": str(road["length"])} if "length" in road else {}),
                "id": name,
                "from": road["start"],
                "to": road["end"],
                "numLanes": str(road["lanes"]),
                "speed": str(road["speed"]),
                "priority": str(road["priority"]),
                "width": str(road["width"]),
                "spreadType": "center",
                "shape": " ".join(f"{x:.4f},{y:.4f}" for x, y in road["shape"]),
            },
        )
    expected = {
        (m["from_edge"], m["to_edge"], m["from_lane"], m["to_lane"])
        for m in graph["movements"]
    }
    road_pairs = {(first, second) for first, second, _, _ in expected}
    # Explicitly forbid unlisted movements, including boundary turnarounds.
    for first, road in roads.items():
        for second, following in roads.items():
            if road["end"] == following["start"] and (first, second) not in road_pairs:
                ET.SubElement(xml_connections, "delete", {"from": first, "to": second})
    for first, second, first_lane, second_lane in sorted(expected):
        ET.SubElement(
            xml_connections,
            "connection",
            {
                "from": first,
                "to": second,
                "fromLane": str(first_lane),
                "toLane": str(second_lane),
            },
        )
    for root, name in [
        (xml_nodes, "nodes.xml"),
        (xml_roads, "roads.xml"),
        (xml_connections, "movements.xml"),
    ]:
        write_xml(root, folder / name)
    network = folder / "network.net.xml"
    run_program(
        [
            netconvert,
            "-n",
            str(folder / "nodes.xml"),
            "-e",
            str(folder / "roads.xml"),
            "-x",
            str(folder / "movements.xml"),
            "--no-turnarounds",
            "-o",
            str(network),
        ],
        folder / "build.log",
    )
    compiled = ET.parse(network)
    actual = {
        (
            link.get("from"),
            link.get("to"),
            int(link.get("fromLane")),
            int(link.get("toLane")),
        )
        for link in compiled.findall("connection")
        if link.get("from") in roads and link.get("to") in roads
    }
    if actual != expected:
        raise InvalidDesign(
            f"Compiled lane movements differ. Missing {expected-actual}, extra {actual-expected}"
        )
    for edge in compiled.findall("edge"):
        if edge.get("id") in roads and any(
            float(lane.get("length")) < 5 for lane in edge.findall("lane")
        ):
            raise InvalidDesign(
                "Junction geometry consumes a road segment " + edge.get("id")
            )
    return network



# --- Signal phases and conflict checks ---
def signal_phases(graph, compiled, junction, settings):
    links = compiled.findall(f"./connection[@tl='{junction}']")
    node = compiled.find(f"./junction[@id='{junction}']")
    internal = node.get("intLanes").split()
    requests = {int(r.get("index")): r for r in node.findall("request")}
    mapping = {
        (m["from_edge"], m["to_edge"], m["from_lane"], m["to_lane"]): m
        for m in graph["movements"]
    }
    records = {}
    for link in links:
        via, visited = link.get("via"), set()
        while via not in internal and via not in visited:
            visited.add(via)
            edge, lane = via.rsplit("_", 1)
            nxt = compiled.find(f"./connection[@from='{edge}'][@fromLane='{lane}']")
            if nxt is None or not nxt.get("via"):
                break
            via = nxt.get("via")
        if via not in internal:
            raise InvalidDesign("Cannot resolve TLS conflict indices")
        key = (
            link.get("from"),
            link.get("to"),
            int(link.get("fromLane")),
            int(link.get("toLane")),
        )
        records[int(link.get("linkIndex"))] = dict(
            request=internal.index(via),
            arm=graph["roads"][link.get("from")]["arm"],
            turn=mapping[key]["turn"],
        )

    def bit(a, b, field):
        return (
            requests[records[a]["request"]].get(field)[-(records[b]["request"] + 1)]
            == "1"
        )

    def conflict(a, b):
        return bit(a, b, "foes") or bit(b, a, "foes")

    phases = []
    for axis in ("ns", "ew"):
        selected = {i: r for i, r in records.items() if r["arm"] in axis.upper()}
        left = {i: "G" for i, r in selected.items() if r["turn"] == "left"}
        if settings[axis]["left"]:
            indices = list(left)
            groups = (
                [left]
                if not any(
                    conflict(a, b)
                    for p, a in enumerate(indices)
                    for b in indices[p + 1 :]
                )
                else [
                    {
                        i: "G"
                        for i, r in selected.items()
                        if r["turn"] == "left" and r["arm"] == arm
                    }
                    for arm in axis.upper()
                ]
            )
            for group in groups:
                phases.append(
                    dict(
                        name=axis + "_left_" + str(len(phases)),
                        active=group,
                        duration=settings[axis]["left"],
                        yellow=settings[axis]["yellow"],
                    )
                )
        active = {
            i: ("g" if r["turn"] == "left" else "G")
            for i, r in selected.items()
            if r["turn"] != "left" or not settings[axis]["left"]
        }
        for i, state in active.items():
            if state == "g":
                for j, other in active.items():
                    if other == "G" and conflict(i, j) and not bit(i, j, "response"):
                        raise InvalidDesign(
                            "Permissive left lacks underlying yield priority"
                        )
        phases.append(
            dict(
                name=axis + "_main",
                active=active,
                duration=settings[axis]["green"],
                yellow=settings[axis]["yellow"],
            )
        )
    # Never silently grant simultaneous priority to conflicting movements.
    for phase in phases:
        priority = [i for i, s in phase["active"].items() if s == "G"]
        for pos, a in enumerate(priority):
            if any(conflict(a, b) for b in priority[pos + 1 :]):
                raise InvalidDesign("Conflicting protected greens in " + phase["name"])
    covered = set().union(*(p["active"] for p in phases))
    if covered != set(records):
        raise InvalidDesign("Some movements have no green")
    return phases, max(records) + 1


def write_signals(graph, network, folder):
    compiled = ET.parse(network)
    extra = ET.Element("additional")
    audit = {}
    for junction, settings in graph["signals"].items():
        phases, count = signal_phases(graph, compiled, junction, settings)
        logic = ET.SubElement(
            extra,
            "tlLogic",
            id=junction,
            type="static",
            programID="candidate",
            offset="0",
        )
        for phase in phases:
            green, yellow = ["r"] * count, ["r"] * count
            for i, state in phase["active"].items():
                green[i], yellow[i] = state, "y"
            ET.SubElement(
                logic,
                "phase",
                duration=str(phase["duration"]),
                state="".join(green),
                name=phase["name"],
            )
            ET.SubElement(
                logic, "phase", duration=str(phase["yellow"]), state="".join(yellow)
            )
            ET.SubElement(logic, "phase", duration="3", state="r" * count)
        audit[junction] = phases
    path = folder / "signals.add.xml"
    ET.indent(extra)
    ET.ElementTree(extra).write(path, encoding="utf-8", xml_declaration=True)
    (folder / "phase_audit.json").write_text(json.dumps(audit, indent=2))
    return path


def main():
    # Only this standalone entry point imports the EA's representation validator.
    # The simulation/evaluation functions above do not call any EA operators.
    from intersection_ea import BASELINE, validate

    parser = argparse.ArgumentParser(description="Run SUMO without running the EA")
    parser.add_argument("--plan", type=Path, help="Saved JSON genome; default: BASELINE")
    parser.add_argument("--view", action="store_true", help="Open this simulation in SUMO GUI")
    args = parser.parse_args()
    plan = validate(json.loads(args.plan.read_text()) if args.plan else BASELINE)
    output = Path("results").resolve()
    output.mkdir(exist_ok=True)
    folder = Path(tempfile.mkdtemp(prefix="simulation_", dir=output))
    traffic = make_traffic(folder / "traffic.rou.xml", TEST_SEED)
    result = run_simulation(plan, folder, traffic, TEST_SEED)
    print(f"Mean delay: {result['score']:.2f}s per car; arrived {result['arrived']}/{CAR_COUNT}")
    print(result["reason"] or "Complete, without collisions or teleports.")
    print("Results:", folder)
    if args.view and (folder / "scenario.sumocfg").exists():
        show_simulation(folder / "scenario.sumocfg")


if __name__ == "__main__":
    main()
