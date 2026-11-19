"""SUMO side of the intersection experiment. Run signal_timing_ea.py.

build_network turns a Design's layout genes (control type, lanes, turning lanes,
right-turn pockets) into a SUMO network; write_signal turns its timer genes into
a signal program; score runs traffic and returns mean delay per car.
"""
from functools import lru_cache
import math
from pathlib import Path
import random
import shutil
import subprocess
from tempfile import TemporaryDirectory
from typing import NamedTuple
import xml.etree.ElementTree as ET

from intersection_design import (ARMS, POCKET_LENGTH, SPEED_LIMIT, cost_breakdown,
                                 describe, total_cost, value_score)


CAR_COUNT = 800
ARRIVAL_WINDOW = 1800       # 30 minutes of scheduled departures
EAST_WEST_SHARE = 0.70      # try 0.30 and see whether the design changes
TRAIN_SEED, TEST_SEED = 42, 314
ARM_LENGTH = 220            # metres from the junction to the end of each arm
ROUNDABOUT_RADIUS = 25
HEADING = {"N": (0, 1), "E": (1, 0), "S": (0, -1), "W": (-1, 0)}
TURNS = {1: "left", 2: "straight", 3: "right"}   # clockwise steps from approach to exit


class Segment(NamedTuple):
    edge: str
    start: float        # distance from the junction where this piece of road begins
    left: bool          # carries the left-turn lane
    pocket: bool        # carries the right-turn pocket


def write_xml(root, path):
    ET.ElementTree(root).write(path, encoding="utf-8", xml_declaration=True)


def axis(arm):
    return "NS" if arm in "NS" else "EW"


def lanes_on(design, arm):
    return design.ns_lanes if arm in "NS" else design.ew_lanes


def exit_arm(approach, turn):
    return ARMS[(ARMS.index(approach) + turn) % len(ARMS)]


def point(arm, distance):
    dx, dy = HEADING[arm]
    return {"x": f"{dx * distance:.2f}", "y": f"{dy * distance:.2f}"}


def layout_key(design):
    """Genes that change the road itself. Designs sharing these share one network."""
    return (design.control, design.ns_lanes, design.ew_lanes,
            design.turn_lane_m, design.right_pockets)


def lane_roles(through, left, pocket):
    """SUMO numbers lanes from the kerb: pocket, then through lanes, then the left-turn lane."""
    return ["pocket"] * pocket + ["through"] * through + ["left"] * left


def approach_segments(design, arm):
    """Cut the approach wherever a turning lane or pocket begins, listed far to near."""
    pocket = design.right_pockets[ARMS.index(arm)]
    starts = {ARM_LENGTH}
    if design.turn_lane_m:
        starts.add(design.turn_lane_m)
    if pocket:
        starts.add(POCKET_LENGTH)
    return [Segment(arm + "_in" + (f"_{i}" if i else ""), start,
                    design.turn_lane_m >= start, pocket and POCKET_LENGTH >= start)
            for i, start in enumerate(sorted(starts, reverse=True))]


def connect(connections, from_edge, to_edge, from_lane, to_lane):
    ET.SubElement(connections, "connection", {"from": from_edge, "to": to_edge,
                                              "fromLane": str(from_lane), "toLane": str(to_lane)})


def join_segments(connections, near, far, through):
    """Feed every lane of the nearer segment; new bays branch off the lane beside them."""
    far_roles = lane_roles(through, far.left, far.pocket)
    for lane, role in enumerate(lane_roles(through, near.left, near.pocket)):
        if role == "through":
            source = far_roles.index("through") + lane - near.pocket
        elif role in far_roles:
            source = far_roles.index(role)
        elif role == "pocket":
            source = far_roles.index("through")                     # kerb-side through lane
        else:
            source = len(far_roles) - 1 - far_roles[::-1].index("through")  # inner through lane
        connect(connections, far.edge, near.edge, source, lane)


def turn_connections(connections, design, arm, last):
    """Pin each movement to its lanes, so bays only serve their own turn."""
    through = lanes_on(design, arm)
    roles = lane_roles(through, last.left, last.pocket)
    for turn, move in TURNS.items():
        target = exit_arm(arm, turn)
        if move == "straight":
            first = roles.index("through")
            pairs = [(first + k, k) for k in range(through)]
        elif move == "right":
            pairs = [(0, 0)]                                    # kerb lane: pocket if any
        else:
            pairs = [(len(roles) - 1, lanes_on(design, target) - 1)]  # inner lane: bay if any
        for from_lane, to_lane in pairs:
            connect(connections, last.edge, target + "_out", from_lane, to_lane)


def add_roundabout(nodes, edges, lanes):
    """Four ring nodes joined counter-clockwise, as traffic circulates in Canada."""
    order = "NWSE"
    for arm in order:
        ET.SubElement(nodes, "node", id="r" + arm, type="priority",
                      **point(arm, ROUNDABOUT_RADIUS))
    ring = []
    for a, b in zip(order, order[1:] + order[0]):
        (ax, ay), (bx, by) = HEADING[a], HEADING[b]
        mx, my = (ax + bx) / math.sqrt(2), (ay + by) / math.sqrt(2)   # 45 degrees between
        r = ROUNDABOUT_RADIUS
        shape = f"{ax * r},{ay * r} {mx * r:.2f},{my * r:.2f} {bx * r},{by * r}"
        ring.append(f"ring_{a}{b}")
        ET.SubElement(edges, "edge", {"id": ring[-1], "from": "r" + a, "to": "r" + b,
                                      "numLanes": str(lanes), "speed": "8.3", "shape": shape})
    ET.SubElement(edges, "roundabout", nodes=" ".join("r" + a for a in order),
                  edges=" ".join(ring))


def build_network(folder, design):
    """Compile one layout. Return {link index: (axis, movement)} for a signal, else None."""
    nodes, edges, connections = (ET.Element(tag) for tag in ("nodes", "edges", "connections"))
    roundabout = design.control == "roundabout"
    if roundabout:
        add_roundabout(nodes, edges, max(design.ns_lanes, design.ew_lanes))
    else:
        node_type = {"stop": "allway_stop", "signal": "traffic_light"}[design.control]
        ET.SubElement(nodes, "node", id="centre", x="0", y="0", type=node_type)

    for arm in ARMS:
        lanes = lanes_on(design, arm)
        junction = "r" + arm if roundabout else "centre"
        ET.SubElement(nodes, "node", id=arm, **point(arm, ARM_LENGTH))
        segments = approach_segments(design, arm)
        for i, seg in enumerate(segments):
            start = arm if i == 0 else f"{arm}{seg.start}"
            end = f"{arm}{segments[i + 1].start}" if i + 1 < len(segments) else junction
            if i:
                ET.SubElement(nodes, "node", id=start, type="priority", **point(arm, seg.start))
                join_segments(connections, seg, segments[i - 1], lanes)
            ET.SubElement(edges, "edge", {"id": seg.edge, "from": start, "to": end,
                                          "numLanes": str(lanes + seg.left + seg.pocket),
                                          "speed": str(SPEED_LIMIT)})
        ET.SubElement(edges, "edge", {"id": arm + "_out", "from": junction, "to": arm,
                                      "numLanes": str(lanes), "speed": str(SPEED_LIMIT)})
        if not roundabout:
            turn_connections(connections, design, arm, segments[-1])

    write_xml(nodes, folder / "nodes.xml")
    write_xml(edges, folder / "roads.xml")
    write_xml(connections, folder / "lanes.xml")
    subprocess.run(["netconvert", "-n", "nodes.xml", "-e", "roads.xml", "-x", "lanes.xml",
                    "--no-turnarounds", "-o", "network.net.xml"],
                   cwd=folder, check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    if design.control != "signal":
        return None
    net = ET.parse(folder / "network.net.xml")
    moves = {"l": "left", "L": "left", "r": "right", "R": "right"}
    return {int(c.get("linkIndex")): (axis(c.get("from")[0]), moves.get(c.get("dir"), "straight"))
            for c in net.findall("./connection[@tl='centre']")}


def signal_phases(links, design):
    """Each axis gets an optional protected left arrow, then its main green, then yellow."""
    def state(rule):
        return "".join(rule(*links[i]) for i in range(len(links)))

    phases = []
    for active, green in (("NS", design.ns_green), ("EW", design.ew_green)):
        if design.turn_arrow:
            # Lefts go first with priority; rights may slip through if clear.
            phases.append((design.arrow_green, state(
                lambda a, m: "r" if a != active else "G" if m == "left" else
                             "g" if m == "right" else "r")))
            phases.append((design.yellow, state(
                lambda a, m: "r" if a != active else "y" if m == "left" else
                             "g" if m == "right" else "r")))
        # Main green: straight and right have priority, lefts yield to oncoming cars.
        phases.append((green, state(lambda a, m: "r" if a != active else "g" if m == "left" else "G")))
        phases.append((design.yellow, state(lambda a, m: "y" if a == active else "r")))
    return phases


def write_signal(folder, links, design):
    logic = ET.Element("tlLogic", id="centre", type="static", programID="candidate", offset="0")
    for duration, state in signal_phases(links, design):
        ET.SubElement(logic, "phase", duration=str(duration), state=state)
    extra = ET.Element("additional")
    extra.append(logic)
    write_xml(extra, folder / "signal.add.xml")


def make_traffic(path, seed):
    """Draw a fixed set of trips. Each candidate faces exactly the same drivers."""
    rng = random.Random(seed)
    trips = ET.Element("routes")
    departures = sorted(rng.uniform(0, ARRIVAL_WINDOW) for _ in range(CAR_COUNT))
    for i, depart in enumerate(departures):
        approach = rng.choice("EW" if rng.random() < EAST_WEST_SHARE else "NS")
        turn = rng.choices([1, 2, 3], weights=[15, 70, 15])[0]
        ET.SubElement(trips, "trip", {"id": f"car{i}", "depart": f"{depart:.2f}",
                                      "departLane": "best", "from": approach + "_in",
                                      "to": exit_arm(approach, turn) + "_out"})
    write_xml(trips, path)


def scenario_args(traffic, seed, signal):
    args = ["-n", "network.net.xml", "-r", str(traffic), "--seed", str(seed),
            "--time-to-teleport", "-1"]
    return args + (["-a", "signal.add.xml"] if signal else [])


def score(network, design, traffic, seed):
    """Minimize delay per car: time lost to the junction plus time waiting to depart."""
    folder, links = network
    if links is not None:
        write_signal(folder, links, design)
    subprocess.run(["sumo", *scenario_args(traffic, seed, links is not None), "--no-step-log",
                    "--no-warnings", "--end", str(ARRIVAL_WINDOW + 3600),
                    "--duration-log.statistics", "--statistic-output", "result.xml"],
                   cwd=folder, check=True, stdout=subprocess.DEVNULL)
    stats = ET.parse(folder / "result.xml")
    trips = stats.find("vehicleTripStatistics")
    if int(trips.get("count")) != CAR_COUNT or int(stats.find("teleports").get("total")):
        return math.inf  # incomplete runs cannot win by omitting delayed cars
    return float(trips.get("timeLoss")) + float(trips.get("departDelay"))


def money(amount):
    return f"${amount:,.0f}"


def run_experiment(search, baseline, *, value_weight, budget, view=False):
    """Prepare SUMO, pass a one-argument fitness function to the EA, and test its winner."""
    programs = ["sumo", "netconvert"] + (["sumo-gui"] if view else [])
    for program in programs:
        if not shutil.which(program):
            raise SystemExit(f"Cannot find {program}. Add SUMO's bin folder to PATH and restart VS Code.")

    with TemporaryDirectory(prefix="intersection_ea_") as workdir:
        folder = Path(workdir)
        traffic = {"train": (folder / "train.rou.xml", TRAIN_SEED),
                   "test": (folder / "test.rou.xml", TEST_SEED)}
        for path, seed in traffic.values():
            make_traffic(path, seed)

        networks = {}

        def network_for(design):
            key = layout_key(design)
            if key not in networks:
                layout_folder = folder / f"layout{len(networks)}"
                layout_folder.mkdir()
                networks[key] = (layout_folder, build_network(layout_folder, design))
            return networks[key]

        @lru_cache(maxsize=None)
        def delay(design, run="train"):
            return score(network_for(design), design, *traffic[run])

        reference = delay(baseline)
        if math.isinf(reference):
            raise SystemExit("The baseline design cannot clear its traffic. Try a lighter demand.")

        @lru_cache(maxsize=None)
        def fitness(design):
            return value_score(delay(design), reference, total_cost(design), budget, value_weight)

        print(f"{CAR_COUNT} trips per run; {EAST_WEST_SHARE:.0%} expected to enter from E/W.\n"
              f"Budget {money(budget)}, value weight {value_weight:.2f} "
              f"(0 = traffic only, 1 = cost only). Lower score wins.\n"
              f"Baseline: {describe(baseline)} | {money(total_cost(baseline))} | "
              f"delay {reference:.2f}s per car\n")
        winner = search(fitness)
        if math.isinf(delay(winner)):
            raise SystemExit("No design cleared all scheduled trips. Try a lighter demand.")

        print(f"\n{delay.cache_info().misses} simulations across {len(networks)} road layouts.")
        print("Delay = seconds lost per car (training traffic / fresh traffic)")
        print(f"{'Design':12s} {'Cost':>12s} {'vs budget':>10s} {'Training':>9s} {'Fresh':>9s} {'Score':>7s}")
        for label, design in [("Baseline", baseline), ("EA winner", winner)]:
            cost = total_cost(design)
            print(f"{label:12s} {money(cost):>12s} {(cost - budget) / budget:>+10.0%} "
                  f"{delay(design):9.2f} {delay(design, 'test'):9.2f} {fitness(design):7.3f}")
        print(f"\nWinner: {describe(winner)}")
        for item, price in cost_breakdown(winner).items():
            print(f"  {item:20s} {money(price):>12s}")
        print(f"  {'Total':20s} {money(total_cost(winner)):>12s}")

        if view:
            layout_folder, links = network_for(winner)
            if links is not None:
                write_signal(layout_folder, links, winner)
            subprocess.run(["sumo-gui", *scenario_args(traffic["test"][0], TEST_SEED, links is not None),
                            "--start", "--delay", "30"], cwd=layout_folder, check=True)
