"""SUMO side of the four-way signal experiment. Run signal_timing_ea.py.

build_intersection creates the fixed roads; write_signal converts two green times
into a SUMO signal program; score runs traffic and returns mean seconds per car.
"""
from copy import deepcopy
from functools import lru_cache
from pathlib import Path
import random
import shutil
import subprocess
from tempfile import TemporaryDirectory
import xml.etree.ElementTree as ET


CAR_COUNT = 800
ARRIVAL_WINDOW = 1800       # 30 minutes of scheduled departures
EAST_WEST_SHARE = 0.70      # try 0.30 and see whether the green split changes
TRAIN_SEED, TEST_SEED = 42, 314


def write_xml(root, path):
    ET.ElementTree(root).write(path, encoding="utf-8", xml_declaration=True)


def build_intersection(folder):
    """Compile one fixed network and identify which direction each green serves."""
    nodes = ET.Element("nodes")
    ET.SubElement(nodes, "node", id="centre", x="0", y="0", type="traffic_light")
    edges = ET.Element("edges")
    for arm, (x, y) in {"N": (0, 220), "S": (0, -220),
                        "E": (220, 0), "W": (-220, 0)}.items():
        ET.SubElement(nodes, "node", id=arm, x=str(x), y=str(y))
        for name, start, end in [(arm + "_in", arm, "centre"), (arm + "_out", "centre", arm)]:
            ET.SubElement(edges, "edge", {"id": name, "from": start, "to": end,
                                          "numLanes": "1", "speed": "13.9"})
    write_xml(nodes, folder / "nodes.xml")
    write_xml(edges, folder / "roads.xml")
    subprocess.run(["netconvert", "-n", "nodes.xml", "-e", "roads.xml", "--no-turnarounds",
                    "-o", "network.net.xml"], cwd=folder, check=True, stdout=subprocess.DEVNULL)
    net = ET.parse(folder / "network.net.xml")
    template = net.find("./tlLogic[@id='centre']")
    # Read SUMO's actual link indices instead of assuming the first green is N/S.
    origins = {int(link.get("linkIndex")): link.get("from")
               for link in net.findall("./connection[@tl='centre']")}
    phase_indices = {}
    for index, phase in enumerate(template.findall("phase")):
        state = phase.get("state")
        active = {arm for i, arm in origins.items() if state[i] in "Gg"}
        if active == {"N_in", "S_in"}:
            phase_indices["NS"] = index
        elif active == {"E_in", "W_in"}:
            phase_indices["EW"] = index
        elif active:
            raise ValueError("Expected a two-direction signal plan for this network.")
    if len(phase_indices) != 2 or len(template.findall("phase")) != 4:
        raise ValueError("Expected two green phases and two yellow transitions.")
    return template, phase_indices


def make_traffic(folder, filename, seed):
    """Draw a fixed set of trips. Each candidate faces exactly the same drivers."""
    rng = random.Random(seed)
    # Clockwise arms: the opposite arm is straight ahead.
    arms = "NESW"
    trips = ET.Element("routes")
    departures = sorted(rng.uniform(0, ARRIVAL_WINDOW) for _ in range(CAR_COUNT))
    for i, depart in enumerate(departures):
        approach = rng.choice("EW" if rng.random() < EAST_WEST_SHARE else "NS")
        turn = rng.choices([1, 2, 3], weights=[15, 70, 15])[0]
        exit_arm = arms[(arms.index(approach) + turn) % 4]
        ET.SubElement(trips, "trip", {"id": f"car{i}", "depart": f"{depart:.2f}",
                                      "from": approach + "_in", "to": exit_arm + "_out"})
    write_xml(trips, folder / filename)


def write_signal(folder, signal, plan):
    """Decode the two genes. Copy SUMO's legal movements and yellow transitions."""
    template, indices = signal
    logic = deepcopy(template)
    logic.set("programID", "candidate")
    phases = logic.findall("phase")
    phases[indices["NS"]].set("duration", str(plan[0]))
    phases[indices["EW"]].set("duration", str(plan[1]))
    extra = ET.Element("additional")
    extra.append(logic)
    write_xml(extra, folder / "signal.add.xml")


def scenario_args(traffic, seed):
    return ["-n", "network.net.xml", "-r", traffic, "-a", "signal.add.xml",
            "--seed", str(seed), "--time-to-teleport", "-1"]


def score(folder, signal, plan, traffic, seed):
    """Minimize seconds from scheduled departure to arrival, averaged over all cars."""
    write_signal(folder, signal, plan)
    subprocess.run(["sumo", *scenario_args(traffic, seed), "--no-step-log",
                    "--end", str(ARRIVAL_WINDOW + 3600), "--duration-log.statistics",
                    "--statistic-output", "result.xml"], cwd=folder, check=True,
                   stdout=subprocess.DEVNULL)
    stats = ET.parse(folder / "result.xml")
    trips = stats.find("vehicleTripStatistics")
    if int(trips.get("count")) != CAR_COUNT or int(stats.find("teleports").get("total")):
        return float("inf")  # incomplete runs cannot win by omitting delayed cars
    seconds = float(trips.get("totalTravelTime")) + float(trips.get("totalDepartDelay"))
    return seconds / CAR_COUNT


def run_experiment(search, baseline, *, view=False):
    """Prepare SUMO, pass a one-argument fitness function to the EA, and test its winner."""
    programs = ["sumo", "netconvert"] + (["sumo-gui"] if view else [])
    for program in programs:
        if not shutil.which(program):
            raise SystemExit(f"Cannot find {program}. Add SUMO's bin folder to PATH and restart VS Code.")

    with TemporaryDirectory(prefix="signal_ea_") as workdir:
        folder = Path(workdir)
        signal = build_intersection(folder)
        make_traffic(folder, "train.rou.xml", TRAIN_SEED)
        make_traffic(folder, "test.rou.xml", TEST_SEED)

        @lru_cache(maxsize=None)
        def fitness(plan):
            return score(folder, signal, plan, "train.rou.xml", TRAIN_SEED)

        print(f"One individual = (N/S green, E/W green). Lower mean journey time wins.\n"
              f"{CAR_COUNT} trips per run; {EAST_WEST_SHARE:.0%} expected to enter from E/W.")
        winner = search(fitness)
        if fitness(winner) == float("inf"):
            raise SystemExit("No training plan cleared all scheduled trips. Try a lighter demand.")
        print(f"\n{fitness.cache_info().misses} distinct training plans simulated.")
        print("Mean journey time (seconds per car)")
        print("Plan               N/S   E/W      Training    Fresh traffic")
        for label, plan in [("Equal-time", baseline), ("EA winner", winner)]:
            test_score = score(folder, signal, plan, "test.rou.xml", TEST_SEED)
            print(f"{label:16s} {plan[0]:3d}   {plan[1]:3d}    {fitness(plan):9.2f}    {test_score:13.2f}")
        if view:
            write_signal(folder, signal, winner)
            subprocess.run(["sumo-gui", *scenario_args("test.rou.xml", TEST_SEED),
                            "--start", "--delay", "30"], cwd=folder, check=True)
