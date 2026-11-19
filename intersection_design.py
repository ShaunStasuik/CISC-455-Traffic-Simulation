"""CISC 455: what one intersection design is, which designs are legal, and what it costs.

A Design is one individual in the EA. Layout genes (control type, lanes, turning
lanes, pockets) change the road and cost money; timer genes (greens, yellow) are
just software and are free. value_score blends traffic delay and cost into the
single number the EA minimizes, steered by VALUE_WEIGHT.
"""
import math
from typing import NamedTuple


ARMS = "NESW"               # clockwise; right_pockets follows this order
SPEED_LIMIT = 13.9          # m/s (50 km/h) on every approach

# ---- Gene ranges ------------------------------------------------------------
CONTROL_TYPES = ("stop", "signal", "roundabout")
MIN_LANES, MAX_LANES = 1, 3             # through lanes per direction, per axis
MAX_TURN_LANE_M, TURN_LANE_STEP = 100, 10
POCKET_LENGTH = 40                      # metres; pockets are on or off, not sized
MIN_GREEN, MAX_GREEN = 10, 60
MIN_ARROW, MAX_ARROW = 5, 20            # protected left-turn green
MAX_YELLOW = 6

# Minimum safe yellow (ITE kinematic rule): reaction time plus time to stop
# comfortably from the speed limit. 1.0 + 13.9 / (2 * 3.0) = 3.3 s, rounded up to 4 s.
REACTION_TIME = 1.0         # s
COMFORT_DECEL = 3.0         # m/s^2
MIN_SAFE_YELLOW = math.ceil(REACTION_TIME + SPEED_LIMIT / (2 * COMFORT_DECEL))

# ---- Prices (CAD) -----------------------------------------------------------
# Placeholder figures with realistic proportions; tune them for your report.
TURN_LANE_BASE = 50_000         # per approach, charged once a turning lane exists
TURN_LANE_PER_M = 800           # per metre of turning lane, per approach
TURN_ARROW_COST = 40_000        # one-time: left-turn arrow heads and controller work
LANE_COST = 120_000             # first lane on a road (both directions counted)
LANE_GROWTH = 1.25              # each extra lane costs 25% more than the last (land)
RIGHT_POCKET_COST = 60_000      # flat, per pocket switched on
STOP_SIGN_COST = 10_000
SIGNAL_COST = 250_000
SIGNAL_UPKEEP_PER_YEAR = 8_000
SERVICE_YEARS = 20              # upkeep is counted over this many years
ROUNDABOUT_COST = 1_500_000


class Design(NamedTuple):
    control: str = "signal"
    ns_lanes: int = 1
    ew_lanes: int = 1
    turn_lane_m: int = 0                            # left-turn lane length; 0 = none
    turn_arrow: bool = False                        # protected left-turn phase
    right_pockets: tuple = (False, False, False, False)   # N, E, S, W approaches
    ns_green: int = 30
    ew_green: int = 30
    arrow_green: int = 10
    yellow: int = MIN_SAFE_YELLOW


def clamp(value, low, high):
    return max(low, min(high, value))


def repair(design):
    """Return the nearest legal design. Every new individual passes through here."""
    d = design._replace(
        ns_lanes=clamp(design.ns_lanes, MIN_LANES, MAX_LANES),
        ew_lanes=clamp(design.ew_lanes, MIN_LANES, MAX_LANES),
        turn_lane_m=clamp(round(design.turn_lane_m / TURN_LANE_STEP) * TURN_LANE_STEP,
                          0, MAX_TURN_LANE_M),
        ns_green=clamp(design.ns_green, MIN_GREEN, MAX_GREEN),
        ew_green=clamp(design.ew_green, MIN_GREEN, MAX_GREEN),
        arrow_green=clamp(design.arrow_green, MIN_ARROW, MAX_ARROW),
        # Yellow may never be shorter than the safe minimum for the speed limit.
        yellow=clamp(design.yellow, MIN_SAFE_YELLOW, MAX_YELLOW),
    )
    if d.control != "signal":
        d = d._replace(turn_arrow=False)        # an arrow needs a signal to live on
    if d.control == "roundabout":
        # Traffic circulates; there is no left-turn bay or right-turn pocket to build.
        d = d._replace(turn_lane_m=0, right_pockets=(False,) * len(ARMS))
    return d


def lane_cost(lanes_on_road):
    """Lane k on one road costs LANE_COST * LANE_GROWTH**(k-1): wider roads need pricier land."""
    return sum(LANE_COST * LANE_GROWTH ** k for k in range(lanes_on_road))


def cost_breakdown(design):
    """Price every feature of a design. Timers are software, so they cost nothing."""
    d = design
    approaches = len(ARMS)
    control = {
        "stop": STOP_SIGN_COST,
        "signal": SIGNAL_COST + SIGNAL_UPKEEP_PER_YEAR * SERVICE_YEARS,
        "roundabout": ROUNDABOUT_COST,
    }[d.control]
    return {
        "Control type": control,
        # Each arm's road carries its lanes in both directions.
        "Lanes": sum(lane_cost(2 * (d.ns_lanes if arm in "NS" else d.ew_lanes)) for arm in ARMS),
        "Turning lanes": 0 if d.turn_lane_m == 0 else
            approaches * (TURN_LANE_BASE + TURN_LANE_PER_M * d.turn_lane_m),
        "Turn arrow signal": TURN_ARROW_COST if d.turn_arrow else 0,
        "Right-turn pockets": RIGHT_POCKET_COST * sum(d.right_pockets),
        "Green/yellow timers": 0,
    }


def total_cost(design):
    return sum(cost_breakdown(design).values())


def value_score(delay, reference_delay, cost, budget, value_weight):
    """Blend the two goals into one number to minimize.

    traffic    = delay relative to the baseline design (1.0 = no better, 0.5 = half the delay)
    budget_gap = distance from the budget as a share of it, under or over
    value_weight 0 cares only about traffic, 1 only about cost, 0.5 splits evenly.
    Rewarding closeness (not cheapness) stops the EA from leaving money unspent
    when spending it would noticeably improve traffic.
    """
    if math.isinf(delay):
        return math.inf             # gridlocked designs can never win
    traffic = delay / reference_delay
    budget_gap = abs(cost - budget) / budget
    return (1 - value_weight) * traffic + value_weight * budget_gap


def describe(design):
    d = design
    parts = [d.control, f"lanes N/S {d.ns_lanes} E/W {d.ew_lanes}"]
    if d.turn_lane_m:
        parts.append(f"left lanes {d.turn_lane_m} m")
    if any(d.right_pockets):
        parts.append("pockets " + "".join(a for a, on in zip(ARMS, d.right_pockets) if on))
    if d.control == "signal":
        timing = f"green {d.ns_green}/{d.ew_green}s, yellow {d.yellow}s"
        if d.turn_arrow:
            timing += f", arrow {d.arrow_green}s"
        parts.append(timing)
    return " | ".join(parts)
