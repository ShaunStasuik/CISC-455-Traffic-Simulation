# Simple infrastructure cost model

## Formula

`cost = road_cost * (extra_lane_area + pocket_area + bypass_area) + arrow_cost * protected_left_approaches`

Defaults are **175 CAD/m2** and **15,000 CAD per protected-left approach**.
The result is an illustrative CAD estimate calibrated to historical benchmarks,
not a 2026 quote. Quantities use metres and square metres. Each component cost
is rounded to cents, and the total is their sum. Budget tests use this same total.

## Evidence and assumptions

1. City of Mississauga, *2022 Development Charges Transportation Background
   Study*, Table F road-improvement unit costs: a 5 m-wide centre turn lane added
   through road widening is listed at 0.865 million CAD/km. Conversion:
   `865000 / (1000 * 5) = 173 CAD/m2`. We round to 175. Applying this particular
   treatment's rate to general lanes, pockets and bypasses is a deliberate
   simplification, not a municipal universal unit rate.
   https://www.mississauga.ca/wp-content/uploads/2022/03/04100858/2022-Development-Charges-Transportation-Background-Study.pdf

2. City of Winnipeg, *Plessis Road Left Turn Signals*, Public Works Committee,
   November 29, 2024, report pp. 7-8: three approach-specific phase installations
   and associated works are estimated at 10,000, 20,000 and 15,000 CAD. We use
   15,000 per approach as a representative classroom assumption. This is not an
   Ontario price or the price of one individual signal head.
   https://dmis.winnipeg.ca/DownloadMeetingDocument/753038/15.%20Plessis%20Road%20Left%20Turn%20Signals.pdf

3. Toronto explains that left-turn arrows require extra signal equipment and
   detection devices. This supports separating equipment cost from road area.
   https://www.toronto.ca/services-payments/streets-parking-transportation/traffic-management/traffic-signals-street-signs/traffic-signals-in-toronto/traffic-signal-installation/

The road and signal benchmarks are from different years and places. They have
not been inflation-adjusted to a common price year. Keep that limitation in the
presentation. Do not label the estimate a municipal budget forecast. Both rates
are configurable inputs and can later be replaced with consistent local prices.

## Common cost reference

Every design is priced above one general lane per direction, basic traffic
signals, no left pockets, no bypasses, and no protected left-arrow equipment.
Common infrastructure is omitted because it does not distinguish candidates.
This reference is different from `BASELINE`, which has 50 m pockets and arrows
on both axes, costing 116,000 CAD in this model.

This is design comparison above a minimum reference, NOT a retrofit estimate
relative to an actual existing intersection. No demolition cost or salvage refund
is modelled. Supporting real retrofit projects would require a separate existing
layout and explicit alteration/removal costs. A budget of zero means zero added
infrastructure above the reference, not that an entire intersection is free.

## Quantities and the decoder

- General lanes: for every directed road segment, count general lanes beyond one
  and multiply by its width and straight shape length. Include incoming AND
  outgoing segments. Segments partition each arm, so adding pocket nodes does
  not multiply the general-lane cost. At width 3.2 m and arm length 1500 m,
  upgrading a single axis to two general lanes costs
  `4 * 1500 * 3.2 * 175 = 3,360,000 CAD`.
- Pockets: only the added inner lane is counted. Use the explicitly specified
  storage lengths, excluding the decoder's 30 m graphical junction allowance.
  Thus `pocket_area = 2 * 3.2 * (pocket_ns + pocket_ew)`.
- Bypasses: count the enabled curved road's actual polyline length times its
  lane width. Repositioning a gate changes this length and therefore its cost.
  No separate full bypass charge is added; that would double count its pavement.
- Protected-left equipment: two approaches per axis if BOTH the pocket and
  left-arrow duration are positive. Dormant arrow genes cost nothing. A positive
  duration changing from 8 to 20 seconds does not add equipment cost. Splitting
  conflicting left phases does not mean buying the equipment twice.

The area calculation uses the abstract road decoder, not a polygon union of an
engineering drawing. Central junction footprints, detailed tapers, merge works,
islands, land, utility relocation, drainage upgrades and lifecycle maintenance
are omitted. Width is a fixed model parameter, not another gene.

## Example and checks

Two 70 m pockets at width 3.2 m and two active left-arrow approaches:

`175 * (2 * 70 * 3.2) + 15000 * 2 = 108400 CAD`

Two 35 m pockets with the same equipment:

`175 * (2 * 35 * 3.2) + 15000 * 2 = 69200 CAD`

The cost difference is 39,200 CAD. Signals' timing changes alone are free under
this capital-cost model. Their benefit or harm is measured through SUMO.

Optional sensitivity experiments can use road rates 125/175/250 and arrow rates
10000/15000/20000. These are test scenarios, not claimed market confidence ranges.

## Budget and preference

The budget is a hard constraint on this cost. The preference weight is a separate
decision input applied after NSGA-II. Neither is evolved. Do not multiply physical
construction costs by a municipality's preference: change the preference score.
When comparing new price assumptions, rerun the search because the cost ordering,
budget feasibility and discovered Pareto front may change.
