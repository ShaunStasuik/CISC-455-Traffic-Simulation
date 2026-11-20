"""Small deterministic correctness tests; no SUMO installation required."""
import contextlib
import io
import json
import math
import os
from pathlib import Path
import random
import tempfile
import unittest
from unittest.mock import patch

import intersection_ea as ea
import sumo_simulation as sim


def record(time, cost, violation=0, plan=None):
    return dict(journey_time=time, cost=cost, violation=violation,
                plan=ea.minimum_design() if plan is None else dict(plan))


class MultiObjectiveTests(unittest.TestCase):
    def test_known_fronts_and_equal_objectives(self):
        pop = [record(280, 100), record(245, 160), record(238, 300),
               record(260, 200), record(300, 400), record(245, 160)]
        self.assertEqual([set(f) for f in ea.non_dominated_sort(pop)],
                         [{0, 1, 2, 5}, {3}, {4}])
        self.assertFalse(ea.dominates(pop[1], pop[5]))

    def test_constraint_priority(self):
        feasible = record(900, 200)
        over = record(math.inf, 201, 1)
        invalid = record(math.inf, 0, math.inf)
        self.assertTrue(ea.dominates(feasible, over))
        self.assertTrue(ea.dominates(feasible, invalid))
        self.assertTrue(ea.dominates(over, invalid))
        self.assertEqual(ea.update_archive([], [over, invalid]), [])

    def test_crowding_preserves_extremes_and_normalizes(self):
        pop = [record(10, 400), record(20, 300), record(30, 200), record(40, 100)]
        distances = ea.crowding_distance(pop, list(range(4)))
        self.assertTrue(math.isinf(distances[0]))
        self.assertTrue(math.isinf(distances[3]))
        self.assertAlmostEqual(distances[1], 4 / 3)
        scaled = [record(p['journey_time'], p['cost'] * 10000) for p in pop]
        self.assertEqual(distances, ea.crowding_distance(scaled, list(range(4))))
        survivors = ea.select_survivors(pop, 2, random.Random(1))
        self.assertEqual({p['journey_time'] for p in survivors}, {10, 40})
        equal = [record(5, 0), record(5, 0)]
        self.assertEqual(ea.crowding_distance(equal, [0, 1]), {0: 0, 1: 0})

    def test_archive_removes_dominated_and_duplicate_objectives(self):
        archive = ea.update_archive([], [record(20, 50), record(10, 100)])
        archive = ea.update_archive(archive, [record(15, 40), record(15, 40), record(9, 120)])
        self.assertEqual([(p['journey_time'], p['cost']) for p in archive], [(15, 40), (10, 100), (9, 120)])

    def test_weight_endpoints_budget_and_stable_references(self):
        front = [record(280, 100), record(245, 160), record(238, 300)]
        self.assertEqual(ea.recommend(front, cost_weight=0)['cost'], 300)
        self.assertEqual(ea.recommend(front, cost_weight=1)['cost'], 100)
        self.assertEqual(ea.recommend(front, cost_weight=0, budget=160)['cost'], 160)
        self.assertEqual(ea.recommend(front, cost_weight=.5, time_reference=300, cost_reference=1000)['cost'], 160)
        with self.assertRaises(ValueError):
            ea.recommend(front, budget=99)
        for options in ({'cost_weight': math.nan}, {'budget': -1}, {'cost_reference': 0}, {'time_reference': math.inf}):
            with self.assertRaises(ValueError):
                ea.recommend(front, **options)

    def test_pocket_example_and_dormant_arrows(self):
        p = ea.minimum_design()
        self.assertEqual(sim.cost_breakdown(p)['total'], 0)
        p.update(pocket_ns=70, left_green_ns=8)
        b = sim.cost_breakdown(p)
        self.assertAlmostEqual(b['areas_m2']['pockets'], 448)
        self.assertEqual(b['protected_left_approaches'], 2)
        self.assertEqual(b['total'], 108400)
        p['left_green_ns'] = 20
        self.assertEqual(sim.cost_breakdown(p)['total'], 108400)
        p['pocket_ns'] = 35
        self.assertEqual(sim.cost_breakdown(p)['total'], 69200)
        p['pocket_ns'] = 0
        self.assertEqual(sim.cost_breakdown(p)['total'], 0)

    def test_full_widening_and_pocket_storage_extremes(self):
        p = ea.minimum_design()
        p['lanes_ew'] = 2
        self.assertEqual(sim.cost_breakdown(p)['total'], 3360000)
        for length in (12, 50, 100, 120, 155, 200, 1000):
            p.update(pocket_ns=length, pocket_ew=length)
            b = sim.cost_breakdown(p)
            self.assertAlmostEqual(b['areas_m2']['pockets'], 4 * 3.2 * length)
            self.assertAlmostEqual(b['areas_m2']['extra_lanes'], 4 * 1500 * 3.2)

    def test_bypass_shape_length_and_unit_prices(self):
        p = ea.minimum_design()
        p['bypass_n'] = True
        graph = sim.build_roads(p)
        points = graph['roads']['N_bypass']['shape']
        expected = sum(math.dist(a, b) for a, b in zip(points, points[1:])) * 3.2
        b = sim.cost_breakdown(p)
        self.assertAlmostEqual(b['areas_m2']['bypasses'], expected)
        self.assertEqual(b['total'], round(expected * 175, 2))
        p['pocket_ns'] = 120  # relocates gate, must change bypass construction length
        self.assertGreater(sim.cost_breakdown(p)['areas_m2']['bypasses'], expected)
        self.assertEqual(sim.cost_breakdown(p, 0, 0)['total'], 0)

    def test_budget_screening_cache_and_failed_simulation(self):
        with tempfile.TemporaryDirectory() as directory:
            evaluate, _, cache, counters = sim.make_evaluator(Path(directory), Path('traffic'), budget=0)
            with patch.object(sim, 'run_simulation') as run:
                bad_budget = evaluate(ea.BASELINE)
                self.assertGreater(bad_budget['violation'], 0)
                run.assert_not_called()
                run.return_value = dict(score=250.0, arrived=600, collisions=0, teleports=0, reason='')
                p = ea.minimum_design()
                self.assertEqual(evaluate(p)['violation'], 0)
                evaluate(dict(p))
                self.assertEqual(run.call_count, 1)
                p['green_ns'] = 31
                run.return_value = dict(score=math.inf, arrived=599, collisions=1, teleports=0, reason='collision')
                self.assertEqual(evaluate(p)['violation'], math.inf)
                self.assertEqual(counters, dict(sumo_evaluations=2, over_budget_skipped=1, simulation_failures=1))
                self.assertEqual(len(cache), 3)

    def test_evolution_budget_zero_and_non_dominated_archive(self):
        def evaluate(plan):
            cost = sim.cost_breakdown(plan)['total']
            t = 200 + abs(plan['green_ns'] - 25) + abs(plan['green_ew'] - 40)
            return record(t if cost == 0 else math.inf, cost, cost, plan)
        with contextlib.redirect_stdout(io.StringIO()):
            outcome = ea.evolve(evaluate, 6, 4, within_budget=lambda p: sim.cost_breakdown(p)['total'] == 0)
        self.assertTrue(outcome['front'])
        self.assertTrue(all(p['cost'] == 0 for p in outcome['front']))
        self.assertEqual(len(outcome['history']), 5)
        for a in outcome['front']:
            for b in outcome['front']:
                self.assertFalse(ea.dominates(a, b))

    def test_json_contains_no_infinity(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'test.json'
            sim.write_json(path, {'items': [record(math.inf, 0, math.inf)]})
            self.assertIsNone(json.loads(path.read_text())['items'][0]['journey_time'])

    def test_no_feasible_run_saves_diagnostics_and_clears_stale_pointer(self):
        original = Path.cwd()
        with tempfile.TemporaryDirectory() as directory:
            try:
                os.chdir(directory)
                Path('results').mkdir()
                Path('results/best_plan.json').write_text('{"old": true}')
                failure = dict(score=math.inf, arrived=599, collisions=1, teleports=0, reason='collision')
                def search(evaluate, affordable):
                    item = evaluate(ea.minimum_design())
                    return dict(front=ea.update_archive([], [item]), history=[])
                with patch.object(sim, 'executable', return_value='unused'), \
                     patch.object(sim, 'run_simulation', return_value=failure), \
                     contextlib.redirect_stdout(io.StringIO()):
                    with self.assertRaisesRegex(RuntimeError, 'No feasible design'):
                        sim.run_experiment(search, ea.BASELINE, recommend=ea.recommend,
                                           preferences=dict(budget=None, cost_weight=.5,
                                                            time_reference=300, cost_reference=1000000))
                self.assertFalse(Path('results/best_plan.json').exists())
                self.assertEqual(json.loads(Path('results/pareto_front.json').read_text())['front'], [])
                self.assertTrue(list(Path('results').glob('experiment_*/evaluations.json')))
            finally:
                os.chdir(original)


if __name__ == '__main__':
    unittest.main()
