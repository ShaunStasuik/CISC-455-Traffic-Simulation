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
    return dict(delay=time, cost=cost, violation=violation,
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
        scaled = [record(p['delay'], p['cost'] * 10000) for p in pop]
        self.assertEqual(distances, ea.crowding_distance(scaled, list(range(4))))
        survivors = ea.select_survivors(pop, 2, random.Random(1))
        self.assertEqual({p['delay'] for p in survivors}, {10, 40})
        equal = [record(5, 0), record(5, 0)]
        self.assertEqual(ea.crowding_distance(equal, [0, 1]), {0: 0, 1: 0})

    def test_archive_removes_dominated_and_duplicate_objectives(self):
        archive = ea.update_archive([], [record(20, 50), record(10, 100)])
        archive = ea.update_archive(archive, [record(15, 40), record(15, 40), record(9, 120)])
        self.assertEqual([(p['delay'], p['cost']) for p in archive], [(15, 40), (10, 100), (9, 120)])

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
        # 4 directed roads x 400 m x 3.2 m x 175 CAD/m2
        self.assertEqual(sim.cost_breakdown(p)['total'], 4 * sim.ARM_LENGTH * 3.2 * 175)
        self.assertEqual(sim.cost_breakdown(p)['total'], 896000)
        for length in (15, 50, 100, 120, 150):
            p.update(pocket_ns=length, pocket_ew=length)
            b = sim.cost_breakdown(p)
            self.assertAlmostEqual(b['areas_m2']['pockets'], 4 * 3.2 * length)
            self.assertAlmostEqual(b['areas_m2']['extra_lanes'], 4 * sim.ARM_LENGTH * 3.2)
        p.update(pocket_ns=1000)  # longer than the road itself: refused, not silently drawn
        with self.assertRaises(ValueError):
            sim.cost_breakdown(p)

    def test_creep_mutation_stays_legal_and_small(self):
        rng = random.Random(3)
        plan = dict(ea.BASELINE)
        for _ in range(2000):
            child = ea.mutate(plan, rng)
            ea.validate(child)
            for gene in ('green_ns', 'green_ew'):
                self.assertLessEqual(abs(child[gene] - plan[gene]), ea.CREEP_STEPS['green'])
            for gene in ('pocket_ns', 'pocket_ew'):
                if child[gene] and plan[gene]:
                    self.assertLessEqual(abs(child[gene] - plan[gene]), 5 * ea.CREEP_STEPS['pocket'])
            plan = child
        self.assertEqual(ea.BASELINE['green_ns'], 30)  # parent dictionaries are never edited

    def test_off_feature_switches_on_and_on_feature_can_switch_off(self):
        rng = random.Random(0)
        with patch.object(ea, 'MUTATION_PROBABILITY', 1.0):
            off = ea.minimum_design()
            self.assertTrue(all(ea.mutate(off, rng)['pocket_ns'] > 0 for _ in range(50)))
            on = dict(ea.BASELINE)
            results = {ea.mutate(on, rng)['pocket_ns'] == 0 for _ in range(200)}
            self.assertEqual(results, {True, False})

    def test_yellow_is_a_safety_rule_not_a_gene(self):
        self.assertNotIn('yellow_ns', ea.GENE_VALUES)
        with self.assertRaises(ValueError):
            ea.validate({**ea.BASELINE, 'yellow_ns': 4})
        settings = sim.build_roads(ea.BASELINE)['signals']['centre']
        # 1 s reaction + (50 km/h = 13.89 m/s) / (2 x 3 m/s2) = 3.31 s
        self.assertAlmostEqual(settings['ns']['yellow'], round(1 + 50 / 3.6 / 6, 3))
        self.assertEqual(settings['ns']['yellow'], settings['ew']['yellow'])

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

    def test_budget_screening_averaging_cache_and_failed_simulation(self):
        ok = lambda delay: dict(score=delay, arrived=600, collisions=0, teleports=0, reason='')
        failure = dict(score=math.inf, arrived=599, collisions=1, teleports=0, reason='collision')
        with tempfile.TemporaryDirectory() as directory:
            runs = [(Path('traffic_a'), 1), (Path('traffic_b'), 2)]
            evaluate, _, cache, counters = sim.make_evaluator(Path(directory), runs, budget=0)
            with patch.object(sim, 'run_simulation') as run:
                bad_budget = evaluate(ea.BASELINE)
                self.assertGreater(bad_budget['violation'], 0)
                run.assert_not_called()
                run.side_effect = [ok(20.0), ok(30.0)]
                p = ea.minimum_design()
                item = evaluate(p)
                self.assertEqual(item['violation'], 0)
                self.assertEqual(item['delay'], 25.0)           # mean of the two traffic samples
                evaluate(dict(p))                                 # cached: no new simulations
                self.assertEqual(run.call_count, 2)
                p['green_ns'] = 31
                run.side_effect = [failure]                       # first sample fails: stop early
                self.assertEqual(evaluate(p)['violation'], math.inf)
                self.assertEqual(run.call_count, 3)
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
            self.assertIsNone(json.loads(path.read_text())['items'][0]['delay'])

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
