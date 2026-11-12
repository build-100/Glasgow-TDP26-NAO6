"""Integration invariants of the published fixed match export."""
import math
from pathlib import Path
import sys
import unittest
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from scenario import Scenario,IDS
from demo_geometry import adjacent

class ExportChecks(unittest.TestCase):
    def setUp(self):self.scene=Scenario()
    def test_complete_match_keepers_and_neighbour_moves(self):
        scene=self.scene
        self.assertEqual(max(scene.score),3)
        self.assertEqual(scene.phases[-1]['label'],'MATCH_COMPLETE_FIRST_TO_THREE')
        for rid in ['red4','blue4']:
            self.assertEqual(len({tuple(p[side][rid]) for p in scene.phases for side in ['before','after']}),1)
        rounds=set()
        for c in scene.contest_events:
            self.assertNotIn(c['round'],rounds);rounds.add(c['round'])
            self.assertLessEqual(c['owner_ball_distance'],.500001)
            self.assertLessEqual(c['challenger_ball_distance'],.500001)
            self.assertLessEqual(c['robot_distance'],.900001)
            self.assertEqual(c['turnover'],c['draw']<.4)
        for ep in scene.plan['episodes']:
            for tr in ep['transitions']:
                for move in tr['execution']:
                    self.assertTrue(adjacent(move['from_node'],move['to_node']))
    def test_continuous_execution_and_settled_ball_contract(self):
        scene=self.scene
        for a,b in zip(scene.phases,scene.phases[1:]):
            if b['setup_reset']:continue
            for rid in IDS:self.assertLess(math.dist(a['after'][rid][:2],b['before'][rid][:2]),1e-8)
            self.assertLess(math.dist(a['ball_after'],b['ball_before']),1e-8)
        for p in scene.phases:
            if p['label']=='BALL_ROLLING_ALL_HOLD':self.assertEqual(p['before'],p['after'])
            middle=scene.sample(p['start']+.5*p['duration'])
            self.assertEqual(len(middle['robots']),8)
            self.assertLessEqual(max(middle['score']),3)
        self.assertEqual(scene.sample(scene.duration)['score'],scene.score)
if __name__=='__main__':unittest.main()
