import math
from pathlib import Path
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from scenario import Scenario, OneShotContest, IDS, FIELD_PLAYERS, KEEPERS, region, edges, KICK_RANGE, WALK_SPEED

class VersionTwoContracts(unittest.TestCase):
    def setUp(self):self.scene=Scenario()
    def test_required_story_and_macro_turnover(self):
        s=self.scene;events=s.contests.events
        self.assertEqual([e['turnover'] for e in events],[False,True])
        self.assertEqual([e['probability'] for e in events],[.4,.4])
        self.assertEqual(events[1]['owner_before'],'blue3');self.assertEqual(events[1]['owner_after'],'red2')
        self.assertEqual(s.sample(s.duration)['score'],[2,0])
        goals=[p for p in s.phases if p['label']=='GOAL_RED']
        self.assertEqual([p['round'] for p in goals],[1,2])
        i=next(i for i,p in enumerate(s.phases) if p['label']=='TURNOVER_ENDS_T')
        self.assertEqual(s.phases[i+1]['step'],s.phases[i]['step']+1)
        self.assertFalse(any(p['label']=='KICK_FORWARD' and p['step']==events[1]['step'] for p in s.phases))
    def test_once_only_and_probability_boundaries(self):
        robots={'red1':(-.28,0,0),'red2':(-3,2,0),'red3':(-3,-2,0),
                'blue1':(.65,.65,0),'blue2':(3,2,0),'blue3':(3,-2,0)}
        for p,expected in [(0,False),(1,True)]:
            contest=OneShotContest(p=p)
            self.assertIsNone(contest.evaluate(1,1,'red1',robots,(0,0),settled=False))
            event=contest.evaluate(1,1,'red1',robots,(0,0));self.assertEqual(event['turnover'],expected)
            for step in [1,2,3]:self.assertIsNone(contest.evaluate(1,step,event['owner_after'],robots,(0,0)))
            self.assertEqual(len(contest.events),1)
        robots['blue1']=(3,0,0)
        self.assertIsNone(OneShotContest(p=1).evaluate(1,1,'red1',robots,(0,0)))
    def test_all_six_move_each_round_and_keepers_hold(self):
        for number in [1,2]:
            for r in FIELD_PLAYERS:
                distance=sum(math.dist(p['before'][r][:2],p['after'][r][:2]) for p in self.scene.phases if p['round']==number)
                self.assertGreater(distance,.3,r)
        for p in self.scene.phases:
            for r,pose in KEEPERS.items():self.assertEqual(p['before'][r],pose);self.assertEqual(p['after'][r],pose)
    def test_neighbour_actions_settlement_and_kick_range(self):
        for p in self.scene.phases:
            for action in p['actions'].values():
                a,b=action['from_node'],action['to_node']
                self.assertTrue(a==b or tuple(sorted([a,b])) in edges())
            if p['label']=='BALL_ROLLING_ALL_HOLD':
                self.assertEqual(p['before'],p['after'])
                self.assertEqual(p['ball_before'][1],p['ball_after'][1])
                distance=math.dist(p['ball_before'],p['ball_after'])
                if abs(p['ball_after'][0])<4.5:self.assertAlmostEqual(distance,KICK_RANGE)
                else:self.assertLessEqual(distance,KICK_RANGE)
            else:self.assertEqual(p['ball_before'],p['ball_after'])
            for r in FIELD_PLAYERS:
                speed=math.dist(p['before'][r][:2],p['after'][r][:2])/p['duration']
                self.assertLessEqual(speed,WALK_SPEED+1e-9)
    def test_spacing_and_in_play_continuity(self):
        for i,p in enumerate(self.scene.phases):
            if i and not p['setup_reset']:
                self.assertEqual(p['before'],self.scene.phases[i-1]['after'])
                self.assertEqual(p['ball_before'],self.scene.phases[i-1]['ball_after'])
            for k in range(21):
                u=k/20
                poses={r:tuple(a[j]+u*(p['after'][r][j]-a[j]) for j in range(2)) for r,a in p['before'].items()}
                for a in range(8):
                    for b in range(a+1,8):
                        self.assertGreaterEqual(math.dist(poses[IDS[a]],poses[IDS[b]]),.4,(p['round'],p['step'],p['label'],IDS[a],IDS[b]))

if __name__=='__main__':unittest.main()
