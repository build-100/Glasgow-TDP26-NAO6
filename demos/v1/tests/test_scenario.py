import math
from pathlib import Path
import sys
import unittest
import xml.etree.ElementTree as ET
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from scenario import Scenario, IDS, edges, centre, region, KICK_RANGE

class IntegrationContracts(unittest.TestCase):
    def test_graph_and_regions(self):
        self.assertEqual(len(edges()),20)
        self.assertEqual([region(*centre(n)) for n in range(9)],list(range(9)))
        self.assertEqual(region(4.6,0),-1)
    def test_settle_then_approach_and_fixed_nominal_kick(self):
        scene=Scenario();rolls=[p for p in scene.phases if p['label']=='BALL_ROLLING']
        self.assertEqual(len(rolls),4)
        for p in rolls:
            self.assertEqual(p['before'],p['after'])
            self.assertEqual(p['ball_before'][1],p['ball_after'][1])
        for p in rolls[:-1]:self.assertAlmostEqual(p['ball_after'][0]-p['ball_before'][0],KICK_RANGE)
        for p in scene.phases:
            if p['label']=='APPROACH':self.assertEqual(p['ball_before'],p['ball_after'])
        self.assertEqual(scene.sample(scene.duration)['score'],[1,0])
    def test_no_contact_between_robot_centres(self):
        scene=Scenario()
        for i in range(int(scene.duration*20)+1):
            robots=scene.sample(i/20)['robots']
            for a in range(8):
                for b in range(a+1,8):
                    p,q=robots[IDS[a]],robots[IDS[b]]
                    self.assertGreater(math.hypot(p[0]-q[0],p[1]-q[1]),.4)
    def test_model_namespaces_and_preserved_physics(self):
        root=Path(__file__).resolve().parents[1]
        source=ET.parse(root/'models/nao6_collision_visual.urdf').getroot()
        for rid in IDS:
            robot=ET.parse(root/'urdf'/(rid+'.urdf')).getroot()
            for old,new in zip(source.findall('link'),robot.findall('link')):
                self.assertEqual(new.get('name'),rid+'/'+old.get('name'))
                for tag in ['inertial','collision']:
                    self.assertEqual([ET.tostring(e) for e in old.findall(tag)],[ET.tostring(e) for e in new.findall(tag)])
            for old,new in zip(source.findall('joint'),robot.findall('joint')):
                self.assertEqual(old.get('type'),new.get('type'))
                self.assertEqual(new.find('parent').get('link'),rid+'/'+old.find('parent').get('link'))
            self.assertIsNotNone(robot.find("link[@name='%s/CameraTop']"%rid))
            self.assertIsNotNone(robot.find("link[@name='%s/CameraBottom']"%rid))

if __name__=='__main__':unittest.main()
