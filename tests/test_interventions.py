from __future__ import annotations
import unittest
from spatialmind.metric_sim import demo_room_map
from spatialmind.physical_benchmark import (
    SCENARIOS,POLICIES,paired_bootstrap_deltas,Trial,
)


class InterventionTests(unittest.TestCase):
    def test_gazebo_intervention_request_whitelist(self):
        from scripts.gazebo_intervene import build_request
        room=demo_room_map()
        command=build_request("blue_toolbox","office",room)
        self.assertIn("blue_toolbox",command)
        self.assertIn("position",command)
        with self.assertRaises(ValueError):
            build_request("robot_base","office",room)

    def test_paired_baseline_negative_results_are_preserved(self):
        samples=[]
        for seed in range(20):
            samples.append(Trial("test","full",seed,0,True,True,False,
                                 15,2,0,1,1.0,"sim:a"))
            samples.append(Trial("test","no_memory",seed,0,True,True,False,
                                 10,2,0,1,1.0,"sim:b"))
        delta=paired_bootstrap_deltas(samples,baseline="no_memory")
        self.assertEqual(delta["n_pairs"],20)
        self.assertEqual(delta["distance_saved_m_positive_is_better"],-5.0)
        self.assertEqual(delta["distance_delta_95ci"],[-5.0,-5.0])

if __name__=="__main__":
    unittest.main()
