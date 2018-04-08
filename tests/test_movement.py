import unittest
import numpy as np
import math
import sys
import os
from os import path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from ScaraPathCalculator import ScaraPathCalculator

class TestMovement(unittest.TestCase):
    def test_movement(self):
        pos1 = [250, 50]
        pos2 = [-5, 200]
        speed = 20 #mm/s
        path_calculator = ScaraPathCalculator(arm_len1=179.9,
                                            arm_len2=150,
                                            angle_res1=2*math.pi/200/3.70588235/8,
                                            angle_res2=2*math.pi/400/2/8)
    
        coef1, coef2, numsteps, direction = path_calculator.line_compute_times(pos1, pos2, speed)
        
        
        expected_coef1 = [2.31893691e-09, -4.59133726e-06, 1.14843710e-02, -1.70038203e-01]
        expected_coef2 = [7.41982043e-10, 1.94931267e-05, -1.89896965e-03, 7.41014063e-01]
        expected_numsteps = [1498.0, 933.0]
        expected_direction = [[True, None], [True, 643]]
        
        # take care of rounding
        precision = 6
        expected_coef1 = np.round(np.array(expected_coef1), precision).tolist()
        expected_coef2 = np.round(np.array(expected_coef2), precision).tolist()
        
        self.assertListEqual(np.round(coef1, precision).tolist(), expected_coef1)
        self.assertListEqual(np.round(coef2, precision).tolist(), expected_coef2)
        self.assertListEqual(numsteps, expected_numsteps)
        self.assertListEqual(direction, expected_direction)

if __name__ == '__main__':
    unittest.main()