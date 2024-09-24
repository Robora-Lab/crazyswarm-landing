
import numpy as np
import matplotlib.pyplot as plt
from scipy import signal


A = np.array([[0, 0, 0, 1, 0, 0],
                [0, 0, 0, 0, 1, 0],
                [0, 0, 0, 0, 0, 1],
                [0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0]])


B = np.array([[0, 0, 0],
                [0, 0, 0],
                [0, 0, 0],
                [1, 0, 0],
                [0, 1, 0],
                [0, 0, 1]])

# Sampling time
dt = 0.01  # Example sampling time

# Convert to discrete-time system
Anp, Bnp, Cd, Dd, dt = signal.cont2discrete((A, B, np.zeros((6,6)), np.zeros((6, 3))), dt, method='zoh')
# print("Ad = \n", Ad)
# print("Bd = \n", Bd)

print("A = \n", Anp)
print("B = \n", Bnp)