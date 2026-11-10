"""Public nine-region geometry; no policy or reward implementation."""
import numpy as np
IDS=['red1','red2','red3','blue1','blue2','blue3']
ALL_IDS=['red1','red2','red3','red4','blue1','blue2','blue3','blue4']

def region(x, y):
    if not (-4.5 <= x < 4.5 and -3.0 <= y < 3.0):
        return -1
    return int((y + 3.0) // 2.0) * 3 + int((x + 4.5) // 3.0)

def centre(n):
    return np.array([-3.0 + 3.0 * (n % 3), -2.0 + 2.0 * (n // 3)])

def adjacent(a, b):
    return a >= 0 and b >= 0 and abs(a // 3 - b // 3) <= 1 and abs(a % 3 - b % 3) <= 1

def edges():
    return [(a, b) for a in range(9) for b in range(a + 1, 9) if adjacent(a, b)]
