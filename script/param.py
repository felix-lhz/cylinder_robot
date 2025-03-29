import math
import numpy as np
def constrain(val, min_val, max_val):
    return min(max_val, max(min_val, val))

def sign(val):
    if val > 0:
        return 1
    elif val < 0:
        return -1
    else:
        return 0

m = 10
g = 9.81
dt = 0.01

# 0.01s
K_1 = 1.0
K_2= 5.0
K_lqr = np.array([[K_1, 0.0000, K_2, 0.0000],
                  [0.0000, K_1, -0.0000, K_2]])

PID_IMPROVE_NONE = 0b00000000                # 0000 0000
PID_Integral_Limit = 0b00000001              # 0000 0001
PID_Derivative_On_Measurement = 0b00000010   # 0000 0010
PID_Trapezoid_Intergral = 0b00000100         # 0000 0100
PID_Proportional_On_Measurement = 0b00001000 # 0000 1000
PID_OutputFilter = 0b00010000                # 0001 0000
PID_ChangingIntegrationRate = 0b00100000     # 0010 0000
PID_DerivativeFilter = 0b01000000            # 0100 0000
PID_ErrorHandle = 0b10000000                 # 1000 0000

