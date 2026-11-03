#? I hardly knower her!

import DP2_SAMPlotter

NumMissiles = 8
NumMissilesLinear = 0
NumTargets = 8
N = 17
Debug = False
Evade = True
EvasionMethod = "Stall" # Options are Stall and Spiral (Spiral is not compatible with Horizontal)
TargetPhase = "Vertical" # Options are Vertical and Horizontal
_2D = False
Plot_LOS_rate = False
Plot_Obs_Convergence = False
Plot = True

MonteComp = False
MonteNumSims = 500
MonteNumN = 1
MonteNLbound = 17
MonteNHbound = 17

if not MonteComp:  
    DP2_SAMPlotter.main(NumMissiles, NumMissilesLinear, NumTargets, Debug, Evade, EvasionMethod, TargetPhase, _2D, Plot_LOS_rate, Plot, N, Plot_Obs_Convergence)
else:
    Plot = False
    Plot_LOS_rate = False
    Plot_Obs_Convergence = False
    _2D = False
    Debug = False

    for n in range(MonteNumN):
        NumIntercepted = 0
        N = MonteNLbound + (MonteNHbound - MonteNLbound) / MonteNumN * n
        for _ in range(MonteNumSims):
            NumIntercepted += DP2_SAMPlotter.main(NumMissiles, NumMissilesLinear, NumTargets, Debug, Evade, EvasionMethod, TargetPhase, _2D, Plot_LOS_rate, Plot, N, Plot_Obs_Convergence)

        avgIntercepted = NumIntercepted / MonteNumSims
        percIntercepted = avgIntercepted / NumTargets * 100

        print("Spiral Linear")
        print(f"For N = {N:.2f}, Average number of targets intercepted = {avgIntercepted:.2f}")
        print(f"For N = {N:.2f}, Percentage of targets intercepted = {percIntercepted:.2f}")
