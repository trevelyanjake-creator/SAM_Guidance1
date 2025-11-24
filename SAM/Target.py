import numpy as np

class Target:
    def __init__(self, ax):
        self.contact = False
        self.Pursuer = None

        Xp = []
        Yp = []
        Zp = []

        self.Target_Path = [Xp, Yp, Zp]

        Target_Xscatter = np.random.default_rng().uniform(low = 50, high = 450)
        Target_Yscatter = 0
        Target_Zscatter = np.random.default_rng().uniform(low = 150, high = 200)

        Xp.append(Target_Xscatter)
        Yp.append(Target_Yscatter)
        Zp.append(Target_Zscatter)

        Target_VXscatter = 0
        Target_VYscatter = 30
        Target_VZscatter = 0

        self.Target_state = np.array([Target_Xscatter, Target_Yscatter, Target_Zscatter], dtype=float)
        self.Target_Vstate = np.array([Target_VXscatter, Target_VYscatter, Target_VZscatter], dtype=float)

        (self.Target,) = ax.plot([self.Target_state[0]], [self.Target_state[1]], [self.Target_state[2]], marker='o', color = 'blue')
        (self.Tpath,) = ax.plot(self.Target_Path[0], self.Target_Path[1], self.Target_Path[2], color = 'blue')