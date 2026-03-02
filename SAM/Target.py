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
        Target_VZscatter = -5

        self.Target_state = np.array([Target_Xscatter, Target_Yscatter, Target_Zscatter], dtype=float)
        self.Target_Vstate = np.array([Target_VXscatter, Target_VYscatter, Target_VZscatter], dtype=float)

        (self.Target,) = ax.plot([self.Target_state[0]], [self.Target_state[1]], [self.Target_state[2]], marker='o', color = 'blue')
        (self.Tpath,) = ax.plot(self.Target_Path[0], self.Target_Path[1], self.Target_Path[2], color = 'blue')
    
    def targetGuidance(self, dt):
        enforce = False
        if self.Target_state[0] < 50:
            self.Target_Vstate[0] += 0.5 * (50 - self.Target_state[0])/50
            self.Target_Vstate[1] = self.Target_Vstate[1] * 1.01
            enforce = True
        elif self.Target_state[0] > 450:
            self.Target_Vstate[0] += -0.5 * (self.Target_state[0] - 450)/50
            self.Target_Vstate[1] = self.Target_Vstate[1] * 1.01
            enforce = True
        if self.Target_state[2] < 20:
            self.Target_Vstate[2] += 0.5 * (20 - self.Target_state[2])/20
            self.Target_Vstate[1] = self.Target_Vstate[1] * 1.01
            enforce = True
        elif self.Target_state[2] > 300:
            self.Target_Vstate[2] += -0.5 * (self.Target_state[2] - 300)/50
            self.Target_Vstate[1] = self.Target_Vstate[1] * 1.01
            enforce = True

        if enforce == False:
            axisChanging = np.random.randint(0, 3)
            Delta = np.random.default_rng().uniform(low=-1, high=1)

            self.Target_Vstate[axisChanging] += Delta

        v_norm = np.linalg.norm(self.Target_Vstate)
        v_max = 35
        if v_norm > v_max and v_norm > 0:
            self.Target_Vstate *= v_max / v_norm

        self.Target_state += self.Target_Vstate * dt

        self.Target_Path[0].append(float(self.Target_state[0]))
        self.Target_Path[1].append(float(self.Target_state[1]))
        self.Target_Path[2].append(float(self.Target_state[2]))
