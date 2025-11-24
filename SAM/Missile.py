import numpy as np

class Missile:
    def __init__(self, ax):
        self.target = None
        self.ax = ax
        self.contact = False
        self.activated = False
        self.LNP = np.array([None, None, None])
        self.isPrimaryPursuer = False
        self.TargetsInRange = []
        self.MissilesInRange = []

        self.Missile_Xscatter = np.random.default_rng().uniform(low = 100, high = 400)
        self.Missile_Yscatter = np.random.default_rng().uniform(low = 100, high = 850)
        self.Missile_Zscatter = 0

        self.Missile_state = np.array([self.Missile_Xscatter, self.Missile_Yscatter, self.Missile_Zscatter], dtype=float)

        (self.Missile,) = ax.plot([self.Missile_state[0]], [self.Missile_state[1]], [self.Missile_state[2]], marker='o', linestyle='', color = 'red')

        MXp = []
        MYp = []
        MZp = []

        MXp.append(self.Missile_Xscatter)
        MYp.append(self.Missile_Yscatter)
        MZp.append(self.Missile_Zscatter)

        self.Missile_Path = [MXp, MYp, MZp]

        (self.Mpath,) = ax.plot([self.Missile_Path[0]], [self.Missile_Path[1]], [self.Missile_Path[2]], color = 'red')


    def activate(self, ax, target_state):
        self.activated = True
        self.LNP = target_state

        Missile_VXscatter = 0
        Missile_VYscatter = 0
        Missile_VZscatter = 35

        self.Missile_Vstate = np.array([Missile_VXscatter, Missile_VYscatter, Missile_VZscatter], dtype=float)
    
    def catch(self, target):
        if self.target is not None and self.target.Pursuer == self:
            self.target.Pursuer = None
        self.target = target
        if target.Pursuer is None:
            target.Pursuer = self
            self.isPrimaryPursuer = True
        else:
            self.isPrimaryPursuer = False
    
    def Guidance(self, dt):
        if self.target is not None:
            if self.target.contact == True:
                self.LNP = self.target.Target_state.copy()
                self.isPrimaryPursuer = False
                self.target = None

        if self.target != None:           
            R = self.target.Target_state - self.Missile_state

            if np.dot(R, R) < 5**2:
                self.contact = True
                self.target.Pursuer = None
                self.target.contact = True
                self.target = None
                self.isPrimaryPursuer = False
                return


            if np.linalg.norm(self.target.Target_state - self.Missile_state) > 150:
                errorX = self.target.Target_state[0] - self.Missile_state[0]
                errorY = self.target.Target_state[1] - self.Missile_state[1]
                errorZ = self.target.Target_state[2] - self.Missile_state[2]

                self.Missile_Vstate[0] += errorX * dt
                self.Missile_Vstate[1] += errorY * dt
                self.Missile_Vstate[2] += errorZ * dt

            else:
                N = 4
                Vr = self.target.Target_Vstate - self.Missile_Vstate

                Omega = np.cross(R, Vr) / np.dot(R, R)
                a = N * np.cross(Vr, Omega)

                a_norm = np.linalg.norm(a)
                if a_norm > 50 and a_norm > 0:
                    a = a * (50 / a_norm)

                self.Missile_Vstate += a * dt
        
        else:
            errorX = self.LNP[0] - self.Missile_state[0]
            errorY = self.LNP[1] - self.Missile_state[1]
            errorZ = self.LNP[2] - self.Missile_state[2]

            self.Missile_Vstate[0] += errorX * dt
            self.Missile_Vstate[1] += errorY * dt
            self.Missile_Vstate[2] += errorZ * dt
        
        v_norm = np.linalg.norm(self.Missile_Vstate)
        v_max = 45
        if v_norm > v_max and v_norm > 0:
            self.Missile_Vstate *= v_max / v_norm
        
        self.Missile_state += self.Missile_Vstate * dt

        self.Missile_Path[0].append(self.Missile_state[0])
        self.Missile_Path[1].append(self.Missile_state[1])
        self.Missile_Path[2].append(self.Missile_state[2])
    
