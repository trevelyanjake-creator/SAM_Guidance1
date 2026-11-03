import numpy as np

class Target:
    def __init__(self, ax, Phase, Evade, EvasionMethod, _2D):
        self.contact = False
        self.Evading = False
        self.EvadingTimer = 0
        self.HasEvaded = False
        self.Pursuer = None

        Xp = []
        Yp = []
        Zp = []

        self.Target_Path = [Xp, Yp, Zp]

        if Phase == "Vertical":
            if Evade and EvasionMethod == "Spiral":
                Target_Xscatter = np.random.default_rng().uniform(low = 1000, high = 4000)
                Target_Yscatter = np.random.default_rng().uniform(low = 1500, high = 2000)
                Target_Zscatter = np.random.default_rng().uniform(low = 7500, high = 8000)

                Target_VXscatter = 0
                Target_VYscatter = 0
                Target_VZscatter = 0

            else:
                Target_Xscatter = np.random.default_rng().uniform(low = 500, high = 4500)
                Target_Yscatter = np.random.default_rng().uniform(low = 500, high = 1000)
                Target_Zscatter = np.random.default_rng().uniform(low = 7500, high = 8000)

                Target_VXscatter = 0
                Target_VYscatter = 100
                Target_VZscatter = -600
        
        elif Phase == "Horizontal":
            Target_Xscatter = np.random.default_rng().uniform(low = 500, high = 4500)
            Target_Yscatter = np.random.default_rng().uniform(low = 0, high = 250)
            Target_Zscatter = np.random.default_rng().uniform(low = 3750, high = 4250)

            Target_VXscatter = 0
            if not _2D:
                Target_VYscatter = 800
                Target_VZscatter = -150
            else:
                Target_VYscatter = 500
                Target_VZscatter = -100
        
        if _2D:
            Target_Xscatter = 0

        Xp.append(Target_Xscatter)
        Yp.append(Target_Yscatter)
        Zp.append(Target_Zscatter)
        
        self.Target_state = np.array([Target_Xscatter, Target_Yscatter, Target_Zscatter], dtype=float)
        self.Target_Vstate = np.array([Target_VXscatter, Target_VYscatter, Target_VZscatter], dtype=float)

        if not _2D:
            (self.Target,) = ax.plot([self.Target_state[0]], [self.Target_state[1]], [self.Target_state[2]], marker='o', color = 'blue')
            (self.Tpath,) = ax.plot(self.Target_Path[0], self.Target_Path[1], self.Target_Path[2], color = 'blue')
        else:
            (self.Target,) = ax.plot([self.Target_state[1]], [self.Target_state[2]], marker='o', color = 'blue')
            (self.Tpath,) = ax.plot(self.Target_Path[1], self.Target_Path[2], color = 'blue')
    
    def Evade_Stall(self, dt):
        if self.EvadingTimer < 0.4:
            accelZ = self.EvadingTimer/0.4 * 500
            accelY = self.EvadingTimer/0.4 * 100
            self.Target_Vstate[2] += accelZ * dt
            self.Target_Vstate[1] += accelY * dt
        if self.EvadingTimer < 0.8:
            accelZ = 500
            accelY = 100 
            self.Target_Vstate[2] += accelZ * dt
            self.Target_Vstate[1] += accelY * dt
        elif self.EvadingTimer < 1.6:
            accel = -500
            self.Target_Vstate[2] += accel * dt
        else:
            self.Evadeing = False
            self.HasEvaded = True
            self.EvadingTimer = 0
        
        self.EvadingTimer += dt
    
    def Evade_Spiral(self, dt):
        descent_rate = 100000
        spiral_radius = 10000
        spiral_rate = 0.4 * np.pi

        ax = -spiral_radius * spiral_rate**2 * np.cos(spiral_rate * self.EvadingTimer)
        ay = -spiral_radius * spiral_rate**2 * np.sin(spiral_rate * self.EvadingTimer)
        az = -descent_rate

        self.Target_Astate = np.array([ax, ay, az])
        self.Target_Vstate += self.Target_Astate * dt

        self.EvadingTimer += dt


    def Evade(self, Method, dt, Evade):
        if Evade == False:
            return False
        
        R = 3500 # Simply a number larger than 100 for if the target does not yet have a puruser
        if self.Pursuer != None:
            R = np.linalg.norm(self.Target_state - self.Pursuer.Missile_state)

        if Method == "Stall" and R < 850 and self.HasEvaded == False:
            self.Evade_Stall(dt)
            return True
        elif Method == "Spiral":
            self.Evade_Spiral(dt)
            return True
        else:
            return False


    def targetGuidance(self, dt, Evade, Evasion_Method, _2D):
        if self.Evade(Evasion_Method, dt, Evade) == False:
            enforce = False
            if self.Target_state[0] < 750:
                self.Target_Vstate[0] += 0.5 * (750 - self.Target_state[0])/750
                enforce = True
            elif self.Target_state[0] > 4250:
                self.Target_Vstate[0] += -0.5 * (self.Target_state[0] - 4250)/750
                enforce = True


            if enforce == False:
                axisChanging = np.random.randint(0, 3)
                Delta = np.random.default_rng().uniform(low=-1, high=1)

                self.Target_Vstate[axisChanging] += Delta

            axisChanging = np.random.randint(0, 3)
            Delta = np.random.default_rng().uniform(low=-10, high=10)
            self.Target_Vstate[axisChanging] += Delta

        self.Target_Vstate[2] += -9.81 * dt

        v_norm = np.linalg.norm(self.Target_Vstate)
        v_max = 700
        if v_norm > v_max:
            self.Target_Vstate *= v_max / (v_norm + 1e-6)

        self.Target_state += self.Target_Vstate * dt

        if _2D:
            self.Target_state[0] = 0
        self.Target_Path[0].append(self.Target_state[0])
        self.Target_Path[1].append(self.Target_state[1])
        self.Target_Path[2].append(self.Target_state[2])

        if self.Target_state[2] < 0:
            self.contact = True
            if self.Pursuer is not None:
                self.Pursuer.target = None
                self.Pursuer.LNP = [250, 0, 5000]
                if self.Pursuer.isPrimaryPursuer:
                    self.Pursuer.isPrimaryPursuer = False
                self.Pursuer = None