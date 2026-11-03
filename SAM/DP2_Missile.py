import numpy as np
import random

class Missile:
    def __init__(self, ax, _2D, dt, N):
        self.target = None
        self.Linear = False
        self.ax = ax
        self.contact = False
        self.activated = False
        self.LNP = np.array([None, None, None])
        self.isPrimaryPursuer = False
        self.TargetsInRange = []
        self.MissilesInRange = []

        self.N = N

        self.LOS_rrate = []
        self.LOS_rrate_t = []

        self.rho_seaLevel = 1.225
        self.A = np.pi * (0.36/2)**2
        self.Cd = 0.43

        self.F_steeringDelay = np.zeros(3)
        self.tau = 0.2

        self.sigma = 0.0002

        self.prevR = 0

        self.R_hat = None
        self.Vr_hat = np.zeros(3)
        self.observer_beta = 3.0

        self.VelComp = []
        self.VelCompT = []
        self.VelCompLinCond = []
        self.VelCompPerc = []

        self.accel = np.zeros(3)

        if not _2D:
            self.Missile_Xscatter = random.gauss(2500, 500)
        else:
            self.Missile_Xscatter = 0
        self.Missile_Yscatter = random.gauss(1500, 500)
        self.Missile_Zscatter = 0

        self.Missile_state = np.array([self.Missile_Xscatter, self.Missile_Yscatter, self.Missile_Zscatter], dtype=float)

        if not _2D:
            (self.Missile,) = ax.plot([self.Missile_state[0]], [self.Missile_state[1]], [self.Missile_state[2]], marker='o', linestyle='', color = 'red')
        else:
            (self.Missile,) = ax.plot([self.Missile_state[1]], [self.Missile_state[2]], marker='o', linestyle='', color = 'red')

        MXp = []
        MYp = []
        MZp = []

        MXp.append(self.Missile_Xscatter)
        MYp.append(self.Missile_Yscatter)
        MZp.append(self.Missile_Zscatter)

        self.Missile_Path = [MXp, MYp, MZp]

        if not _2D:
            (self.Mpath,) = ax.plot([self.Missile_Path[0]], [self.Missile_Path[1]], [self.Missile_Path[2]], color = 'red')
        else:
            (self.Mpath,) = ax.plot([self.Missile_Path[1]], [self.Missile_Path[2]], color = 'red')

    def getRho(self):
        T = 15.04 - 0.00649 * self.Missile_state[2]
        P = 101.29 * ((T + 273.15) / 288.08) ** 5.356

        rho = P / (.2869 * (T + 273.15))

        return rho

    def activate(self, target_state, targetV_state, Evade, EvasionMethod, t):
        R = target_state - self.Missile_state
        self.activated = True

        if Evade and EvasionMethod == "Spiral":
            self.LNP = target_state
        else:
            a = 600
            coeffs = [(a**2) / 4, 0, -np.dot(targetV_state, targetV_state), -2 * np.dot(R, targetV_state), -np.dot(R, R)]
            roots = np.roots(coeffs)

            valid = [r.real for r in roots if abs(r.imag) < 1e-6 and r.real > 0]
            TimeToIntercept = min(valid)

            self.LNP = target_state + targetV_state * TimeToIntercept # LNP is nolonger the correct name but it makes the code easier

        Missile_VX = 0
        Missile_VY = 0
        Missile_VZ = 50

        self.Missile_Vstate = np.array([Missile_VX, Missile_VY, Missile_VZ], dtype=float)
    
    def catch(self, target):
        self.target = target
        if target.Pursuer is None:
            closest_Missile = self
            closest_Missile_Old = self
            while True:
                closest_Missile_Old = closest_Missile
                for missile in closest_Missile.MissilesInRange:
                    if missile.isPrimaryPursuer == False and np.linalg.norm(missile.Missile_state - target.Target_state) < np.linalg.norm(closest_Missile.Missile_state - target.Target_state):
                        closest_Missile = missile
                        break
                if closest_Missile_Old == closest_Missile:
                    break
            closest_Missile.target = target
            target.Pursuer = closest_Missile
            closest_Missile.isPrimaryPursuer = True
    
    def updateObserver(self, RNoisy, dt):
        if self.R_hat is None:
            self.R_hat = RNoisy.copy()
            self.prevR = RNoisy.copy()
            self.Vr_hat = self.target.Target_Vstate - self.Missile_Vstate
            return self.Vr_hat

        R_norm = np.linalg.norm(RNoisy)
        R_hat_meas = RNoisy / R_norm

        Vc = -np.dot(R_hat_meas, self.Vr_hat)
        Vc_obs = Vc_obs = np.clip(abs(Vc), 1.0, 1000.0)

        if Vc > 0:
            self.VelCompLinCond.append(True)
        else:
            self.VelCompLinCond.append(False)

        alpha = self.N * Vc_obs / R_norm
        gamma = self.N * Vc_obs**2 / R_norm**2

        I = np.eye(3)
        P = I - np.outer(R_hat_meas, R_hat_meas)

        L1 = 5.0 * alpha * I
        L2 = 4.0 * alpha**2 * I - gamma * P

        innovation = RNoisy - self.R_hat

        R_hat_dot = self.Vr_hat + L1 @ innovation
        Vr_hat_dot = -self.accel + L2 @ innovation

        self.R_hat += R_hat_dot * dt
        self.Vr_hat += Vr_hat_dot * dt

        return self.Vr_hat

    def Guidance(self, dt, t, _2D):
        if self.target is not None:
            if self.target.contact == True:
                self.target.Pursuer = None
                self.LNP = self.target.Target_state.copy()
                self.isPrimaryPursuer = False
                self.target = None
                self.F_steeringDelay = np.zeros(3)
                self.R_hat = None
                self.Vr_hat = np.zeros(3)
        
        if t < 5:
            ThrustAccel = 82404
            m = 639 - t/5 * 147.5
        elif t < 26 :
            ThrustAccel = 20601
            m = 491.5 - (t - 5)/21 * 147.5
        else:
            ThrustAccel = 0
            m = 344

        if self.target is not None:           
            R = self.target.Target_state - self.Missile_state
            noise  = random.gauss(0, self.sigma*np.linalg.norm(R)) * np.random.randn(3)
            RNoisy = R + noise

            if np.linalg.norm(R) < 30:
                self.contact = True
                self.target.Pursuer = None
                self.target.contact = True
                self.target = None
                self.isPrimaryPursuer = False
                return 1

            if np.linalg.norm(R) > 4000:
                accel = R * (ThrustAccel / m) / np.linalg.norm(R)
                self.Missile_Vstate += accel * dt

            else:
                Vr_Real = self.target.Target_Vstate - self.Missile_Vstate
                Vr = self.updateObserver(RNoisy, dt)
                self.VelComp.append(Vr - Vr_Real)
                self.VelCompPerc.append((Vr - Vr_Real) / Vr_Real * 100)
                self.VelCompT.append(t)

                Omega_Real = np.cross(R, Vr_Real) / np.dot(R, R)
                noise  = self.sigma * np.random.randn(3)
                Omega = np.cross(RNoisy, Vr) / np.dot(RNoisy, RNoisy)

                F_steering = self.N * np.cross(Vr, Omega)

                v_hat = self.Missile_Vstate / (np.linalg.norm(self.Missile_Vstate) + 1e-6)
                F_steering -= np.dot(F_steering, v_hat) * v_hat 

                v_norm = np.linalg.norm(self.Missile_Vstate)
                v_max = 850
                SteeringMax = 55 * 9.81 * (v_norm / v_max)**2 * (self.getRho()/self.rho_seaLevel)

                Fs_norm = np.linalg.norm(F_steering)
                if Fs_norm > SteeringMax:
                    F_steering *= SteeringMax / (Fs_norm + 1e-6)

                self.F_steeringDelay += (F_steering - self.F_steeringDelay) / self.tau * dt
                F_steeringCommand = self.F_steeringDelay * m


                F_heading = self.Missile_Vstate / (np.linalg.norm(self.Missile_Vstate) + 1e-6) * ThrustAccel


                F_drag = 0.5 * self.getRho() * self.A * self.Cd * self.Missile_Vstate * np.linalg.norm(self.Missile_Vstate)


                F_gravity = np.array([0, 0, -9.81*m])


                a = (F_heading + F_steeringCommand + F_gravity - F_drag) / m

                self.accel = a

                self.Missile_Vstate += a * dt

                self.LOS_rrate.append(np.linalg.norm(Omega_Real))
                self.LOS_rrate_t.append(t)

                self.prevR = RNoisy
        
        else:
            R = self.LNP - self.Missile_state
            accel = (R * ThrustAccel / m) / np.linalg.norm(R)

            F_gravity = np.array([0, 0, -9.81*m])

            accel += F_gravity/m

            self.Missile_Vstate += accel * dt
        
        # v_norm = np.linalg.norm(self.Missile_Vstate)
        # v_max = 850
        # if (v_norm > v_max):
        #     print(v_norm)
        
        self.Missile_state += self.Missile_Vstate * dt

        if _2D:
            self.Missile_state[0] = 0
        self.Missile_Path[0].append(self.Missile_state[0])
        self.Missile_Path[1].append(self.Missile_state[1])
        self.Missile_Path[2].append(self.Missile_state[2])

        if self.Missile_state[2] < 0 and t > 3.5:
            self.contact = True
            if self.isPrimaryPursuer:
                self.target.Pursuer = None
            self.target = None
            self.isPrimaryPursuer = False

        return 0