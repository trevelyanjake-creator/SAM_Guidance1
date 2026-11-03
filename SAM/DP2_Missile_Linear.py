import numpy as np
from DP2_Missile import Missile
import random

class Missile_Linear(Missile):
    def __init__(self, ax, _2D, dt, N):
        self.target = None
        self.Linear = True
        self.ax = ax
        self.contact = False
        self.activated = False
        self.LNP = np.array([None, None, None])
        self.isPrimaryPursuer = False
        self.TargetsInRange = []
        self.MissilesInRange = []
        self.PreviousV = None

        self.N = N

        self.LOS_rrate = []
        self.LOS_rrate_t = []

        self.rho_seaLevel = 1.225
        self.A = np.pi * (0.36/2)**2
        self.Cd = 0.43

        self.rho = 0.91
        self.m = 477
        self.vAbs = 730

        self.F_steeringDelay = np.zeros(3)
        self.tau = 0.2

        Phi_FN = 0.002**2 * 0.1
        self.sigma  = np.sqrt(Phi_FN / dt)

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
            (self.Missile,) = ax.plot([self.Missile_state[0]], [self.Missile_state[1]], [self.Missile_state[2]], marker='o', linestyle='', color = 'black')
        else:
            (self.Missile,) = ax.plot([self.Missile_state[1]], [self.Missile_state[2]], marker='o', linestyle='', color = 'black')

        MXp = []
        MYp = []
        MZp = []

        MXp.append(self.Missile_Xscatter)
        MYp.append(self.Missile_Yscatter)
        MZp.append(self.Missile_Zscatter)

        self.Missile_Path = [MXp, MYp, MZp]

        if not _2D:
            (self.Mpath,) = ax.plot([self.Missile_Path[0]], [self.Missile_Path[1]], [self.Missile_Path[2]], color = 'black')
        else:
            (self.Mpath,) = ax.plot([self.Missile_Path[1]], [self.Missile_Path[2]], color = 'black')
    
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

    def Guidance(self, dt, t, _2D):
        if self.target is not None:
            if self.target.contact == True:
                self.target.Puruser = None
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
            noise = random.gauss(0, self.sigma) * np.random.randn(3)
            RNoisy = R + noise

            if np.linalg.norm(R) < 30:
                self.contact = True
                self.target.Pursuer = None
                self.target.contact = True
                self.target = None
                self.isPrimaryPursuer = False
                return 1

            if np.linalg.norm(R) > 4000:
                accel = R * (ThrustAccel / self.m) / np.linalg.norm(R)
                self.Missile_Vstate += accel * dt

            else:
                R_hat = RNoisy/np.linalg.norm(RNoisy)
                Vtp_real = self.target.Target_Vstate - self.Missile_Vstate
                Vtp = self.updateObserver(RNoisy, dt)

                self.VelComp.append(Vtp - Vtp_real)
                self.VelCompPerc.append((Vtp - Vtp_real) / Vtp_real * 100)
                self.VelCompT.append(t)

                Vc = -(np.dot(R_hat, Vtp))

                error = Vtp - np.dot(Vtp, R_hat) * R_hat

                F_steering = self.N * (error * Vc / np.linalg.norm(RNoisy))

                v_hat = self.Missile_Vstate / (np.linalg.norm(self.Missile_Vstate) + 1e-6)
                F_steering -= np.dot(F_steering, v_hat) * v_hat 

                v_norm = np.linalg.norm(self.Missile_Vstate)
                v_max = 850
                SteeringMax = 55 * 9.81 * (v_norm / v_max)**2 * (self.rho/self.rho_seaLevel)

                Fs_norm = np.linalg.norm(F_steering)
                if Fs_norm > SteeringMax:
                    F_steering *= SteeringMax / (Fs_norm + 1e-6)

                k = 1/2 * self.rho * self.A * self.Cd 
                s = np.linalg.norm(self.PreviousV)
                DragJacobian = k * s * np.identity(3) + k / s * np.outer(self.PreviousV, self.PreviousV)
                ThrustJacobian = ThrustAccel/s * (np.identity(3) - np.outer(self.PreviousV, self.PreviousV) / s**2)

                self.F_steeringDelay += (F_steering - self.F_steeringDelay) / self.tau * dt
                F_steeringCommand = self.F_steeringDelay * m

                F_heading = self.PreviousV / np.linalg.norm(self.PreviousV) * ThrustAccel + ThrustJacobian @ (self.Missile_Vstate - self.PreviousV)

                F_drag = k * s * self.PreviousV + DragJacobian @ (self.Missile_Vstate - self.PreviousV)

                F_gravity = np.array([0, 0, -9.81*self.m])

                a = (F_steeringCommand + F_heading + F_gravity - F_drag) / self.m

                self.accel = a

                self.Missile_Vstate += a * dt

                Omega_Real = np.cross(R, Vtp_real) / np.dot(R, R)
                self.LOS_rrate.append(np.linalg.norm(Omega_Real))
                self.LOS_rrate_t.append(t)

                self.prevR = RNoisy
        
        else:
            R = self.LNP - self.Missile_state
            accel = (R * ThrustAccel / m) / np.linalg.norm(R)

            F_gravity = np.array([0, 0, -9.81*m])

            accel += F_gravity / m
            
            self.Missile_Vstate += accel * dt
        
        v_norm = np.linalg.norm(self.Missile_Vstate)
        v_max = 850
        if v_norm > v_max:
            self.Missile_Vstate *= v_max / (v_norm + 1e-6)
        
        self.Missile_state += self.Missile_Vstate * dt

        self.PreviousV = self.Missile_Vstate

        if _2D:
            self.Missile_state[0] = 0
        self.Missile_Path[0].append(self.Missile_state[0])
        self.Missile_Path[1].append(self.Missile_state[1])
        self.Missile_Path[2].append(self.Missile_state[2])

        if self.Missile_state[2] < 0 and t > 1:
            self.contact = True
            if self.isPrimaryPursuer:
                self.target.Pursuer = None
            self.target = None
            self.isPrimaryPursuer = False

        return 0