import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
from DP2_Target import Target
from DP2_Missile_Linear import Missile_Linear
from DP2_Missile import Missile

def main(NumMissiles, NumMissilesL, NumTargets, Debug, Evade, EvasionMethod, TargetPhase, _2D, Plot_LOS_rate, Plot, N, Plot_Obs_Convergence):
    plt.rcParams.update({'font.size': 12})
    if Plot:
        plt.ion()

    t = 0
    dt = 0.005

    fig = plt.figure()
    if not _2D:
        ax = fig.add_subplot(111, projection='3d')
    else:
        ax = fig.add_subplot()
        if Plot:
            plt.grid()

    if TargetPhase == "Vertical":
        if not _2D:
            ax.set_xlim3d(0, 5000)
            ax.set_ylim3d(0, 3000)
            ax.set_zlim3d(0, 8000)
            # ax.set_xticks([0, 2500, 5000])
            # ax.set_yticks([1500])
            # ax.set_zticks([0, 4000, 8000])
            # ax.grid(True)
        else:
            ax.set_xlim(0, 3000)
            ax.set_ylim(0, 8000)
        
        ActivationDistance = 7000
        CatchDistance = 4500

    elif TargetPhase == "Horizontal":
        if not _2D:
            ax.set_xlim3d(0, 5000)
            ax.set_ylim3d(0, 10000)
            ax.set_zlim3d(0, 5000)
        else:
            ax.set_xlim(0, 10000)
            ax.set_ylim(0, 5000)
        
        ActivationDistance = 4000
        CatchDistance = 3500

    

    if not _2D:
        ax.set_xlabel('X-axis')
        ax.set_ylabel('Y-axis')
        ax.set_zlabel('Z-axis')
    elif Plot:
            plt.xticks([])
            plt.yticks([])

    Missiles = []
    Targets = []

    for _ in range(NumMissiles):
        missile = Missile(ax, _2D, dt, N)
        Missiles.append(missile)

    for _ in range(NumMissilesL):
        missile = Missile_Linear(ax, _2D, dt, N)
        Missiles.append(missile)

    MissileHolder = Missiles.copy()

    for _ in range(NumTargets):
        target = Target(ax, TargetPhase, Evade, EvasionMethod, _2D)
        Targets.append(target)

    TargetsDestroyed = 0

    for _ in range (4000):
        contact = True

        t += dt
        targetsToRemove = []
        for target in Targets:
            if target.contact == True:
                if not _2D:
                    Exp = ax.scatter(target.Target_state[0], target.Target_state[1], target.Target_state[2], color='orange', s=100, depthshade=True)
                else:
                    Exp = ax.scatter(target.Target_state[1], target.Target_state[2], color='orange', s=100, zorder=100)
                targetsToRemove.append(target)
                continue

            if target.contact == False:
                target.targetGuidance(dt, Evade, EvasionMethod, _2D)

                if target.Pursuer is not None:
                    if target.Pursuer.target != target:
                        target.Pursuer = None

                if Debug:
                    target.Target.set_markerfacecolor('blue')
                    if target.Pursuer is not None:
                        target.Target.set_markerfacecolor('yellow')
                        if target.Pursuer.isPrimaryPursuer:
                            target.Target.set_markerfacecolor('green')

                if not _2D:
                    target.Target.set_data([target.Target_state[0]], [target.Target_state[1]])
                    target.Target.set_3d_properties([target.Target_state[2]])

                    target.Tpath.set_data(target.Target_Path[0], target.Target_Path[1])
                    target.Tpath.set_3d_properties(target.Target_Path[2])
                else:
                    target.Target.set_data([target.Target_state[1]], [target.Target_state[2]])
                    target.Tpath.set_data(target.Target_Path[1], target.Target_Path[2])

                contact = False
        
        for target in targetsToRemove:
            Targets.remove(target)
        
        for missile in Missiles:
            missile.TargetsInRange = []
            missile.MissilesInRange = []

            if missile.target is not None and missile.isPrimaryPursuer == True:
                for target in Targets:
                    if target.contact == False and np.linalg.norm(target.Target_state - missile.Missile_state) < CatchDistance:
                        if target.Pursuer == None:
                            for missile2 in missile.MissilesInRange:
                                if missile2.isPrimaryPursuer == False:
                                    missile2.catch(target)
                                    break

            for missile2 in Missiles:
                if missile2.contact == False and np.linalg.norm(missile2.Missile_state - missile.Missile_state) < CatchDistance:
                    missile.MissilesInRange.append(missile2)


            for target in Targets:
                dist = np.linalg.norm(target.Target_state - missile.Missile_state)
                if missile.activated == False and dist < ActivationDistance:
                    missile.activate(target.Target_state, target.Target_Vstate, Evade, EvasionMethod, t)
                if target.contact == False and dist < CatchDistance:
                    missile.TargetsInRange.append(target)
                    if target.Pursuer == None and missile.isPrimaryPursuer == False or missile.target is None:
                        missile.catch(target)
                
            targetToCatch = None
            check = False
            for missile2 in missile.MissilesInRange:
                for target2 in missile2.TargetsInRange:
                    if target2 is not None and target2.contact == False:
                        if target2.Pursuer == None and missile.isPrimaryPursuer == False:
                            targetToCatch = target2
                            check = True
                            break
                        elif missile.target == None:
                            targetToCatch = target2
                if check == True:
                    break
            if check == True:
                missile.catch(targetToCatch)

        missile_toRemove = []    
        for missile in Missiles:
            if missile.activated == True:
                TargetsDestroyed += missile.Guidance(dt, t, _2D)

            if missile.contact == True:
                if not _2D:
                    Exp = ax.scatter(missile.Missile_state[0], missile.Missile_state[1], missile.Missile_state[2], color='orange', s=100, depthshade=True)
                else:
                    Exp = ax.scatter(missile.Missile_state[1], missile.Missile_state[2], color='orange', s=100, zorder=100)
                missile_toRemove.append(missile)
                continue

            if not _2D:
                missile.Missile.set_data([missile.Missile_state[0]], [missile.Missile_state[1]])
                missile.Missile.set_3d_properties([missile.Missile_state[2]])
            else:
                missile.Missile.set_data([missile.Missile_state[1]], [missile.Missile_state[2]])

            if Debug:
                missile.Missile.set_markerfacecolor('red')
                if missile.target is not None:
                    missile.Missile.set_markerfacecolor('yellow')
                if missile.isPrimaryPursuer:
                    missile.Missile.set_markerfacecolor('green')

            if not _2D:
                missile.Mpath.set_data(missile.Missile_Path[0], missile.Missile_Path[1])
                if missile.activated == True:
                    missile.Mpath.set_3d_properties(missile.Missile_Path[2])
            else:
                missile.Mpath.set_data(missile.Missile_Path[1], missile.Missile_Path[2])
        
        for missile in missile_toRemove:
            Missiles.remove(missile)
            
        if contact == True: 
            # if (TargetsDestroyed == NumTargets):
            #     ax.set_title(f"All Targets Destroyed! t = {t:.1f}s", color='red', fontsize=16)
            # else:
            #     ax.set_title(f"Failure! {NumTargets - TargetsDestroyed} Targets Missed!", color='red', fontsize=16)
            break

        if Plot:
            if _2D:
                plt.grid()
            if t % 0.05 == 0:
                plt.draw()
            plt.pause(0.001)

    if Plot:
        plt.ioff()
        plt.show()

    if Plot_LOS_rate:
        fig2, ax2 = plt.subplots()

        linear_labeled = False
        nonlinear_labeled = False

        for m in MissileHolder:
            t_arr = np.array(m.LOS_rrate_t)
            if m.Linear:
                color = 'black'
                label = 'Linear' if not linear_labeled else None
                linear_labeled = True
            else:
                color = 'red'
                label = 'Non-Linear' if not nonlinear_labeled else None
                nonlinear_labeled = True

            ax2.plot(t_arr - t_arr[0], m.LOS_rrate, color=color, label=label)
            
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('LOS Rate (rad/s)')
        ax2.legend()
        plt.show()
    
    if Plot_Obs_Convergence:
        fig3, ax3 = plt.subplots(3, 1, sharex=True, figsize=(10, 8))

        linear_labeled = False
        nonlinear_labeled = False

        for m in MissileHolder:
            t_arr = np.array(m.VelCompT)
            if m.Linear:
                color = 'black'
                label = 'Linear' if not linear_labeled else None
                linear_labeled = True
            else:
                color = 'red'
                label = 'Non-Linear' if not nonlinear_labeled else None
                nonlinear_labeled = True

            VelComp = np.array(m.VelComp)

            Xplot = VelComp[:, 0]
            Yplot = VelComp[:, 1]
            Zplot = VelComp[:, 2]

            ax3[0].plot(t_arr, Xplot, color=color, label=label)
            ax3[0].set_ylim([-150, 150])
            ax3[1].plot(t_arr, Yplot, color=color)
            ax3[1].set_ylim([-150, 150])
            ax3[2].plot(t_arr, Zplot, color=color)
            ax3[2].set_ylim([-150, 150])

        ax3[0].set_ylabel("X error (m/s)")
        ax3[1].set_ylabel("Y error (m/s)")
        ax3[2].set_ylabel("Z error (m/s)")
        ax3[2].set_xlabel("Time (s)")

        ax3[0].set_title("Observer Velocity Error Convergence")

        for ax in ax3:
            ax.grid(True)

        ax3[0].legend()

        fig3.tight_layout()


        fig4, ax4 = plt.subplots(3, 1, sharex=True, figsize=(10, 8))

        linear_labeled = False
        nonlinear_labeled = False

        for m in MissileHolder:
            t_arr = np.array(m.VelCompT)

            if m.Linear:
                color = 'black'
                label = 'Linear' if not linear_labeled else None
                linear_labeled = True
            else:
                color = 'red'
                label = 'Non-Linear' if not nonlinear_labeled else None
                nonlinear_labeled = True

            Xplot = []
            Yplot = []
            Zplot = []

            VCLC = np.array(m.VelCompLinCond)

            check = False
            for i in range(len(m.VelComp)):
                if i >= len(m.VelCompLinCond) or check:
                    Xplot.append(None)
                    Yplot.append(None)
                    Zplot.append(None)
                elif VCLC[i]:
                    Xplot.append(m.VelComp[i][0])
                    Yplot.append(m.VelComp[i][1])
                    Zplot.append(m.VelComp[i][2])
                else:
                    Xplot.append(None)
                    Yplot.append(None)
                    Zplot.append(None)
                    check = True

            ax4[0].plot(t_arr, Xplot, color=color, label=label)
            ax4[0].set_ylim([-150, 150])
            ax4[1].plot(t_arr, Yplot, color=color)
            ax4[1].set_ylim([-150, 150])
            ax4[2].plot(t_arr, Zplot, color=color)
            ax4[2].set_ylim([-150, 150])

        ax4[0].set_ylabel("X error (m/s)")
        ax4[1].set_ylabel("Y error (m/s)")
        ax4[2].set_ylabel("Z error (m/s)")
        ax4[2].set_xlabel("Time (s)")

        ax4[0].set_title("Observer Velocity Error Convergence Under Positive Closing Velocity")

        for ax in ax4:
            ax.grid(True)

        ax4[0].legend()
        fig4.tight_layout()
        plt.show()



        fig5, ax5 = plt.subplots(3, 1, sharex=True, figsize=(10, 8))

        linear_labeled = False
        nonlinear_labeled = False

        for m in MissileHolder:
            t_arr = np.array(m.VelCompT)
            if m.Linear:
                color = 'black'
                label = 'Linear' if not linear_labeled else None
                linear_labeled = True
            else:
                color = 'red'
                label = 'Non-Linear' if not nonlinear_labeled else None
                nonlinear_labeled = True

            VelComp = np.array(m.VelCompPerc)

            Xplot = VelComp[:, 0]
            Yplot = VelComp[:, 1]
            Zplot = VelComp[:, 2]

            ax5[0].plot(t_arr, Xplot, color=color, label=label)
            ax5[0].set_ylim([-150, 150])
            ax5[1].plot(t_arr, Yplot, color=color)
            ax5[1].set_ylim([-150, 150])
            ax5[2].plot(t_arr, Zplot, color=color)
            ax5[2].set_ylim([-150, 150])

        ax5[0].set_ylabel("X error (%)")
        ax5[1].set_ylabel("Y error (%)")
        ax5[2].set_ylabel("Z error (%)")
        ax5[2].set_xlabel("Time (s)")

        ax5[0].set_title("Observer Percentage Velocity Error Convergence")

        for ax in ax5:
            ax.grid(True)

        ax5[0].legend()

        fig5.tight_layout()



        fig6, ax6 = plt.subplots(3, 1, sharex=True, figsize=(10, 8))

        linear_labeled = False
        nonlinear_labeled = False

        for m in MissileHolder:
            t_arr = np.array(m.VelCompT)

            if m.Linear:
                color = 'black'
                label = 'Linear' if not linear_labeled else None
                linear_labeled = True
            else:
                color = 'red'
                label = 'Non-Linear' if not nonlinear_labeled else None
                nonlinear_labeled = True

            Xplot = []
            Yplot = []
            Zplot = []

            VCLC = np.array(m.VelCompLinCond)

            for i in range(len(m.VelCompPerc)):
                if i >= len(m.VelCompLinCond):
                    Xplot.append(None)
                    Yplot.append(None)
                    Zplot.append(None)
                elif VCLC[i]:
                    Xplot.append(m.VelCompPerc[i][0])
                    Yplot.append(m.VelCompPerc[i][1])
                    Zplot.append(m.VelCompPerc[i][2])
                else:
                    Xplot.append(None)
                    Yplot.append(None)
                    Zplot.append(None)

            ax6[0].plot(t_arr, Xplot, color=color, label=label)
            ax6[0].set_ylim([-150, 150])
            ax6[1].plot(t_arr, Yplot, color=color)
            ax6[1].set_ylim([-150, 150])
            ax6[2].plot(t_arr, Zplot, color=color)
            ax6[2].set_ylim([-150, 150])

        ax6[0].set_ylabel("X error (%)")
        ax6[1].set_ylabel("Y error (%)")
        ax6[2].set_ylabel("Z error (%)")
        ax6[2].set_xlabel("Time (s)")

        ax6[0].set_title("Observer Percentage Velocity Error Convergence Under Positive Closing Velocity")

        for ax in ax6:
            ax.grid(True)

        ax6[0].legend()
        fig6.tight_layout()
        plt.show()

    return TargetsDestroyed;   