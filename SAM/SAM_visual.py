import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
from Target import Target
from Missile import Missile

plt.ion()

t = 0
dt = 0.01

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

ax.set_xlim3d(0, 500)
ax.set_ylim3d(0, 1000)
ax.set_zlim3d(0, 300)

ax.set_xlabel('X-axis')
ax.set_ylabel('Y-axis')
ax.set_zlabel('Z-axis')

Missiles = []
Targets = []

for _ in range(20):
    missile = Missile(ax)
    Missiles.append(missile)

for _ in range(15):
    target = Target(ax)
    Targets.append(target)

for _ in range (2500):
    contact = True

    t += dt
    for target in Targets:
        if target.contact == False:
            target.targetGuidance(dt)

            if target.Pursuer is not None:
                if target.Pursuer.target != target:
                    target.Pursuer = None

            # target.Target.set_markerfacecolor('blue')
            # if target.Pursuer is not None:
                # target.Target.set_markerfacecolor('yellow')
                # if target.Pursuer.isPrimaryPursuer:
                    # target.Target.set_markerfacecolor('green')

            target.Target.set_data([target.Target_state[0]], [target.Target_state[1]])
            target.Target.set_3d_properties([target.Target_state[2]])

            target.Tpath.set_data(target.Target_Path[0], target.Target_Path[1])
            target.Tpath.set_3d_properties(target.Target_Path[2])

            contact = False
    
    
    for missile in Missiles:
        missile.TargetsInRange = []
        missile.MissilesInRange = []

        if missile.target is not None and missile.isPrimaryPursuer == True:
            for target in Targets:
                if target.contact == False and np.linalg.norm(target.Target_state - missile.Missile_state) < 150:
                    if target.Pursuer == None:
                        for missile2 in Missiles:
                            if missile2 != missile and np.linalg.norm(missile2.Missile_state - missile.Missile_state) < 150 and missile2.isPrimaryPursuer == False:
                                missile2.catch(target)
                                break

        for missile2 in Missiles:
            if missile2.contact == False and np.linalg.norm(missile2.Missile_state - missile.Missile_state) < 150:
                missile.MissilesInRange.append(missile2)


        for target in Targets:
            dist = np.linalg.norm(target.Target_state - missile.Missile_state)
            if missile.activated == False and dist < 300:
                missile.activate(ax, target.Target_state)
            elif target.contact == False and dist < 150:
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
            missile.Guidance(dt)

        if missile.contact == True:
            Exp = ax.scatter(missile.Missile_state[0], missile.Missile_state[1], missile.Missile_state[2], color='orange', s=100, depthshade=True)
            missile_toRemove.append(missile)
            continue

        missile.Missile.set_data([missile.Missile_state[0]], [missile.Missile_state[1]])
        missile.Missile.set_3d_properties([missile.Missile_state[2]])

        # missile.Missile.set_markerfacecolor('red')
        # if missile.target is not None:
            # missile.Missile.set_markerfacecolor('yellow')
        # if missile.isPrimaryPursuer:
            # missile.Missile.set_markerfacecolor('green')

        missile.Mpath.set_data(missile.Missile_Path[0], missile.Missile_Path[1])
        if missile.activated == True:
            missile.Mpath.set_3d_properties(missile.Missile_Path[2])
    
    for missile in missile_toRemove:
        Missiles.remove(missile)
        

    if contact == True:
        ax.set_title(f"All Targets Destroyed! t = {t:.1f}s", color='red', fontsize=16)
        break

    plt.draw()
    plt.pause(0.01)

plt.ioff()
plt.show()
