import numpy as np

def targetGuidance(state, Vstate, dt, path):
    axisChanging = np.random.randint(0, 3)
    Delta = np.random.default_rng().uniform(low=-1, high=1)

    Vstate[axisChanging] += Delta

    v_norm = np.linalg.norm(Vstate)
    v_max = 35
    if v_norm > v_max and v_norm > 0:
        Vstate *= v_max / v_norm

    state += Vstate * dt

    path[0].append(float(state[0]))
    path[1].append(float(state[1]))
    path[2].append(float(state[2]))

    return state, Vstate