import numpy as np
import modern_robotics as mr

M01 = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0.089159], [0, 0, 0, 1]]
M12 = [[0, 0, 1, 0.28], [0, 1, 0, 0.13585], [-1, 0, 0, 0], [0, 0, 0, 1]]
M23 = [[1, 0, 0, 0], [0, 1, 0, -0.1197], [0, 0, 1, 0.395], [0, 0, 0, 1]]
M34 = [[0, 0, 1, 0], [0, 1, 0, 0], [-1, 0, 0, 0.14225], [0, 0, 0, 1]]
M45 = [[1, 0, 0, 0], [0, 1, 0, 0.093], [0, 0, 1, 0], [0, 0, 0, 1]]
M56 = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0.09465], [0, 0, 0, 1]]
M67 = [[1, 0, 0, 0], [0, 0, 1, 0.0823], [0, -1, 0, 0], [0, 0, 0, 1]]
G1 = np.diag([0.010267495893, 0.010267495893, 0.00666, 3.7, 3.7, 3.7])
G2 = np.diag([0.22689067591, 0.22689067591, 0.0151074, 8.393, 8.393, 8.393])
G3 = np.diag([0.049443313556, 0.049443313556, 0.004095, 2.275, 2.275, 2.275])
G4 = np.diag([0.111172755531, 0.111172755531, 0.21942, 1.219, 1.219, 1.219])
G5 = np.diag([0.111172755531, 0.111172755531, 0.21942, 1.219, 1.219, 1.219])
G6 = np.diag([0.0171364731454, 0.0171364731454, 0.033822, 0.1879, 0.1879, 0.1879])
Glist = [G1, G2, G3, G4, G5, G6]
Mlist = [M01, M12, M23, M34, M45, M56, M67]
Slist = [
    [0, 0, 0, 0, 0, 0],
    [0, 1, 1, 1, 0, 1],
    [1, 0, 0, 0, -1, 0],
    [0, -0.089159, -0.089159, -0.089159, -0.10915, 0.005491],
    [0, 0, 0, 0, 0.81725, 0],
    [0, 0, 0.425, 0.81725, 0, 0.81725],
]
# beginning of initialization
thetalist0 = np.transpose(np.array([0, 0, 0, 0, 0, 0]))
thetalist0 = list(thetalist0)
f = open("simulation1.csv", "w")  # creation of csv file
f.write(str(thetalist0)[1 : len(str(thetalist0)) - 1] + "\n")
thetalist0 = np.array(thetalist0)
dthetalist0 = np.transpose(np.array([0, 0, 0, 0, 0, 0]))
tau0 = np.transpose(np.array([0, 0, 0, 0, 0, 0]))
g = np.transpose(np.array([0, 0, -9.81]))
Ftip = np.transpose(np.array([0, 0, 0, 0, 0, 0]))
t = 3
N = t * 100
df = t / N
ddtheta = mr.ForwardDynamics(
    thetalist0, dthetalist0, tau0, g, Ftip, Mlist, Glist, Slist
)
dthetanew = dthetalist0 + (df * ddtheta)
thetanew = np.around(thetalist0 + (df * dthetanew), decimals=3)
thetanew = list(thetanew)
thetanew = list(thetanew)
f = open("simulation1.csv", "a")
f.write(str(thetanew)[1 : len(str(thetanew)) - 1] + "\n")
thetanew = np.array(thetanew)
for i in range(1, N - 1):  # Iteration loop
    ddthetanew = mr.ForwardDynamics(
        thetanew, dthetanew, tau0, g, Ftip, Mlist, Glist, Slist
    )
    dthetanew = dthetanew + (df * ddthetanew)
    thetanew = np.around(thetanew + (df * dthetanew), decimals=3)
    thetanew = list(thetanew)
    f = open("simulation1.csv", "a")
    f.writelines(str(thetanew)[1 : len(str(thetanew)) - 1] + "\n")
    thetanew = np.array(thetanew)
    print("\n\niteration:", i)
    print(thetanew)
