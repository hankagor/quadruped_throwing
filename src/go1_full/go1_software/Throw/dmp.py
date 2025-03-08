import numpy as np
from cs import CanonicalSystem

class DMP(object):

    def __init__(self, dmps, bfs, dt = 0.01, y0 = None, goal = None, weights = None, **kwargs):
        self.dt = dt
        self.cs = CanonicalSystem(dt = self.dt, **kwargs)
        self.timesteps = (int)(self.cs.timesteps)

        self.dmps = dmps
        self.bfs = bfs

        # Ijspeert 2013
        self.ay = 25.0 * np.ones(dmps) 
        self.by = self.ay / 4.0

        if y0 is None:
            y0 = np.zeros(self.dmps)
        self.y0 = y0
        if goal is None:
            self.goal = np.ones(self.dmps)
        self.goal = goal
        if weights is None:
            weights = np.zeros((self.dmps, self.bfs))
        # basis functions parameters
        self.weights = weights
        self.find_centers()
        self.find_widths()
    
        self.reset()

    def reset(self):
        self.y = self.y0.copy()
        self.dy = np.zeros(self.dmps)
        self.ddy = np.zeros(self.dmps)
        self.cs.reset()

    def find_centers(self):
        t = np.linspace(0, self.cs.run_time, self.bfs)
        self.centers = np.ones(self.bfs)
        self.h = np.ones(self.bfs)
        diff = 1.0
        for i in range(1, self.bfs):
            # x' = -ax * x -> x = e^(-ax * t)
            self.centers[i] = np.exp(-self.cs.ax * t[i])

        #     diff = self.centers[i] - self.centers[i - 1]
            # self.h[i - 1] = - np.log(0.6)/ (self.centers[i] - self.centers[i - 1]) ** 2.0
        # self.h[self.bfs - 1] = self.h[self.bfs - 2]

    def find_psi(self, x):
        if isinstance(x, np.ndarray):
            x = x[:, None]
        return np.exp(-self.h*(x - self.centers) ** 2)
    
    def find_widths(self):
        n = self.centers.shape[0]
        c = np.zeros(n)
        for i in range(n):
            c[i] = np.exp(-self.cs.ax * i / (n - 1))

        dc = np.diff(c)
        dc = np.append(dc, dc[-1])
        self.h = 1.0 / dc / dc * np.ones(self.bfs)

        # self.h = self.bfs ** 1.9/ self.centers / self.cs.ax * np.ones(self.bfs)
        # self.h = self.bfs ** 1.5 / (self.centers ** 1.9 )

    def find_weights(self, forcing_t_des):
        x = self.cs.run()
        psi = self.find_psi(x)
        self.weights = np.zeros((self.dmps, self.bfs))

        for i in range(self.dmps):
            val = self.goal[i] - self.y0[i]
            for j in range(self.bfs):
                a = np.sum(x * psi[:, j] * forcing_t_des[:, i])
                b = np.sum(x ** 2 * psi[:, j])
                self.weights[i, j] = a / b / val
    
    def step(self, tau = 1.0):
        x = self.cs.step(tau = tau)
        psi = self.find_psi(x)
        
        for i in range(self.dmps):
            f = x * (self.goal[i] - self.y0[i]) * (np.dot(psi, self.weights[i])) / np.sum(psi)
            self.ddy[i] = self.ay[i] * (self.by[i] * (self.goal[i] - self.y[i]) - self.dy[i]) + f
            self.dy[i] += self.ddy[i] * self.dt / tau
            self.y[i] += self.dy[i] * self.dt / tau

        return self.y, self.dy, self.ddy

    def run(self, tau = 1.0, **kwargs):
        self.reset()

        timesteps = int(self.timesteps * tau)
        y = np.zeros((timesteps, self.dmps))
        dy = np.zeros((timesteps, self.dmps))
        ddy = np.zeros((timesteps, self.dmps))

        for i in range(timesteps):
            y[i], dy[i], ddy[i] = self.step()

        return y, dy, ddy
          
    def desired_trajectory(self, y_des):

        self.y0 = y_des[:, 0].copy()
        self.goal = y_des[:, -1].copy()

        import scipy.interpolate

        path = np.zeros((self.dmps, self.timesteps))
        x = np.linspace(0, self.cs.run_time, y_des.shape[1])

        for i in range(self.dmps):
            path_gen = scipy.interpolate.interp1d(x, y_des[i])
            for t in range(self.timesteps):
                path[i, t] = path_gen(t*self.dt)

        y_des = path
        dy_des = np.gradient(y_des, axis = 1)/self.dt
        ddy_des = np.gradient(dy_des, axis = 1)/self.dt

        forcing_t_des = np.zeros((y_des.shape[1], self.dmps))
        
        for i in range(self.dmps):
            forcing_t_des[:, i] = ddy_des[i] - self.ay[i]*(self.by[i]*(self.goal[i] - y_des[i]) - dy_des[i])

        self.find_weights(forcing_t_des)

        return self.run()


# plot basis functions
# bfs = 20
# dmp = DMP(bfs = bfs, dmps = 1, run_time = 1.0)

# import matplotlib.pyplot as plt
# x = dmp.cs.run()
# psi = dmp.find_psi(x)
# t = dmp.cs.run()
# print(psi.shape)
# for i in range(bfs):
#     plt.plot(t, psi[:, i])

# plt.show()


