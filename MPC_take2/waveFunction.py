# Imports
import time
import math

import casadi as ca
#import MPC_take2.newWaveGP

class TimerError(Exception):

    """A custom exception used to report errors in use of Timer class"""


class wave(object):

    def __init__(self):

        self._start_time = None


    def start(self):

        """Start a new timer"""

        if self._start_time is not None:

            raise TimerError(f"Timer is running. Use .stop() to stop it")


        self._start_time = time.perf_counter()


    def stop(self):

        """Stop the timer, and report the elapsed time"""

        if self._start_time is None:

            raise TimerError(f"Timer is not running. Use .start() to start it")


        elapsed_time = time.perf_counter() - self._start_time

        self._start_time = None

        print(f"Elapsed time: {elapsed_time:0.4f} seconds")

    def getTime(self):
        elapsed_time = time.perf_counter() - self._start_time
        return elapsed_time

    # Returns boat goal z? **still dependant on equation
    def getCurrWave(self,x,t):
        amp = -1.0/20.0*(x-40)*math.sin(math.pi/5.0*(t+2*x))    # integral of theta
        return amp

    # for boat goal z
    def getWave(self):
        XG = ca.SX.sym("XG", 1)
        T = ca.SX.sym("T", 1)
        W = -1.0/ 20.0 * (XG - 40) * ca.sin(math.pi / 5.0 * (T + 2 * XG))   # integral of theta
        F = ca.Function('F', [XG, T], [W])
        return F


    def getCurrTheta(self,x,t):
        theta = -math.pi / 100.0 * (x - 40) * math.cos(math.pi / 5.0 * (t + 2 * x))
        absT = abs(theta)
        return absT

    def getCurrThetaGP(self,x,t,gp):
        z_pred, sigma = newWaveGP.predictOne(gp,x,t)
        absT=abs(z_pred)
        return absT
    
    def getCurrThetaVicon(self,x,t):
        theta=2.3 * (x + 3.5) * ca.sin(math.pi * (t + x))
        absT = abs(theta)
        return absT


    def getTheta(self):
        # ride the wave
        XG = ca.SX.sym("XG", 1)
        T = ca.SX.sym("T", 1)
        TH = -math.pi / 100.0 * (XG - 40) * ca.cos(math.pi / 5.0 * (T + 2 * XG))
        F = ca.Function('F', [XG, T], [TH])
        return F
    
    #indoor wave function
    def getThetaVicon(self):
        XG = ca.SX.sym("XG", 1)
        T = ca.SX.sym("T", 1)

        TH = 2.3 * (XG + 3.5) * ca.sin(math.pi * (T + XG))

        #TH = -math.pi / 100.0 * (XG - 40) * ca.cos(math.pi / 5.0 * (T + 2 * XG))
        F = ca.Function('F', [XG, T], [TH])
        return F


    def getThetaGP(self,gp):
        # ride the wave
        XG = ca.SX.sym("XG", 1)
        T = ca.SX.sym("T", 1)

        z_pred, sigma = newWaveGP.predictOne(gp, XG, T)

        F = ca.Function('F', [XG, T], [z_pred])
        return F

    def getMeanS(self):
        # calm waters
        XG = ca.SX.sym("XG", 1)
        T = ca.SX.sym("T", 1)

        #TEST = pow(-ca.pi/100*(XG-40),5)
        #LIN = -0.5*XG+100    # linear test

        period = 5
        step = period/100    # will complete 100 steps to cover one period
        theta_sum = 0

        for i in range(0,100):
            theta = -math.pi / 100.0 * (XG - 40) * ca.cos(math.pi / 5.0 * (T+(step*i) + 2 * XG))
            theta_sum += pow(theta,2)

        MS = theta_sum/100   # returns mean square of the next period starting at XG

        M = ca.Function('F', [XG, T], [MS])
        return M


    def getMeanSGP(self,gp):
        # calm waters
        XG = ca.SX.sym("XG", 1)
        T = ca.SX.sym("T", 1)

        period = 5
        step = period/100    # will complete 100 steps to cover one period
        theta_sum = 0

        for i in range(0,100):
            x_val = ca.DM([XG])
            t_val = ca.DM([T + (step * i)])
            z_pred, sigma = newWaveGP.predictOne(gp, x_val, (t_val+(step*i)))
            theta_sum += pow(z_pred,2)

        MS = theta_sum/100   # returns mean square of the next period starting at XG

        M = ca.Function('F', [XG, T], [MS])
        return M


    # for boat goal z
    def getMeanA(self):
        # calm waters
        XG = ca.SX.sym("XG", 1)
        T = ca.SX.sym("T", 1)

        getAmp = self.getWave()
        period = 5
        step = period/100    # will complete 100 steps to cover one period
        Amp_sum = 0

        for i in range(0,100):
            Amp = getAmp(XG, (T+(step*i)))
            Amp = ca.if_else(Amp<=0.0, -Amp, Amp)
            Amp_sum += Amp

        A = 2*Amp_sum/100.0

        M = ca.Function('F', [XG, T], [A])
        return M


_wave = wave()
_wave.start()


# fill=[]
# for i in range(20):
#     waveTest = _wave.getCurrThetaVicon(-3,i*0.2)
#     fill.append(waveTest)
#     print(waveTest)

# Wave = (-1/50*x+3)sin(2pi/5*x+2pi/10*t)
#     # theta = d/dt wave = -1/250*pi(x-150)cos(1/5*pi(t+2x))
#     theta = -math.pi/250.0*(x-150)*math.cos(math.pi/5.0*(t+2*x))