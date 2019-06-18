from numpy import * 
from matplotlib.pyplot import *

class RecordedPath():
    def __init__(self,fname,dt):

        self.xnow = 0.
        self.ynow = 0.
        self.znow = 0.
        self.pitchnow =0.
        self.yawnow = 0.
        #data should come in as t,x,y,z
        self.data = loadtxt(fname)
        self.tdata = self.data[:,0]
        self.xdata = self.data[:,1]
        self.ydata = self.data[:,2]
        self.zdata = self.data[:,3]
        #calculate pitch data from z vs. planar velocity
        self.pitchdata = arctan2(diff(self.zdata),sqrt(diff(self.xdata)**2+diff(self.ydata)**2))
        self.pitchdata = append(array([self.pitchdata[0]]),self.pitchdata)
        #constrain the pitch data so that the fish never goes below 0 or above a threhold
        self.pitchdata[(self.pitchdata<0)]=0
        self.pitchdata[(self.pitchdata>pi/4.)]=pi/4.
        #now calculate the yaw angle of the fish. There will be "laps" we'll have to account for.
        self.yawraw = arctan2(diff(self.ydata),diff(self.xdata))
        self.yawraw = append(array(self.yawraw[0]),self.yawraw)
        laps = 0
        self.yawdata = zeros(len(self.yawraw))
        self.yawdata[0] = self.yawraw[0]

        for k in range(1,len(self.yawraw)):
            if(abs(self.yawraw[k]-self.yawraw[k-1])>=pi):
                laps += sign(self.yawraw[k]-self.yawraw[k-1])
            self.yawdata[k] = laps*2*pi+self.yawraw[k]

        #zero out the time just in case it does not start at zero
        self.tdata = self.tdata-self.tdata[0]
        self.tnow = 0.
        self.U = sqrt((diff(self.xdata)/diff(self.tdata))**2+(diff(self.ydata)/diff(self.tdata))**2)
        self.U = append(array(self.U[0]),self.U)
        self.maxspeed = max(self.U)

        self.Udot = diff(self.U)/diff(self.tdata)
        self.Udot = append(array([self.Udot[0]]),self.Udot)


    def update(self,dt):
        self.tnow +=dt
        if(self.tnow<=self.tdata[-1]):
            self.xnow = interp(self.tnow,self.tdata,self.xdata)
            self.ynow = interp(self.tnow,self.tdata,self.ydata)
            self.znow = interp(self.tnow,self.tdata,self.zdata)
            self.pitchnow = interp(self.tnow,self.tdata,self.pitchdata)
            self.yawnow = interp(self.tnow,self.tdata,self.yawdata)
            self.Unow = interp(self.tnow,self.tdata,self.U)
            self.Udotnow = interp(self.tnow,self.tdata,self.Udot)
        return self.xnow,self.ynow,self.znow,self.pitchnow,self.yawnow,self.Unow,self.Udotnow


def demo():
    fname = 'out3d.txt'

    RP = RecordedPath(fname,.01)

    t = arange(0,500.,0.1)
    newdata = zeros((len(t),7))
    for k in range(0,len(t)):
        newdata[k,:] = RP.update(.1)


    figure()
    plot(t,newdata[:,6])
    figure()
    plot(newdata[:,0],newdata[:,1])
    axis('equal')
    show()


if __name__=='__main__':
    demo()
