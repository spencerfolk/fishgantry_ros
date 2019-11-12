from numpy import *
from matplotlib.pyplot import *

data = loadtxt('robotmotion.txt')
savetxt('robotmotion_backup.txt',data,delimiter='\t')

# data[:,5] = data[:,5]+pi
data[:,3] = 0

figure()
plot(data[:,0],data[:,5])

savetxt('robotmotion.txt',data,delimiter='\t')

show()