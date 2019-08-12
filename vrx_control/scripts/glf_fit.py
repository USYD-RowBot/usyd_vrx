import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import fmin

# Get thruster data from CSV file
#data=np.genfromtxt('kingfisher_tcurve_nohead.csv',delimiter=',')
ndata=np.genfromtxt('/home/jackson/rowbot_ws/src/USYD-RowBot/rowbot_control/scripts/thrust_vs_velocity_neg.csv',delimiter=',')
pdata=np.genfromtxt('/home/jackson/rowbot_ws/src/USYD-RowBot/rowbot_control/scripts/thrust_vs_velocity_pos.csv',delimiter=',')

# Plot
plt.ion()
plt.figure(1)
plt.clf()
plt.plot(pdata[:,0],pdata[:,1],'ro',label='pdata')
plt.plot(ndata[:,0],ndata[:,1],'bo',label='ndata')
plt.grid(True)
plt.xlabel('Command [-1.0 to 1.0]')
plt.ylabel('Measured Velocity (m/s)')




def glf(A,K,B,v,C,M,x):
    aa = C+np.exp(-B*(x-M))
    bb = np.power(aa,(1/v))
    return A+ np.divide(K-A,bb)


def glf_error(x,data):
    A=x[0]
    K=x[1]
    B=x[2]
    v=x[3]
    C=x[4]
    M=x[5]
    yy = glf(A,K,B,v,C,M,data[:,0])
    return np.sum(np.power(yy-data[:,1],2))




#o=fmin(glf,1,args=(data))
#o=fmin(glf,1,args=(data,))
'''
yy = []
for ii in range(data.shape[0]):
    yy.append = glf(1,1,1,1,1,1,data[ii,0])

plot(data[:,0],yy,'r')
'''
# Iteratively develop an initial guess
yy = glf(0,250,10,1,1,0.5,pdata[:,0])
#plot(pdata[:,0],yy,'r',label='Pos. init')

# Fine tune with fmin
outp =fmin(glf_error,[0,250,10,1,1,0.5],args=(pdata,))

zz = glf(outp[0],outp[1],outp[2],outp[3],outp[4],outp[5],pdata[:,0])
plt.plot(pdata[:,0],zz,'g',linewidth=2,label='GLF Fit +')#'Pos. final')

# Iteratively develop an initial guess
yy = glf(-100,0,10,1,1,-0.7,ndata[:,0])
#plot(ndata[:,0],yy,'b',label='Neg. init')

# Fine tune with fmin
outn =fmin(glf_error,[-100,0,10,1,1,-0.7],args=(ndata,))

zz = glf(outn[0],outn[1],outn[2],outn[3],outn[4],outn[5],ndata[:,0])
plt.plot(ndata[:,0],zz,'g',linewidth=2,label='GLF Fit -')#'Neg. final')

plt.legend(loc='upper left')

txtstr='GLF+:A=%.2f,K=%.2f,B=%.2f,\n   v=%.2f,C=%.2f,M=%.2f\nGLF-:A=%.2f,K=%.2f,B=%.2f,\n   v=%.2f,C=%.2f,M=%.2f'%tuple(np.hstack((outp,outn)))
props = dict(boxstyle='round', facecolor='wheat', alpha=0.5)

plt.text(0.05,0.7,txtstr,fontsize=14,transform=plt.gca().transAxes,verticalalignment='top',bbox=props)

plt.savefig('thrust_v_vel.png')
plt.show()
