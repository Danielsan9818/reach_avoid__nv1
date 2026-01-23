import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
from mpl_toolkits.mplot3d import Axes3D
from optimal_control import Optimal_Control


fig = plt.figure()
ax = fig.add_subplot(111,projection='3d')


# z positive for real experiment
# x,y,z lim [-250,250],[-150,150],[30,150]
pos_pursuer = 1e-2*np.array([[0,0,20],[10,10,40],[-50,10,60],[-140,-20,30],[150,3,70]],dtype=float) 
number_pursuers = len(pos_pursuer)
pos_evader = 1e-2*np.array([0,-100,150],dtype=float)

which_area = 2
# ellipsoid -> 1
# n-ball -> 2
# elliptic paraboloid -> 3

if which_area==1:

    center = np.array([0, 0, 0])  # x0, y0, z0
    a, b, c = 0.8, 0.8, 0.2                   # ellipse axes lengths
    par_ellipsoide = np.array([a,b,c])
    # theta = np.radians(0)        # rotation angle around Z axis
    u = np.linspace(0, 2 * np.pi, 100)
    v = np.linspace(0, np.pi, 50)

    x =  np.outer(np.cos(u), np.sin(v))
    y = b * np.outer(np.sin(u), np.sin(v))
    z = c * np.outer(np.ones_like(u), np.cos(v))

elif (which_area==2):

    center = np.array([0, 0, 0])  # x0, y0, z0
    a,b,c,p = 0.8,0.1,0.1,3 # scale x,y,x and exponential     p>=2
    par_ellipsoide = np.array([a,b,c,p])                   # lp- ball 
    # par_ellipsoide = np.array([a,b,c])

    # theta = np.radians(0)        # rotation angle around Z axis
    u = np.linspace(0, 2*np.pi, 100)
    v = np.linspace(-np.pi/2, np.pi/2, 50)

    U, V = np.meshgrid(u, v)

    # Parametric formula for Lp-ball surface (smooth convex bodies)
    x =  a * np.sign(np.cos(V) * np.cos(U)) * np.abs(np.cos(V) * np.cos(U))**(1/par_ellipsoide[3])
    y =  b * np.sign(np.cos(V) * np.sin(U)) * np.abs(np.cos(V) * np.sin(U))**(1/par_ellipsoide[3])
    z =  c * np.sign(np.sin(V)) * np.abs(np.sin(V))**(1/par_ellipsoide[3])

else:

    center = np.array([0, 0, 0])  # x0, y0, z0
    a, b, c = 8, 8, 2                   # ellipse axes lengths
    par_ellipsoide = np.array([a,b,c])
    # theta = np.radians(0)        # rotation angle around Z axis
    u = np.linspace(0, 2 * np.pi, 100)
    v = np.linspace(0, np.pi, 50)

    x = a * np.outer(np.cos(u), np.sin(v))
    y = b * np.outer(np.sin(u), np.sin(v))
    z = c * np.outer(np.ones_like(u), np.cos(v))

# Apply rotation around Z-axis
# x_rot = x * np.cos(theta) - y * np.sin(theta) + center[0]
# y_rot = x * np.sin(theta) + y * np.cos(theta) + center[1]
# z_rot = z + center[2]

ax.plot_surface(x, y, z, color='m', alpha=0.2, label='Target Area')
# plt.show()


t=0.0
dt=0.1
# r = np.array([1,1.1,0.6,0.9,0.1])
# alpha = np.array([1.1,1.2,1.7,1.9,1.1]) ## greater than 1

evader_speed = 1e-2*np.array([19])
pursuers_speed = 1e-2*np.array([20,40,30,20,20])
# alpha = pursuers_speed/evader_speed
r = 1e-2*np.array([30,15,20,50,25])
#min 5cm/s max 100cm/s
#ruido posicao 0.1
#ruido velocidade metade da velocidade
failure_rate = 0.0



traj_pursuer = [[pos_pursuer[i]] for i in range(number_pursuers)]  # list of lists
traj_evader = [pos_evader]
mode=3
x0=1e-2*np.array([20.1, 50.1 , 100.0])


ax.plot(pos_pursuer[:,0],pos_pursuer[:,1],pos_pursuer[:,2], label='Pursuer start', color='b',marker="*",linestyle='')
ax.plot(pos_evader[0],pos_evader[1],pos_evader[2], label='Evader start', color='r',marker="*")
# ax.plot(opt_tra[:,0],opt_tra[:,1],opt_tra[:,2], label='Optimal point start', color='k',marker="*")
failure_flag = np.zeros(number_pursuers, dtype=bool)
flag_fail = 0

if which_area==1:
    gameValue = (pos_evader[0]**2) / par_ellipsoide[0]**2 + (pos_evader[1]**2) / par_ellipsoide[1]**2 + (pos_evader[2]**2) / par_ellipsoide[2]**2 - 1
elif (which_area==2):
    gameValue = (np.abs(pos_evader[0] / par_ellipsoide[0]))**par_ellipsoide[3] + (np.abs(pos_evader[1] / par_ellipsoide[1]))**par_ellipsoide[3]  + (np.abs(pos_evader[2] / par_ellipsoide[2]))**par_ellipsoide[3]  - 1
else:
    gameValue = (pos_evader[0]**2) / par_ellipsoide[0]**2 + (pos_evader[1]**2) / par_ellipsoide[1]**2 + (pos_evader[2]**2) / par_ellipsoide[2]**2 - 1

while (not any(np.linalg.norm(pos_pursuer-pos_evader,axis=1)<r)) and gameValue>1:
    
    ## debug
    if 2-t<0.1:
        debug_aux=1
    ## fixed failures
    if t>2 and t<2.1:
        failure_flag[1] = True
        ax.plot(pos_evader[0],pos_evader[1],pos_evader[2], label='Evader fail time', color='r',marker="x")
    ## failure
    failure_flag |= (np.random.rand(number_pursuers)>(1-failure_rate))
    pursuers_speed[failure_flag] = 0
    r[failure_flag] = 0
    if all (pursuers_speed==0):
        print("fail")
        flag_fail = 1
        # break
    ## noise
    noisy_speedp = np.copy(pursuers_speed) #+ np.random.uniform(-pursuers_speed/4,pursuers_speed/4,number_pursuers)
    noisy_speede = np.copy(evader_speed) #+ np.random.uniform(-evader_speed/4,evader_speed/4,1)

    ## control
    if flag_fail:
        vel_evader = dt*evader_speed*(center-pos_evader)/(np.linalg.norm(center-pos_evader))
        vel_pursuer = np.zeros(pos_pursuer.shape)
    else:
        vel_pursuer,vel_evader,x0 = Optimal_Control(pos_pursuer,pos_evader,r,pursuers_speed,evader_speed,mode,dt,x0,noisy_speedp,noisy_speede,par_ellipsoide,which_area)
    
    pos_pursuer = pos_pursuer + vel_pursuer
    pos_evader = pos_evader + vel_evader

    if which_area==1:

        gameValue = (pos_evader[0]**2) / par_ellipsoide[0]**2 + (pos_evader[1]**2) / par_ellipsoide[1]**2 + (pos_evader[2]**2) / par_ellipsoide[2]**2 - 1
    
    elif (which_area==2):

        gameValue = (np.abs(pos_evader[0] / par_ellipsoide[0]))**par_ellipsoide[3] + (np.abs(pos_evader[1] / par_ellipsoide[1]))**par_ellipsoide[3]  + (np.abs(pos_evader[2] / par_ellipsoide[2]))**par_ellipsoide[3]  - 1
    
    else:
        
        gameValue = (pos_evader[0]**2) / par_ellipsoide[0]**2 + (pos_evader[1]**2) / par_ellipsoide[1]**2 + (pos_evader[2]**2) / par_ellipsoide[2]**2 - 1
    

    t+=dt


    for i in range(number_pursuers):
        traj_pursuer[i].append(pos_pursuer[i].copy())

    traj_evader.append(pos_evader.copy())

traj_pursuer = [np.array(tp) for tp in traj_pursuer]
traj_evader = np.array(traj_evader)
for i in range(number_pursuers):
    ax.plot(traj_pursuer[i][:,0],
            traj_pursuer[i][:,1],
            traj_pursuer[i][:,2],
            label=f'Pursuer {i+1}')

# Plot evader
ax.plot(traj_evader[:,0],
        traj_evader[:,1],
        traj_evader[:,2],
        label='Evader')


ax.legend()
plt.show()
print("done")
print("Game Value:", gameValue)
print("Time:", t)
