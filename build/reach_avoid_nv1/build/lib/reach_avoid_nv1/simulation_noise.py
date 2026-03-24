import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
from mpl_toolkits.mplot3d import Axes3D
from optimal_control_noise import Optimal_Control_noise
from optimal_control import Optimal_Control
# from estimation import estimate_evader_speed

column_width = 3.45         # inches (adjust for your journal)
panel_height = 2.5          # inches per panel (adjust as needed)
font_size = 10

plt.rcParams.update({'font.size': font_size, 'lines.linewidth': 1})

# fig, ax = plt.subplots(2, 1, figsize=(column_width, 2*panel_height), constrained_layout=True,projection='3d')


fig = plt.figure()
ax = fig.add_subplot(111,projection='3d')


# z positive for real experiment
# x,y,z lim [-250,250],[-150,150],[30,150]
# pos_pursuer = 1e-2*np.array([[0,0,20],[10,10,40],[-50,10,60],[-140,-20,30],[150,0,70]],dtype=float)
# pos_pursuer = np.array([[0,0,3],[1,0.5,2.1],[-1,-2,2.3],[1,1.8,4],[-1,-1,2.5]],dtype=float)

pos_pursuer = np.array([[-0.47806612, -0.02226625, 0.00878122]],dtype=float)
## paper
# pos_pursuer = np.array([[3,0,3],[-5,4,4],[-2,-5,1]],dtype=float)
number_pursuers = len(pos_pursuer)
# pos_evader = 1e-2*np.array([0,-100,150],dtype=float)
## paper
pos_evader = np.array([-1.89408696,  0.00735356,  0.00573833],dtype=float)

which_area = 1
# ellipsoid -> 1
# n-ball -> 2
# elliptic paraboloid -> 3

if which_area==1:

    center = np.array([0, 0, 0])  # x0, y0, z0
    a, b, c = 0.08, 0.08, 0.02                  # ellipse axes lengths
    par_ellipsoide = np.array([a,b,c])
    # theta = np.radians(0)        # rotation angle around Z axis
    u = np.linspace(0, 2 * np.pi, 100)
    v = np.linspace(0, np.pi, 50)

    x =  np.outer(np.cos(u), np.sin(v))
    y = b * np.outer(np.sin(u), np.sin(v))
    z = c * np.outer(np.ones_like(u), np.cos(v))

elif (which_area==2):

    center = np.array([0, 0, 0])  # x0, y0, z0
    a,b,c,p = 1.3,0.8,0.5,3 # scale x,y,x and exponential     p>=2
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


t=10.0
dt=0.1
# r = np.array([1,1.1,0.6,0.9,0.1])
# alpha = np.array([1.1,1.2,1.7,1.9,1.1]) ## greater than 1

# evader_speed = 1e-2*np.array([19])
# pursuers_speed = 1e-2*np.array([20,40,30,20,20])

## paper

evader_speed = 1e-2*np.array([5])
pursuers_speed = 1e-2*np.array([15]) #np.array([2,2.1,2.4])
# alpha = pursuers_speed/evader_speed
# r = 1e-2*np.array([30,15,20,50,25])
## paper
r = np.array([0.5])
#min 5cm/s max 100cm/s
#ruido posicao 0.1
#ruido velocidade metade da velocidade
failure_rate = 0.0

initial_distance = np.min(np.linalg.norm(pos_pursuer - pos_evader, axis=1))

traj_pursuer = [[pos_pursuer[i]] for i in range(number_pursuers)]  # list of lists
traj_evader = [pos_evader]
mode=1
evader_mode = 1
# x0=np.array([-1.6, 0.1 , 0.1])
x0 = np.copy(pos_evader) - 0.1*np.copy(pos_evader)
x0_eva=np.copy(x0)
speed_history = []

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


    # theta = 0.05*np.min(np.linalg.norm(pos_pursuer-pos_evader,axis=1)) # std deviation for noise
    theta = np.copy(evader_speed)/10 # std deviation for noise
    ## noise
    noisy_speedp = np.copy(pursuers_speed) #+ np.random.uniform(-pursuers_speed/4,pursuers_speed/4,number_pursuers)
    noisy_speede = np.copy(evader_speed) #+ np.random.normal(0,theta/2,size=evader_speed.shape)
    
    measurad_evader_speed = np.copy(noisy_speede) + np.random.normal(0,theta,size=evader_speed.shape)

    # estimated_evader_speed, speed_history = estimate_evader_speed(measurad_evader_speed,speed_history)
    # estimated_evader_speed=measurad_evader_speed
    estimated_evader_speed = 0.99*np.copy(pursuers_speed)
    ## failure
    failure_flag |= (np.random.rand(number_pursuers)>(1-failure_rate))
    pursuers_speed[failure_flag] = 0
    r[failure_flag] = 0
    if all (pursuers_speed==0):
        print("fail")
        flag_fail = 1
        # break


    ## control
    if flag_fail:
        vel_evader = dt*evader_speed*(center-pos_evader)/(np.linalg.norm(center-pos_evader))
        vel_pursuer = np.zeros(pos_pursuer.shape)
    else:
        vel_pursuer_real,vel_evader,x0_eva,flag_error1 = Optimal_Control(pos_pursuer,pos_evader,r,pursuers_speed,evader_speed,mode,dt,x0_eva,noisy_speedp,noisy_speede,par_ellipsoide,which_area,evader_mode)
        vel_pursuer,vel_evader_noisy,x0,flag_error2 = Optimal_Control_noise(pos_pursuer,pos_evader,r,pursuers_speed,evader_speed,mode,dt,x0,noisy_speedp,noisy_speede,par_ellipsoide,which_area,evader_mode,estimated_evader_speed)
    
    pos_pursuer = pos_pursuer + vel_pursuer
    pos_evader = pos_evader + vel_evader

    ##debug
    # print("Distance toptimal point:",np.linalg.norm(x0_eva-x0))
    # print("Difference vel evader:",vel_evader_noisy-vel_evader)
    # print("Difference pursuer vel:",vel_pursuer_real-vel_pursuer)

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
        label='Evader',
        color='r')


u = np.linspace(0, 2 * np.pi, 20)
v = np.linspace(0, np.pi, 20)

U, V = np.meshgrid(u, v)

# unit sphere parameterization (radius=1)
x_ = np.cos(U) * np.sin(V)
y_ = np.sin(U) * np.sin(V)
z_ = np.cos(V)

# for i in range(number_pursuers):
#     # scale then translate so the sphere is centered at pos_pursuer[i]
#     x_s = r[i] * x_ + pos_pursuer[i, 0]
#     y_s = r[i] * y_ + pos_pursuer[i, 1]
#     z_s = r[i] * z_ + pos_pursuer[i, 2]
#     lbl = 'Pursuer capture zone' if i == 0 else '_nolegend_'
#     ax.plot_surface(x_s, y_s, z_s, color='b', alpha=0.1, linewidth=0, label=lbl)

# for i in range(number_pursuers):
#     ax.plot([traj_pursuer[i][-1,0]], [traj_pursuer[i][-1,1]], [traj_pursuer[i][-1,2]], marker='X', color='b', markersize=8, label=f'Pursuer {i+1} End' if i==0 else "")
# ax.plot([traj_evader[-1,0]], [traj_evader[-1,1]], [traj_evader[-1,2]], marker='X', color='r', markersize=8, label='Evader End')



# ax.legend()
font_size = 12
ax.tick_params(labelsize=font_size)
# set axis label sizes (if any)
ax.xaxis.label.set_size(font_size)
ax.yaxis.label.set_size(font_size)
ax.zaxis.label.set_size(font_size)
ax.legend(fontsize=font_size,bbox_to_anchor=(1.25, 1), loc='upper left')
plt.show()
print("done")
print("Game Value:", gameValue)
print("Time:", t)

plt.savefig('3v1_3D.png', dpi=600,bbox_inches='tight')