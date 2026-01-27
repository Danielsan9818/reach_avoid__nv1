import numpy as np
from scipy.optimize import fsolve, minimize

def convex_area_3d_1v1(x, par,par_ellipsoide,which_area):
    # x[0] = x*
    # x[1] = y*
    # x[2] = z*
    #
    # par[0] = [xd, yd, zd]
    # par[1] = [xi, yi, zi]
    # par[2] = alpha
    # par[3] = r

    p1 = np.array(par[0])  # pursuer position
    p2 = np.array(par[1])  # evader position
    alpha = par[2]
    r = par[3]

    x_vec = np.array(x)

    # Norm vectors
    n1 = np.linalg.norm(x_vec - p1)
    n2 = np.linalg.norm(x_vec - p2)

    # Components
    num1 = (x_vec[0] - p1[0]) / n1 - alpha * ((x_vec[0] - p2[0]) / n2)
    den1 = (x_vec[2] - p1[2]) / n1 - alpha * ((x_vec[2] - p2[2]) / n2)

    num2 = (x_vec[1] - p1[1]) / n1 - alpha * ((x_vec[1] - p2[1]) / n2)
    den2 = den1  # same denominator

    F = np.zeros(3)

    if which_area==1:

        F[0] = (x_vec[0] / ((par_ellipsoide[0]**2)/2)) - (x_vec[2] / ((par_ellipsoide[2]**2)/2)) * (num1 / den1)
        F[1] = (x_vec[1] / ((par_ellipsoide[1]**2)/2)) - (x_vec[2] / ((par_ellipsoide[2]**2)/2)) * (num2 / den2)
        F[2] = n1 - alpha * n2 - r

    elif (which_area==2):
        aux1 = (par_ellipsoide[3]*x_vec[0]*(np.abs(x_vec[0]/par_ellipsoide[0])**(par_ellipsoide[3]-2)))/(par_ellipsoide[0]**2)
        aux2 = (par_ellipsoide[3]*x_vec[1]*(np.abs(x_vec[1]/par_ellipsoide[1])**(par_ellipsoide[3]-2)))/(par_ellipsoide[1]**2)
        aux3 = (par_ellipsoide[3]*x_vec[2]*(np.abs(x_vec[2]/par_ellipsoide[2])**(par_ellipsoide[3]-2)))/(par_ellipsoide[2]**2)


        F[0] = aux1 - aux3 * (num1 / den1)
        F[1] = aux2 - aux3 * (num2 / den2)
        F[2] = n1 - alpha * n2 - r
        
    else:
        F = np.zeros(3)


    return F

def convex_area_3d_2v1(x1, par,par_ellipsoide,which_area):

    # Unpack parameters for readability
    p1 = np.array(par[0]) # pursuer 1
    p2 = np.array(par[1]) # pursuer 2
    p3 = np.array(par[2]) # evader
    a1 = par[3]
    a2 = par[4]
    r1 = par[5]
    r2 = par[6]

    # Objective function
    def objective(x):
        if which_area==1:
            func = (x[0]**2)/(par_ellipsoide[0])**2 + (x[1]**2)/(par_ellipsoide[1])**2 + (x[2]**2)/(par_ellipsoide[2])**2 - 1
        elif (which_area==2):
            func = (np.abs(x[0] / par_ellipsoide[0]))**par_ellipsoide[3] + (np.abs(x[1] / par_ellipsoide[1]))**par_ellipsoide[3]  + (np.abs(x[2] / par_ellipsoide[2]))**par_ellipsoide[3]  - 1
        else:
            func=0
        return func

    # Constraint 1
    def constraint1(x):
        return np.linalg.norm(x - p1) - a1*np.linalg.norm(x - p3) - r1

    # Constraint 2
    def constraint2(x):
        return np.linalg.norm(x - p2) - a2*np.linalg.norm(x - p3) - r2

    constraints = [
        {"type": "eq", "fun": constraint1},
        {"type": "eq", "fun": constraint2},
    ]

    # Solve
    sol = minimize(
        objective,
        x1,
        constraints=constraints,
        method='SLSQP',
        options={"disp": False}
    )

    if not sol.success:
        return np.array([np.nan, np.nan, np.nan])

    return sol.x


def convex_area_3d_3v1(x, par,par_ellipsoide,which_area):
    """
    Python equivalent of the MATLAB convex_area_3d_complete_3pur function.

    x: [x*, y*, z*]
    par = [
        p1, p2, p3,     # target positions (xd1, xd2, xd3)
        pi,             # pursuer position (xi)
        alpha1, alpha2, alpha3,
        r1, r2, r3
    ]
    """

    p1 = np.array(par[0])
    p2 = np.array(par[1])
    p3 = np.array(par[2])
    pi = np.array(par[3])

    alpha1 = par[4]
    alpha2 = par[5]
    alpha3 = par[6]

    r1 = par[7]
    r2 = par[8]
    r3 = par[9]

    x_vec = np.array(x)

    F = np.zeros(3)

    F[0] = np.linalg.norm(x_vec - p1) - alpha1 * np.linalg.norm(x_vec - pi) - r1
    F[1] = np.linalg.norm(x_vec - p2) - alpha2 * np.linalg.norm(x_vec - pi) - r2
    F[2] = np.linalg.norm(x_vec - p3) - alpha3 * np.linalg.norm(x_vec - pi) - r3

    return F

def oneVone(pos_pursuer,pos_evader,r,alpha,x0,par_ellipsoide,which_area):
    par = [
        pos_pursuer,
        pos_evader,
        alpha,
        r
    ]

    opt,_,ier,message = fsolve(convex_area_3d_1v1, x0, args=(par,par_ellipsoide,which_area,),full_output=True)
    # print("Solution:", opt)##
    if ier !=1:
        # print("error:",message)
        opt = np.full_like(x0,np.nan)
    

    return opt


def twoVone(p1,p2,pos_evader,r1,r2,a1,a2,x0,par_ellipsoide,which_area):
    par = [p1, p2, pos_evader, a1, a2, r1, r2]

    # x0 = [0.1, 0.1, 0.1]

    opt = convex_area_3d_2v1(x0, par,par_ellipsoide,which_area)
    # print("Solution:", opt)
    return opt

def threeVone(p1,p2,p3,pos_evader,r1,r2,r3,a1,a2,a3,x0,par_ellipsoide,which_area):
    par = [
        p1,
        p2,
        p3,
        pos_evader,
        a1,a2,a3,
        r1,r2,r3    
    ]

    opt,_,ier,_ = fsolve(convex_area_3d_3v1, x0, args=(par,par_ellipsoide,which_area,),full_output=True)
    if ier !=1:
        opt = np.full_like(x0,np.nan)

    return opt

def global_optima(opt_pur1,opt_pur2,opt_pur3,pos_d,pos_i,alpha,r,aux_pur2,aux_pur3,par_ellipsoide,which_area):
    
    x = np.vstack((opt_pur1,opt_pur2,opt_pur3))

    x = np.array(x, dtype=float)
    pos_d = np.array(pos_d)
    pos_i = np.array(pos_i)
    alpha = np.array(alpha)
    r = np.array(r)

    x_new = np.copy(x)

    for i in range(x.shape[0]):
        # vecnorm(x(i,:) - pos_d)
        d1 = np.linalg.norm(x[i] - pos_d,axis=1)

        # alpha .* norm(x(i,:) - pos_i)  → elementwise
        d2 = alpha * np.linalg.norm(x[i] - pos_i)

        aux_value = d1 - (d2 + r)

        # If ANY element is < -1e-5
        if np.any(aux_value < -1e-7):
            x_new[i] = np.array([np.nan, np.nan, np.nan])

    if which_area==1:
        value = (x_new[:, 0]**2) / (par_ellipsoide[0])**2 + (x_new[:, 1]**2) / (par_ellipsoide[1])**2 + (x_new[:, 2]**2) / (par_ellipsoide[2])**2 - 1
    elif (which_area==2):
        value = (np.abs(x_new[:,0] / par_ellipsoide[0]))**par_ellipsoide[3] + (np.abs(x_new[:,1] / par_ellipsoide[1]))**par_ellipsoide[3]  + (np.abs(x_new[:,2] / par_ellipsoide[2]))**par_ellipsoide[3]  - 1
    else:
        value = (x_new[:, 0]**2) / (par_ellipsoide[0])**2 + (x_new[:, 1]**2) / (par_ellipsoide[1])**2 + (x_new[:, 2]**2) / (par_ellipsoide[2])**2 - 1
    
    if all(np.isnan(value)):
        Value_func = np.nan
        pur = np.nan
        optimal_point = np.array([0.0,0.0,0.0])
        return optimal_point,pur,1
    else:
        Value_func = np.nanmin(value)
        pur = np.nanargmin(value)
        optimal_point = x_new[pur]

    return optimal_point,pur,0

def find_active_pursuer(pur,opt_pur1,opt_pur2,aux_pur2,aux_pur3,mode,optimal_point,pos_pursuer):
    
    if pur<len(opt_pur1):
        active_pursuer = [pur]
    elif pur<len(opt_pur1)+len(opt_pur2):
        active_pursuer = aux_pur2[pur-len(opt_pur1)]
    else:
        active_pursuer = aux_pur3[pur-len(opt_pur1)-len(opt_pur2)]

    if mode==1:
        ##active pursuer -> opt point everyone else so not move
        target = pos_pursuer.copy()
        target[active_pursuer,:] = optimal_point
    elif mode==2:
        ##everyone -> opt point
        target = np.tile(optimal_point, (pos_pursuer.shape[0], 1))
    else:
        # active puruser -> opt point and everyone else to their opt point
        target = opt_pur1.copy()
        target[active_pursuer,:] = optimal_point

    return target

def Optimal_Control(pos_pursuer,pos_evader,r,pursuers_speed,evader_speed,mode,dt,x0,noisy_speedp,noisy_speede,par_ellipsoide,which_area):
    n_pur = len(pos_pursuer)
    # x0 = [0.1, 0.1, 0.1]  # initial guess
    alpha = pursuers_speed/evader_speed

    opt_pur1 = np.zeros((n_pur,3))
    for i in range(n_pur):
        if alpha[i]==0:
            opt_pur1[i]=np.array([np.nan,np.nan,np.nan])
        else:
            opt_pur1[i]=oneVone(pos_pursuer[i],pos_evader,r[i],alpha[i],x0,par_ellipsoide,which_area)
    if n_pur<2:
        opt_pur2 = np.array([np.nan,np.nan,np.nan])
        aux_act_pur2 = np.array([np.nan,np.nan])
    else:
        opt_pur2 = []
        aux_act_pur2=[]
        for i in range(n_pur-1):
            for j in range(i+1,n_pur):
                if alpha[i]==0 or alpha[j]==0:
                    aux = np.array([np.nan,np.nan,np.nan])
                else:
                    aux = twoVone(pos_pursuer[i],pos_pursuer[j],pos_evader,r[i],r[j],alpha[i],alpha[j],x0,par_ellipsoide,which_area)
                opt_pur2.append(aux)
                aux_act_pur2.append([i,j])

    if n_pur<3:
        opt_pur3 = np.array([np.nan,np.nan,np.nan])
        aux_act_pur3 = np.array([np.nan,np.nan,np.nan])
    else:
        opt_pur3=[]
        aux_act_pur3=[]
        for i in range(n_pur-2):
            for j in range(i+1,n_pur-1):
                for k in range(j+1,n_pur):
                    if alpha[i]==0 or alpha[j]==0 or alpha[k]==0:
                        aux = np.array([np.nan,np.nan,np.nan])
                    else:
                        aux = threeVone(pos_pursuer[i],pos_pursuer[j],pos_pursuer[k],pos_evader,r[i],r[j],r[k],alpha[i],alpha[j],alpha[k],x0,par_ellipsoide,which_area)
                    opt_pur3.append(aux)
                    aux_act_pur3.append([i,j,k])

    optimal_point,pur,flag_nan = global_optima(opt_pur1,opt_pur2,opt_pur3,pos_pursuer,pos_evader,alpha,r,aux_act_pur2,aux_act_pur3,par_ellipsoide,which_area)

    if flag_nan:
        center = np.array([0, 0, 0])  # x0, y0, z0
        vel_evader = dt*evader_speed*(center-pos_evader)/(np.linalg.norm(center-pos_evader))
        vel_pursuer = np.zeros(pos_pursuer.shape)
        x0 = center
        return[vel_pursuer,vel_evader,x0,flag_nan]
    target = find_active_pursuer(pur,opt_pur1,opt_pur2,aux_act_pur2,aux_act_pur3,mode,optimal_point,pos_pursuer)

    aux_norm = np.linalg.norm(target-pos_pursuer,axis=1,keepdims=True)
    aux_norm[aux_norm==0] = 1e-8
    vel_pursuer = (dt*noisy_speedp.reshape(-1,1)*(target-pos_pursuer)/aux_norm)
    vel_evader = dt*noisy_speede*(optimal_point-pos_evader)/np.linalg.norm(optimal_point-pos_evader)
    x0 = optimal_point
    vel_pursuer = vel_pursuer.reshape((n_pur,3))
    ##
    # vel_evader = np.array([1,0,0])
    # vel_pursuer = np.zeros((n_pur,3))
    # for i in range(n_pur):
    #     vel_pursuer[i] = [0,0,1]
    return[vel_pursuer,vel_evader,x0,flag_nan]