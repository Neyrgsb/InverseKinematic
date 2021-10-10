import numpy as np
from numpy.linalg import pinv, norm
from robo import Robot
import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt 


def plot_robot3d(robot, target):
    x_arm, y_arm, z_arm = robot.arm.get_end_pos()
    x = [robot.arm.position[0] , x_arm]
    y = [robot.arm.position[1] , y_arm]
    z = [robot.arm.position[2] , z_arm]
    #print("x e y e z", x, y, z)
    
    fig = plt.figure()
    ax = fig.gca(projection='3d')
    ax.plot(x, y, z, label='parametric curve')

    x_wrist, y_wrist, z_wrist = robot.wrist.get_end_pos()
    x = [robot.wrist.position[0] , x_wrist]
    y = [robot.wrist.position[1] , y_wrist]
    z = [robot.wrist.position[2] , z_wrist]
    print("x e y e z", x, y, z)

    ax.plot(x, y, z, label='parametric curve')

    ax.scatter(target[0], target[1], target[2])

    
    ax.legend()

    plt.show()

def main(x, y, z):
    theta = np.pi/4
    phi = np.pi/4
    psi = np.pi/4

    F = np.array([0, 0, 0])
    robot = Robot((0, 0, 0), 2, theta, psi, 1, phi) # robot initial position
    
    tolerance = 1e-5 

    delta_angle = np.array([robot.arm.rotation_xy, robot.wrist.rotation_xy, robot.arm.rotation_xz])
    
    # initial theta and phi
    y_new = np.array([0, 0, 0])

    F = np.copy(delta_angle)

    error = 10e9

    target = (x, y, z)
    plot_robot3d(robot, target)

    errors = []

    #print(robot.fx(theta, phi, psi))
    #print(robot.fy(theta, phi, psi))
    #print(robot.fz(theta, phi, psi))

    
    # Solve loop
    i=0
    #y_new = delta_angle -alpha*inv(J)*F
    while norm(error) > tolerance:
        F[0] = robot.fx(delta_angle[0], delta_angle[1], delta_angle[2])
        F[1] = robot.fy(delta_angle[0], delta_angle[1], delta_angle[2])
        F[2] = robot.fz(delta_angle[0], delta_angle[1], delta_angle[2])
        print("delta",delta_angle)
        print("F", F)


        #y_new = delta_angle - alpha * np.matmul(pinv(J), F)
        error = np.array(target) - F
        
        y_new = delta_angle + (pinv(robot.Jacobian()) @ (error))
        
        delta_angle = y_new

        robot.rotate(y_new[0] , y_new[1], y_new[2])
        
        
        errors += [norm(error)]

        #print("ynem",y_new)

        print("erro",norm(error))

        print("guia",i)
        i=i+1
        
        plot_robot3d(robot, target)
        
    print (f"target: {target}")
    print(f" func: {F}")
    print(f"pos: {robot.wrist.get_end_pos()}")
    

    plt.plot(errors)
    
    plt.xlabel('Iteração')
    plt.ylabel('Erro')
    
    plt.show()


if __name__ == "__main__":
    main(2, 0, 0)
    #nao funciona para (0,x,0)
