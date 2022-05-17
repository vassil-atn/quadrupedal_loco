# This is a Python-based optimisation-based IK solver.
import pinocchio as pin
from pinocchio.utils import *
from os.path import dirname, join, abspath
from pinocchio.visualize import GepettoVisualizer
from pinocchio.robot_wrapper import RobotWrapper
import time
import scipy.optimize
from scipy.optimize import fmin_bfgs, fmin_slsqp
from scipy.optimize import minimize
from numpy.linalg import pinv,solve
from matplotlib import pyplot as plt
import numpy as np
import rospy
from go1_rt_control.srv import IKsolver, IKsolverResponse



def createRobotModel():
    pinocchio_model_dir = join("/home/vassil/TU_Delft/Internship/catkin_ws/src", "quadrupedal_loco/unitree_ros")
    model_path = join(pinocchio_model_dir, "robots")

    robotName = "go1" # change to a1 for the A1 robot

    mesh_dir = model_path
    urdf_filename = "go1.urdf"
    urdf_model_path = join(join(model_path, "go1_description/urdf"), urdf_filename)

    robot = RobotWrapper.BuildFromURDF(urdf_model_path, mesh_dir)
    
    return robot

def handle_IKsolver(req):
    Mgoal = pin.SE3(np.eye(3), np.matrix([req.desired_pos[0], req.desired_pos[1], req.desired_pos[2]]).T) # Desired goal
    robot = createRobotModel()
    q = np.zeros(12)
    q[req.feet_flag:req.feet_flag+3] = np.array([req.joint_angles])
    q = fbgs_opt(q,robot,Mgoal)
    q = q.x[req.feet_flag:req.feet_flag+3]

    return IKsolverResponse(q)

def IKsolver_server():
    rospy.init_node('IKsolver_server')
    s = rospy.Service('IKSolver', IKsolver, handle_IKsolver)
    print("Ready to perform IK")
    rospy.spin()


def positionError(q,qprev,Mgoal,robot):
    #des = pin.log(Mgoal).vector
    des = Mgoal.translation
    pin.forwardKinematics(robot.model, robot.data, q)  # Compute joint placements
    pin.updateFramePlacements(robot.model, robot.data)      # Also compute operational frame placements
    Mtool = robot.data.oMf[IDX_TOOL]  # Get placement from world frame o to frame f oMf
    cur = Mtool.translation
    #err = pin.log(Mtool.inverse() * Mgoal).vector # This is one way of computing an error in SO3
    err = np.linalg.norm(des-cur) # Calculate norm of end-effector position error
    qvel = np.abs(q-qprev)%(2*np.pi)

    qvel = np.linalg.norm(qvel) # Calculate norm of joint "velocity" (i.e. change of joint angle)

    # Define error cost Q and joint velocity cost Qj
    Q = 10
    Qj = 0.01
    cost = err*Q*err #+ qvel*Qj*qvel
    

    return cost

def constraintFun_vel(q,q1,qprev):
    qvel = np.abs(q-qprev)%(2*np.pi)/0.01
    
    # if np.any(q < -2):
    #     print('error1')
    #     print(qvel)
    #     time.sleep(5)
    # if np.any(qvel > 2):
    #     print('error')
    #     print(qvel)
    #     time.sleep(5)

    limits = np.tile(np.array([30.1]),12)


    ineq = limits - qvel

    return ineq


def constraintFun_lo(q,q1,qprev):
    
    limits = np.tile(np.array([-1.0471975512,-0.663225115758,-2.72271363311]),4)


    ineq_lo = -limits + q


    return ineq_lo

def constraintFun_up(q,q1,qprev):

    limits = np.tile(np.array([1.0471975512,2.96705972839,-0.837758040957]),4)


    ineq_up = limits - q

    return ineq_up

def fbgs_opt(q,robot,Mgoal):
    try:  # This is needed for the first iteration (when there is no qprev)
        qprev
    except:
        qprev = q

    # xopt_bfgs = fmin_bfgs(positionError, q, args = (qprev,Mgoal,robot))
    ineq_lo = {
        "type" : "ineq",
        "fun" : constraintFun_lo,
        "args" : (q,qprev)
    }
    ineq_up = {
        "type" : "ineq",
        "fun" : constraintFun_up,
        "args" : (q,qprev)
    }

    ineq_vel = {
        "type" : "ineq",
        "fun" : constraintFun_vel,
        "args" : (q,qprev)
    }
#  
    xopt_bfgs = minimize(positionError,q,args = (qprev,Mgoal,robot),method='SLSQP',constraints=[ineq_lo,ineq_up,ineq_vel])
    qprev = q

    return xopt_bfgs


if __name__ == "__main__":
    IKsolver_server()
