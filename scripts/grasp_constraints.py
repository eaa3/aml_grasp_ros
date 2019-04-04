import numpy as np
from aml_math import *
import robot_kinematics as jk

# kin = jk.robot_kinematics()

def axis_alignment_constraint(manipulator, solution, **kwargs):

    directionA = kwargs.get('directionA', np.array([0.0, 0.0, 1.0]))
    directionB = kwargs.get('directionB', np.array([0.1, 0.0, 0.0]))
    transformation = position_quaternion2matrix(*solution.base_pose)

    costA = np.abs(np.dot(transformation[:3, 2],directionA))*100.0#(1.0-np.abs(np.dot(transformation[:3, 2],directionA)))*50.0
    costB = np.abs(np.dot(transformation[:3, 2],directionB))*100.0

    selector = 1.0 if np.random.rand() > 0.5 else 0.0

    #print "Axis constraint: ", (costA*selector + costB*(1.0 - selector))
    # solution.constraint_score -= costA#(costA*selector + costB*(1.0 - selector))


# def hard_axis_alignment_constraint(manipulator, solution, **kwargs):

#     directionA = kwargs.get('directionA', np.array([0.0, 0.0, 1.0]))
#     directionB = kwargs.get('directionB', np.array([1.0, 0.0, 0.0]))
#     transformation = position_quaternion2matrix(*solution.base_pose)

#     # costA = (1.0 - np.dot(transformation[:3, 2],directionA))*50.0#(1.0-np.abs(np.dot(transformation[:3, 2],directionA)))*50.0
#     # costB = (1.0-np.abs(np.dot(transformtation[:3, 2],directionB)))*50.0

#     direction = directionA if np.random.rand() > 0.5 else directionB

#     if np.abs(np.dot(transformation[:3, 2],direction)) < 0.6:
#         solution.skip = True
#         print "Skipping solution: ", solution.score()


def hard_kinematic_constraint(manipulator, solution, **kwargs):


    #solution.skip = not kin.kdl_inverse_kinematics_jl(solution.base_pose[0], solution.base_pose[1], None)
    
    grasp_ok = True
    # for pose in solution.base_trajectory:
    #     grasp_ok = grasp_ok and kin.trac_ik_inverse_kinematics(pose.p, pose.q, None)
    
    solution.skip = not kin.trac_ik_inverse_kinematics(solution.base_pose[0], solution.base_pose[1], None)
    #solution.skip = solution.skip or solution.base_pose[0][2] < -0.28
    # solution.skip = solution.skip or not np.all([e for e,_ in solution.contact_mask.items()])
    #print "Kinematics constraint: ",solution.skip

def hard_closure_constraint(manipulator, solution, **kwargs):
    
    mask = [3, 5, 6]
    
    # solution.skip = not np.all([solution.contact_mask.get(e,False) for e in mask])