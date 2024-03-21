# tryingout casadi optimizing with pinocchio wrapper
from os.path import dirname, join, abspath
 
import numpy as np
import pinocchio as pin
from pinocchio import casadi as cpin
import casadi as cs
 
using_cost_function1 = False # flage for switching different cost function
# load urdf file
pino_model_dir = "/home/anran.zhang/tutorials_code/pinocchio/"
urdf_file = pino_model_dir + 'ur5_robot.urdf'
 
# build model and data
model = pin.buildModelFromUrdf(urdf_file)
data = model.createData()
print('model name:' + model.name )
 
# generate casadi model from pinocchio
model_cs = cpin.Model(model)
data_cs = model_cs.createData()
q_cs = cs.SX.sym('q', model.nq)
 
# evaluate feedforward kinematics
cpin.framesForwardKinematics(model_cs, data_cs, q_cs)
pos_cs = data_cs.oMf[model.getFrameId("ee_fixed_joint")].translation

# create casadi function
fk = cs.Function("forward_kinematics", [q_cs], [pos_cs])

#* cost function 1:
# desired pose: using desired 4x4 homogeneous transformation
transform_target_to_world = pin.SE3(
    pin.utils.rotate("x", np.pi / 4),
    np.array([-0.5, 0.3, 0.2]), # reference pose
)
error_tool = cs.Function(
    "error_tool",
    [q_cs],
    [
        cpin.log6(
            data_cs.oMf[model.getFrameId("ee_fixed_joint")].inverse() * cpin.SE3(transform_target_to_world)
        ).vector
    ],
)
 
# solve the optimal control problem
# min||x_ref - fk(q)||
# s.t. -PI < q < PI
opti = cs.Opti()
var_q = opti.variable(model.nq)
if using_cost_function1:
    opti.minimize(cs.sumsqr(error_tool(var_q)))
    opti.subject_to(opti.bounded(-np.pi, var_q, np.pi))
else:
    slack = opti.variable()
    reference_position = opti.parameter(3)
    opti.set_value(reference_position, np.array([-0.5, 0.7, 0.2])) # reference pose value
    var_qmx = cs.MX.sym("x", model.nq)

    # cost function 1
    opti.minimize(cs.sumsqr(cs.log10(reference_position - fk(var_q)+1))) 
    # opti.minimize(cs.sumsqr(reference_position - fk(var_q)))

    # opti.subject_to(opti.bounded(-slack, reference_position - fk(var_q), slack))
    opti.subject_to(opti.bounded(-np.pi, var_q, np.pi))
 
opti.set_initial(var_q, 1)
 
opti.solver('ipopt')
 
sol = opti.solve()
 
# print(sol.value(slack))
print(sol.value(var_q))
print(fk(sol.value(var_q)))

# print("************numerical solution*******************")
# q = [0.1,0.1,0.1,0.1,0.1,0.1]
# print(fk(q))
# q = [0.2,0.2,0.2,0.2,0.2,0.2]
# print(fk(q))
