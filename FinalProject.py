# %%
import time
import mujoco
import mujoco.viewer
import numpy as np

# %%

# Path to the xml file
xml_path = "Scene/wonik_allegro/scene_right.xml"

# Load model
model = mujoco.MjModel.from_xml_path(xml_path)
data = mujoco.MjData(model)

mujoco.mj_kinematics(model, data)

renderer = mujoco.Renderer(model)

# PID controller parameters
kp = 0.2  # Proportional gain
ki = 0.001  # Integral gain
kd = 0.01  # Derivative gain
integral_error = np.zeros(3)  # Integral error initialization
prev_error = np.zeros(3)  # Previous error initialization

error = 0.000001
hand_initial_pos = data.body('palm').xpos
target_initial_pos = data.body('cylinder_object').xpos
# Calculate the distance to the target
distance_to_target =  target_initial_pos - hand_initial_pos
init_flag = False
grasp_flag = False
ff_flag = False
mf_flag = False
rf_flag = False
th_flag = False
mujoco.mj_resetData(model, data)

# Init translational and rotational component of Jacobian
ff_tip_jacp = np.zeros((3, model.nv))  # position part of geometric Jacobian
ff_tip_jacr = np.zeros((3, model.nv))  # rotation part of geometric Jacobian

mf_tip_jacp = np.zeros((3, model.nv))  # position part of geometric Jacobian
mf_tip_jacr = np.zeros((3, model.nv))  # rotation part of geometric Jacobian

rf_tip_jacp = np.zeros((3, model.nv))  # position part of geometric Jacobian
rf_tip_jacr = np.zeros((3, model.nv))  # rotation part of geometric Jacobian

th_tip_jacp = np.zeros((3, model.nv))  # position part of geometric Jacobian
th_tip_jacr = np.zeros((3, model.nv))  # rotation part of geometric Jacobian

jacp = np.zeros((3, model.nv))  # position part of geometric Jacobian
jacr = np.zeros((3, model.nv))  # rotation part of geometric Jacobian

#Set target positions based on contact points for each finger
ff_tip_target = np.asarray([ 0.01164853, -0.03474731,  0.11374375])
mf_tip_target = np.asarray([ 0.01188803, -0.03492336,  0.06610088])
rf_tip_target = np.asarray([ 0.01188256, -0.03492185,  0.01817355])
th_tip_target = np.asarray([0.02739237, 0.03479632, 0.10815931])

# Get indexes for each finger and object so we can compute Jacobian matrix for each of them
ff_tip_idx = model.body('ff_tip').id
mf_tip_idx = model.body('mf_tip').id
rf_tip_idx = model.body('rf_tip').id
th_tip_idx = model.body('th_tip').id
cylinder_object_idx = model.body('cylinder_object').id

# Function that calculates the desired joint positions to move the hand towards the target.
def compute_control_signals(kp,ki,kd,error):
    global integral_error, prev_error

    # Proportional term
    p = kp * error

    # Integral term
    integral_error += error
    i = ki * integral_error

    # Derivative term
    derivative_error = error - prev_error
    d = kd * derivative_error

    # Calculate the control signals (desired joint positions)
    control_signals = p + i + d

    # Update previous error
    prev_error = error

    return control_signals

error_fingers = 0.025
with mujoco.viewer.launch_passive(model, data) as viewer:
    # Close the viewer automatically after 30 wall-seconds.
    start = time.time()
    while viewer.is_running() and time.time() - start < 1000:
        step_start = time.time()
        
        if np.abs(distance_to_target[0]) > error and np.abs(distance_to_target[1]) > error and np.abs(distance_to_target[2]) > error and init_flag == False:
          hand_initial_pos = data.body('palm').xpos

          target_initial_pos = data.body('cylinder_object').xpos

          # Calculate the distance to the target
          distance_to_target =  target_initial_pos - hand_initial_pos + [-0.095,0.01,0]
          #distance_to_target =  target_initial_pos - hand_initial_pos + [0,-0.04,0.19]

          # mj_step can be replaced with code that also evaluates
          # a policy and applies a control signal before stepping the physics.
          #data.ctrl[:]= data.ctrl[:] + [0.001, 0.002, 0.001,0.001, 0.002, 0.001,0.001, 0.002, 0.001,0.001, 0.002, 0.001,0.001, 0.002, 0.001,0.001]
          data.ctrl[0:3] = compute_control_signals(0.2,0.001,0.01,distance_to_target)
        elif grasp_flag == False:
          
          init_flag = True
          
          ff_tip_current_pos = data.body('ff_tip').xpos
          mf_tip_current_pos = data.body('mf_tip').xpos
          rf_tip_current_pos = data.body('rf_tip').xpos
          th_tip_current_pos = data.body('th_tip').xpos

          # Calculate the distance to the target
          ff_tip_distance_to_target =  ff_tip_target - ff_tip_current_pos
          mf_tip_distance_to_target =  mf_tip_target - mf_tip_current_pos
          rf_tip_distance_to_target =  rf_tip_target - rf_tip_current_pos
          th_tip_distance_to_target =  th_tip_target - th_tip_current_pos
          if np.abs(ff_tip_distance_to_target[0]) <  error_fingers and np.abs(ff_tip_distance_to_target[1]) < error_fingers and np.abs(ff_tip_distance_to_target[2]) < error_fingers:
            ff_flag = True
            
          if np.abs(mf_tip_distance_to_target[0]) < error_fingers and np.abs(mf_tip_distance_to_target[1]) < error_fingers and np.abs(mf_tip_distance_to_target[2]) < error_fingers:
            mf_flag = True
            
          if np.abs(rf_tip_distance_to_target[0]) < error_fingers and np.abs(rf_tip_distance_to_target[1]) < error_fingers and np.abs(rf_tip_distance_to_target[2]) < error_fingers:
            rf_flag = True
            
          if np.abs(th_tip_distance_to_target[0]) < error_fingers + 0.01 and np.abs(th_tip_distance_to_target[1]) < error_fingers + 0.01 and np.abs(th_tip_distance_to_target[2]) < error_fingers:
            th_flag = True
            
          if ff_flag == True and mf_flag == True and rf_flag == True and th_flag == True:
            grasp_flag = True
            hand_initial_pos = data.body('palm').xpos
            target_final_pos = np.asarray([0.02237285, 0.03170479, 0.36440524])

            # Calculate the distance to the target
            distance_to_final_target =  target_initial_pos - hand_initial_pos
          ff_tip_control_signals = compute_control_signals(5,0.4,0.01,ff_tip_distance_to_target)
          mf_tip_control_signals = compute_control_signals(5,0.4,0.01,mf_tip_distance_to_target)
          rf_tip_control_signals = compute_control_signals(5,0.4,0.01,rf_tip_distance_to_target)
          th_tip_control_signals = compute_control_signals(0.9,0.03,0.01,th_tip_distance_to_target)

          mujoco.mj_jac(model, data, ff_tip_jacp, ff_tip_jacr, data.body('ff_tip').xpos, ff_tip_idx)
          mujoco.mj_jac(model, data, mf_tip_jacp, mf_tip_jacr, data.body('mf_tip').xpos, mf_tip_idx)
          mujoco.mj_jac(model, data, rf_tip_jacp, rf_tip_jacr, data.body('rf_tip').xpos, rf_tip_idx)
          mujoco.mj_jac(model, data, th_tip_jacp, th_tip_jacr, data.body('th_tip').xpos, th_tip_idx)

          # Reshape Jacobians to 3D matrices
          ff_tip_jacp = ff_tip_jacp.reshape((3, model.nv))
          ff_tip_jacr = ff_tip_jacr.reshape((3, model.nv))
          
          mf_tip_jacp = mf_tip_jacp.reshape((3, model.nv))
          mf_tip_jacr = mf_tip_jacr.reshape((3, model.nv))
          
          rf_tip_jacp = rf_tip_jacp.reshape((3, model.nv))
          rf_tip_jacr = rf_tip_jacr.reshape((3, model.nv))
          
          th_tip_jacp = th_tip_jacp.reshape((3, model.nv))
          th_tip_jacr = th_tip_jacr.reshape((3, model.nv))
          # Assuming 'control_signals' are your current control signals in Cartesian space
          # Convert them to joint space using the Jacobian. This is a simple pseudoinverse-Jacobian control.
          ff_tip_control_signals_joint_space = np.linalg.pinv(ff_tip_jacp) @ ff_tip_control_signals
          mf_tip_control_signals_joint_space = np.linalg.pinv(mf_tip_jacp) @ mf_tip_control_signals
          rf_tip_control_signals_joint_space = np.linalg.pinv(rf_tip_jacp) @ rf_tip_control_signals
          th_tip_control_signals_joint_space = np.linalg.pinv(th_tip_jacp) @ th_tip_control_signals

          # Now you can apply these control signals to the joints
          control_signals_joint_space = np.concatenate((ff_tip_control_signals_joint_space[3:7], mf_tip_control_signals_joint_space[7:11],  rf_tip_control_signals_joint_space[11:15], th_tip_control_signals_joint_space[15:19]))
          data.ctrl[3:] = control_signals_joint_space
        
        elif np.abs(distance_to_final_target[0]) > error and np.abs(distance_to_final_target[1]) > error and np.abs(distance_to_final_target[2]) > error:
          hand_initial_pos = data.body('palm').xpos

          target_final_pos = np.asarray([0.02347479, 0.03373757, 0.34524079])

          # Calculate the distance to the target
          distance_to_final_target =  target_final_pos - hand_initial_pos

          # mj_step can be replaced with code that also evaluates
          # a policy and applies a control signal before stepping the physics.
          #data.ctrl[:]= data.ctrl[:] + [0.001, 0.002, 0.001,0.001, 0.002, 0.001,0.001, 0.002, 0.001,0.001, 0.002, 0.001,0.001, 0.002, 0.001,0.001]
          data.ctrl[0:3] = compute_control_signals(0.2,0.001,0.01,distance_to_final_target)
        
          
        mujoco.mj_step(model, data)
        #mujoco.mj_forward(model, data)
        
        # Update the scene in the renderer
        renderer.update_scene(data)
        
        with viewer.lock():
          viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = 1

        # Pick up changes to the physics state, apply perturbations, update options from GUI.
        viewer.sync()


        # Rudimentary time keeping, will drift relative to wall clock.
        time_until_next_step = model.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)
            
# %%
# Iterate over all contact points
for i in range(data.ncon):
    # Get contact point i
    contact = data.contact[i]

    # Get geom id from the contact point
    geom1_id, geom2_id = contact.geom1, contact.geom2

    # Get the body id for the geoms
    body1_id = model.geom_bodyid[geom1_id]
    body2_id = model.geom_bodyid[geom2_id]

    # Get the names of the bodies involved in the contact
    body1_name = model.body(body1_id).name
    body2_name = model.body(body2_id).name

    # Check if 'ff_tip' and 'cylinder_object' are involved in the contact
    if ((body1_name == 'ff_tip' and body2_name == 'cylinder_object') or
        (body1_name == 'cylinder_object' and body2_name == 'ff_tip')):
        # Print the position of the contact
        print('Contact position:', contact.pos)
        
    if ((body1_name == 'mf_tip' and body2_name == 'cylinder_object') or
        (body1_name == 'cylinder_object' and body2_name == 'mf_tip')):
        # Print the position of the contact
        print('Contact position:', contact.pos)
        
    if ((body1_name == 'rf_tip' and body2_name == 'cylinder_object') or
        (body1_name == 'cylinder_object' and body2_name == 'rf_tip')):
        # Print the position of the contact
        print('Contact position:', contact.pos)
        
    if ((body1_name == 'th_tip' and body2_name == 'cylinder_object') or
        (body1_name == 'cylinder_object' and body2_name == 'th_tip')):
        # Print the position of the contact
        print('Contact position:', contact.pos)

# %%
