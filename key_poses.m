function poses = key_poses(geo, timing, T_home)
% Compute the key end-effector poses for the assembly task
%
% Args:
%   geo: Struct containing scene geometry parameters
%   timing: Struct containing trajectory timing parameters
%   T_home: 4x4 homogeneous transformation matrix of the initial pose
%
% Returns:
%   poses: Struct with the 4 target waypoints (home, grasp, above_B, assembly)

% Home
poses.p_home = T_home(1:3, 4);
poses.phi_home = atan2(T_home(2,1), T_home(1,1));

% Grasp pose: meet Part A on belt A
xA_grasp = geo.beltA_x0 + geo.beltA_vx * timing.t_appr;
poses.p_grasp = [xA_grasp; geo.beltA_y; geo.beltA_z + geo.partA_offset];
poses.phi_grasp = poses.phi_home;

% Above Part B: final position, clearance height
xB = geo.beltB_x0 + geo.beltB_vx * geo.beltB_dur;
clearance = 0.30;
z_above_B = geo.beltB_z + geo.partB_h + clearance;
poses.p_above_B = [xB; geo.beltB_y; z_above_B];
poses.phi_above_B = poses.phi_home + pi/2;   % rotation done during transit

% Assembly
z_assembly = geo.beltB_z + geo.partB_h - geo.partB_socket_d + geo.partA_offset;
poses.p_assembly = [xB; geo.beltB_y; z_assembly];
poses.phi_assembly = poses.phi_above_B;

end
