%% Robotics - Part 1
clear; close all; clc;

%% 1. Robot
[scara, q0] = create_robot();

%% 2. Scene geometry
geo = struct();

% Conveyor A - moves continuously in +x
geo.beltA_y = -1.0;
geo.beltA_z = 1.5;
geo.beltA_x0 = -0.5;
geo.beltA_vx = 0.3;

% Conveyor B - moves stepwise in -x for 2.4 s
geo.beltB_y = 1.0;
geo.beltB_z = 1.0;
geo.beltB_x0 = 2.0;
geo.beltB_vx = -0.8;
geo.beltB_dur = 2.4;

% Vertical offset from TCP to bottom of Part A while held in the gripper
geo.partA_offset = 0.18;

% Part A
geo.partA_head_w = 0.25;
geo.partA_head_h = 0.10;
geo.partA_screw_w = 0.15;
geo.partA_screw_h = 0.08;

% Part B
geo.partB_w = 0.30;
geo.partB_h = 0.10;
geo.partB_socket_d = 0.08;
geo.partB_socket_w = 0.15;

%% 3. Timing
timing = struct();
timing.dt = 1e-3;
timing.T_total = 12.0;
timing.t_appr = 2.0;    % home -> grasp
timing.t_track = 0.5;   % follow belt A while the gripper closes
timing.t_trans = 3.0;   % grasp end -> above Part B
timing.t_lower = 0.5;   % above Part B -> entry (vertical, no rotation)
timing.t_screw = 1.0;   % entry -> assembly (vertical + 90° rotation)
timing.t_hold = 0.5;    % gripper opens

%% 4. Key poses and trajectory
T_home = scara.fkine(q0);   % forward kinematics
if isa(T_home,'SE3'), T_home = T_home.T; end

poses = key_poses(geo, timing, T_home);
traj = plan_trajectory(poses, geo, timing);

%% 5. Inverse kinematics over the trajectory
N = numel(traj.t);
q = zeros(4, N);
q(:,1) = q0;
for k = 2:N
    q(:,k) = scara_ikine(traj.T(:,:,k), q(:,k-1));
end

%% 6. Joint velocities (numerical differentiation)
dq = zeros(4, N);
dq(:, 2:end) = diff(q, 1, 2) / timing.dt;
dq(:, 1) = dq(:, 2);   % Assume constant velocity for the first time step

%% 7. Plots
plot_results(traj, q, dq);

%% 8. Animation (twο cycles)
animate_scene(scara, q, traj.t, geo, timing, 2);

%% Visualize simulation results
function plot_results(traj, q, dq)

figure('Name','End-effector position');
lab = {'x [m]','y [m]','z [m]'};
for i = 1:3
    subplot(3,1,i); plot(traj.t, traj.p(i,:), 'LineWidth', 1.4); grid on;
    xlabel('t [s]'); ylabel(lab{i});
end
sgtitle('End-effector position');

figure('Name','End-effector velocity');
lab = {'v_x [m/s]','v_y [m/s]','v_z [m/s]'};
for i = 1:3
    subplot(3,1,i); plot(traj.t, traj.v(i,:), 'LineWidth', 1.4); grid on;
    xlabel('t [s]'); ylabel(lab{i});
end
sgtitle('End-effector linear velocity');

figure('Name','End-effector orientation');
subplot(2,1,1); plot(traj.t, rad2deg(traj.phi), 'LineWidth', 1.4); grid on;
    xlabel('t [s]'); ylabel('\phi [deg]'); title('Tool rotation about z');
subplot(2,1,2); plot(traj.t, rad2deg(traj.w), 'LineWidth', 1.4); grid on;
    xlabel('t [s]'); ylabel('d\phi/dt [deg/s]'); title('Angular velocity');

figure('Name','Joint positions');
lab = {'q_1 [rad]','q_2 [rad]','q_3 [m]','q_4 [rad]'};
for i = 1:4
    subplot(4,1,i); plot(traj.t, q(i,:), 'LineWidth', 1.4); grid on;
    xlabel('t [s]'); ylabel(lab{i});
end
sgtitle('Joint positions');

figure('Name','Joint velocities');
lab = {'dq_1 [rad/s]','dq_2 [rad/s]','dq_3 [m/s]','dq_4 [rad/s]'};
for i = 1:4
    subplot(4,1,i); plot(traj.t, dq(i,:), 'LineWidth', 1.4); grid on;
    xlabel('t [s]'); ylabel(lab{i});
end
sgtitle('Joint velocities');

end
