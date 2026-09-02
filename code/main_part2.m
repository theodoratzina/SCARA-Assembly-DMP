%% Robotics - Part 2
clear; close all; clc;

%% 1. Robot, scene, timing  (same as Part 1)
[scara, q0] = create_robot();

% Scene geometry
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

% Timing
timing = struct();
timing.dt = 1e-3;
timing.T_total = 12.0;
timing.t_appr = 2.0;    % home -> grasp
timing.t_track = 0.5;   % follow belt A while the gripper closes
timing.t_trans = 3.0;   % grasp end -> above Part B
timing.t_lower = 0.5;   % above Part B -> entry (vertical, no rotation)
timing.t_screw = 1.0;   % entry -> assembly (vertical + 90° rotation)
timing.t_hold = 0.5;    % gripper opens

%% 2. Generate Part 1 demonstration
T_home = scara.fkine(q0);
if isa(T_home, 'SE3'), T_home = T_home.T; end

poses = key_poses(geo, timing, T_home);
demo = plan_trajectory(poses, geo, timing);

% Compute demo joint positions and velocities (for overlay in plots)
N_demo = numel(demo.t);
demo.q = zeros(4, N_demo);  demo.q(:,1) = q0;
for k = 2:N_demo
    demo.q(:,k) = scara_ikine(demo.T(:,:,k), demo.q(:,k-1));
end
demo.dq = zeros(4, N_demo);
for i = 1:4
    demo.dq(i,:) = gradient(demo.q(i,:), timing.dt);
end

%% 3. Slice the demo into three segments
T2 = timing.t_appr + timing.t_track;   % grasp: 0 -> T2
T_fwd_end = T2 + timing.t_trans + timing.t_lower + timing.t_screw;   % deliver: T2 -> T_fwd
T_ret_start = T_fwd_end + timing.t_hold;   % return:  T_ret -> end

idx_grasp = demo.t <= T2 + 1e-9;
idx_deliver = demo.t >= T2 - 1e-9 & demo.t <= T_fwd_end + 1e-9;
idx_ret = demo.t >= T_ret_start - 1e-9;

% Grasp segment (replayed unchanged every cycle)
t_grasp = demo.t(idx_grasp);
p_grasp = demo.p(:, idx_grasp);
phi_grasp = demo.phi(idx_grasp);
 
% Deliver segment (DMP-adapted)
t_deliver = demo.t(idx_deliver) - demo.t(find(idx_deliver, 1));
p_deliver = demo.p(:, idx_deliver);
phi_deliver = demo.phi(idx_deliver);
 
% Return segment (DMP-adapted)
t_ret = demo.t(idx_ret) - demo.t(find(idx_ret, 1));
p_ret = demo.p(:, idx_ret);
phi_ret = demo.phi(idx_ret);

%% 4. Numerical derivatives for DMP training
[dp_del, ddp_del ] = num_deriv(p_deliver, timing.dt);
[dp_ret, ddp_ret ] = num_deriv(p_ret, timing.dt);
[dphi_del, ddphi_del] = num_deriv(phi_deliver, timing.dt);
[dphi_ret, ddphi_ret] = num_deriv(phi_ret, timing.dt);

%% 5. Train DMPs (deliver + return only; grasp is replayed)
dmp_pos_del = train_dmp(t_deliver, p_deliver, dp_del, ddp_del);
dmp_pos_ret = train_dmp(t_ret, p_ret, dp_ret, ddp_ret);
dmp_phi_del = train_dmp(t_deliver, phi_deliver, dphi_del, ddphi_del);
dmp_phi_ret = train_dmp(t_ret, phi_ret, dphi_ret, ddphi_ret);

%% 6. Three scenarios (delta, theta_delta)
scen(1).delta = [0; 0; 0];   
scen(1).theta = deg2rad(0);
scen(2).delta = [-0.2; 0.2; 0];   
scen(2).theta = deg2rad(20);
scen(3).delta = [0.4; -0.15; 0];   
scen(3).theta = deg2rad(-25);
n_s = numel(scen);   % number of scenarios

%% 7. Generate Full Cycles
% Start points for the deliver DMP (= end of grasp, same every cycle)
p_grasp_end = p_grasp(:, end);
phi_grasp_end = phi_grasp(end);

% Question 2: Position DMP only (orientation remains constant)
all_q_q2 = cell(n_s, 1);   
all_p_q2 = cell(n_s, 1);
all_phi_q2 = cell(n_s, 1); 
all_t_q2 = cell(n_s, 1);
geo_per_cycle_q2 = cell(n_s, 1);

for s = 1:n_s
    g_pos = poses.p_assembly + scen(s).delta;
    
    % 1. Grasp: replay Part 1 (unchanged)
    p_1 = p_grasp;
    phi_1 = phi_grasp;
 
    % 2. Deliver: position DMP, orientation from Part 1
    dy0_del = dp_del(:, 1);
    [p_2, ~, ~] = simulate_dmp(dmp_pos_del, p_grasp_end, g_pos, t_deliver, dy0_del);
    phi_2 = phi_deliver;
 
    % 3. Hold
    n_h = round(timing.t_hold / timing.dt);
    p_3 = repmat(p_2(:, end), 1, n_h);
    phi_3 = repmat(phi_2(end), 1, n_h);
 
    % 4. Return: position DMP, orientation from Part 1
    [p_4, ~, ~] = simulate_dmp(dmp_pos_ret, g_pos, poses.p_home, t_ret);
    phi_4 = phi_ret;
 
    % Concatenate
    p_full = [p_1, p_2, p_3, p_4];
    phi_full = [phi_1, phi_2, phi_3, phi_4];
    N_full = size(p_full, 2);
    t_full = (0:N_full - 1) * timing.dt;
 
    % Inverse Kinematics
    q_full = zeros(4, N_full);   
    q_full(:, 1) = q0;
    for k = 2:N_full
        Tk = transl(p_full(:, k).') * trotz(phi_full(k)) * trotx(pi);
        q_full(:, k) = scara_ikine(Tk, q_full(:, k - 1));
    end
 
    % Per-cycle geometry
    geo_s = geo;
    geo_s.beltB_x0 = geo.beltB_x0 + scen(s).delta(1);
    geo_s.beltB_y = geo.beltB_y + scen(s).delta(2);
    geo_s.partB_rot = 0;
 
    all_q_q2{s} = q_full;   
    all_p_q2{s} = p_full;
    all_phi_q2{s} = phi_full; 
    all_t_q2{s} = t_full;
    geo_per_cycle_q2{s} = geo_s;
end

% Question 3: Position and Orientation DMPs (full adaptation)
all_q_q3 = cell(n_s, 1);   
all_p_q3 = cell(n_s, 1);
all_phi_q3 = cell(n_s, 1); 
all_t_q3 = cell(n_s, 1);
geo_per_cycle_q3 = cell(n_s, 1);

for s = 1:n_s
    g_pos = poses.p_assembly + scen(s).delta;
    g_phi = poses.phi_assembly + scen(s).theta;

     % 1. Grasp: replay Part 1 (unchanged)
    p_1 = p_grasp;
    phi_1 = phi_grasp;
 
    % 2a. Deliver Position
    dy0_del = dp_del(:, 1);
    [p_2, ~, ~] = simulate_dmp(dmp_pos_del, p_grasp_end, g_pos, t_deliver, dy0_del);
    
    % 2b. Deliver Orientation: Custom polynomial instead of DMP 
    phi_2 = zeros(1, length(t_deliver));
    t_align = timing.t_trans + timing.t_lower; % Duration of air movement
    
    for i = 1:length(t_deliver)
        t_loc = t_deliver(i);
        if t_loc <= t_align
            % Phase A: Rotate to match the misalignment (theta_delta)
            [phi_2(i), ~] = poly5_scalar(phi_grasp_end, scen(s).theta, 0, 0, t_align, t_loc);
        else
            % Phase B: The actual screw motion (strictly 90 degrees / pi/2)
            t_screw_loc = t_loc - t_align;
            [phi_2(i), ~] = poly5_scalar(scen(s).theta, scen(s).theta + pi/2, 0, 0, timing.t_screw, t_screw_loc);
        end
    end
 
    % 3. Hold
    n_h = round(timing.t_hold / timing.dt);
    p_3 = repmat(p_2(:, end), 1, n_h);
    phi_3 = repmat(phi_2(end), 1, n_h);
 
    % 4. Return: position + orientation DMPs
    [p_4, ~, ~] = simulate_dmp(dmp_pos_ret, g_pos, poses.p_home, t_ret);
    [phi_4, ~, ~] = simulate_dmp(dmp_phi_ret, g_phi, poses.phi_home, t_ret);
 
    % Concatenate
    p_full = [p_1, p_2, p_3, p_4];
    phi_full = [phi_1, phi_2, phi_3, phi_4];
    N_full = size(p_full, 2);
    t_full = (0:N_full - 1) * timing.dt;
 
    % Inverse Kinematics
    q_full = zeros(4, N_full);   
    q_full(:, 1) = q0;
    for k = 2:N_full
        Tk = transl(p_full(:, k).') * trotz(phi_full(k)) * trotx(pi);
        q_full(:, k) = scara_ikine(Tk, q_full(:, k - 1));
    end
 
    % Per-cycle geometry
    geo_s = geo;
    geo_s.beltB_x0 = geo.beltB_x0 + scen(s).delta(1);
    geo_s.beltB_y = geo.beltB_y + scen(s).delta(2);
    geo_s.partB_rot = scen(s).theta;
 
    all_q_q3{s} = q_full;   
    all_p_q3{s} = p_full;
    all_phi_q3{s} = phi_full; 
    all_t_q3{s} = t_full;
    geo_per_cycle_q3{s} = geo_s;
end

%% 8. Plots
plot_dmp_results(all_t_q2, all_p_q2, all_phi_q2, all_q_q2, demo, scen, 'Q2: Position DMP');
plot_dmp_results(all_t_q3, all_p_q3, all_phi_q3, all_q_q3, demo, scen, 'Q3: Pos + Orient DMP');

%% 9. Animations: 3 cycles, one per scenario
disp('Q2: Gripper with constant angle');
q_concat_q2 = cat(2, all_q_q2{:});
t_concat_q2 = (0:size(q_concat_q2, 2) - 1) * timing.dt;
animate_scene(scara, q_concat_q2, t_concat_q2, geo_per_cycle_q2, timing, n_s);

disp('Q2 completed');
disp('Press ENTER in the Command Window to start Q3...');
pause;

disp('Q3: Full angle adaptation');
q_concat_q3 = cat(2, all_q_q3{:});
t_concat_q3 = (0:size(q_concat_q3, 2) - 1) * timing.dt;
animate_scene(scara, q_concat_q3, t_concat_q3, geo_per_cycle_q3, timing, n_s);
disp('Q3 completed');

%% Helper functions
function [dy, ddy] = num_deriv(y, dt)
% Per-row central differences for first and second derivatives.
[D, ~] = size(y);
dy = zeros(size(y));
ddy = zeros(size(y));
for d = 1:D
    dy(d, :) = gradient(y(d, :), dt);
    ddy(d, :) = gradient(dy(d, :), dt);
end
end

function plot_dmp_results(all_t, all_p, all_phi, all_q, demo, scen, ttl)
% Visualize DMP simulation results (5 figures per call)
n = numel(all_t);
dt = all_t{1}(2) - all_t{1}(1);
colors = lines(n);
demo_style = {'--', 'Color', [1 1 1], 'LineWidth', 1.5};   % white dashed, on top
labs = arrayfun(@(s) sprintf('S%d  \\delta=[%.2f,%.2f]  \\theta_\\delta=%d°', ...
                s, scen(s).delta(1), scen(s).delta(2), ...
                round(rad2deg(scen(s).theta))), ...
                1:n, 'UniformOutput', false);
 
% 1. EE Position
figure('Name', [ttl ' - EE position']);
ylabs = {'x [m]','y [m]','z [m]'};
for ax_i = 1:3
    subplot(3, 1, ax_i); hold on; grid on;
    for s = 1:n
        plot(all_t{s}, all_p{s}(ax_i, :), 'Color', colors(s, :), 'LineWidth', 1.3);
    end
    plot(demo.t, demo.p(ax_i, :), demo_style{:});   % demo on top
    xlabel('t [s]'); ylabel(ylabs{ax_i});
    if ax_i == 1, legend([labs, {'Demo'}], 'Location', 'best'); end
end
sgtitle([ttl ' - End-effector position']);
 
% 2. EE Velocity
figure('Name', [ttl ' - EE velocity']);
ylabs = {'v_x [m/s]','v_y [m/s]','v_z [m/s]'};
for ax_i = 1:3
    subplot(3, 1, ax_i); hold on; grid on;
    for s = 1:n
        v_s = gradient(all_p{s}(ax_i, :), dt);
        plot(all_t{s}, v_s, 'Color', colors(s, :), 'LineWidth', 1.3);
    end
    plot(demo.t, demo.v(ax_i, :), demo_style{:});
    xlabel('t [s]'); ylabel(ylabs{ax_i});
    if ax_i == 1, legend([labs, {'Demo'}], 'Location', 'best'); end
end
sgtitle([ttl ' - End-effector velocity']);
 
% 3. EE Orientation
figure('Name', [ttl ' - EE orientation']);
hold on; grid on;
for s = 1:n
    plot(all_t{s}, rad2deg(all_phi{s}), 'Color', colors(s, :), 'LineWidth', 1.3);
end
plot(demo.t, rad2deg(demo.phi), demo_style{:});
xlabel('t [s]'); ylabel('\phi [deg]');
legend([labs, {'Demo'}], 'Location', 'best');
title([ttl ' - End-effector orientation']);
 
% 4. Joint Positions
figure('Name', [ttl ' - Joint positions']);
ylabs = {'q_1 [rad]','q_2 [rad]','q_3 [m]','q_4 [rad]'};
for ax_i = 1:4
    subplot(4, 1, ax_i); hold on; grid on;
    for s = 1:n
        plot(all_t{s}, all_q{s}(ax_i, :), 'Color', colors(s, :), 'LineWidth', 1.3);
    end
    if isfield(demo, 'q')
        plot(demo.t, demo.q(ax_i, :), demo_style{:});
    end
    xlabel('t [s]'); ylabel(ylabs{ax_i});
    if ax_i == 1, legend([labs, {'Demo'}], 'Location', 'best'); end
end
sgtitle([ttl ' - Joint positions']);
 
% 5. Joint Velocities
figure('Name', [ttl ' - Joint velocities']);
ylabs = {'dq_1 [rad/s]','dq_2 [rad/s]','dq_3 [m/s]','dq_4 [rad/s]'};
for ax_i = 1:4
    subplot(4, 1, ax_i); hold on; grid on;
    for s = 1:n
        dq_s = gradient(all_q{s}(ax_i, :), dt);
        plot(all_t{s}, dq_s, 'Color', colors(s, :), 'LineWidth', 1.3);
    end
    if isfield(demo, 'dq')
        plot(demo.t, demo.dq(ax_i, :), demo_style{:});
    end
    xlabel('t [s]'); ylabel(ylabs{ax_i});
    if ax_i == 1, legend([labs, {'Demo'}], 'Location', 'best'); end
end
sgtitle([ttl ' - Joint velocities']);
 
end


% 5th-order polynomial with position+velocity boundary conditions
function [s, v] = poly5_scalar(s0, sf, v0, vf, T, t)
    h = sf - s0;
    a3 = (20*h - (8*vf + 12*v0)*T) / (2*T^3);
    a4 = (-30*h + (14*vf + 16*v0)*T) / (2*T^4);
    a5 = (12*h - 6*(vf + v0)*T) / (2*T^5);
    s = s0 + v0*t + a3*t^3 + a4*t^4 + a5*t^5;
    v = v0 + 3*a3*t^2 + 4*a4*t^3 + 5*a5*t^4;
end