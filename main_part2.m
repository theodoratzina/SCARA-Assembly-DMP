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
timing.t_trans = 3.5;   % grasp end -> above Part B (90 deg rotation)
timing.t_desc = 1.0;    % descend onto Part B
timing.t_hold = 0.5;    % gripper opens

%% 2. Generate Part 1 demonstration
T_home = scara.fkine(q0);
if isa(T_home, 'SE3'), T_home = T_home.T; end

poses = key_poses(geo, timing, T_home);
demo = plan_trajectory(poses, geo, timing);

%% 3. Slice the demo into "forward" (home -> assembly) and "return"
T_fwd_end = timing.t_appr + timing.t_track + timing.t_trans + timing.t_desc;
T_ret_start = T_fwd_end + timing.t_hold;

idx_fwd = demo.t <= T_fwd_end + 1e-9;
idx_ret = demo.t >= T_ret_start - 1e-9;

% Extract time vectors for each phase
t_fwd = demo.t(idx_fwd) - demo.t(find(idx_fwd, 1));
t_ret = demo.t(idx_ret) - demo.t(find(idx_ret, 1));

% Extract position (p) and orientation (phi) data
p_fwd = demo.p(:, idx_fwd);     
phi_fwd = demo.phi(idx_fwd);
p_ret = demo.p(:, idx_ret);    
phi_ret = demo.phi(idx_ret);

%% 4. Numerical derivatives for DMP training
[dp_fwd, ddp_fwd ] = num_deriv(p_fwd, timing.dt);
[dp_ret, ddp_ret ] = num_deriv(p_ret, timing.dt);
[dphi_fwd, ddphi_fwd] = num_deriv(phi_fwd, timing.dt);
[dphi_ret, ddphi_ret] = num_deriv(phi_ret, timing.dt);

%% 5. Train four DMPs (position + orientation, forward + return)
dmp_pos_fwd = train_dmp(t_fwd, p_fwd, dp_fwd, ddp_fwd);
dmp_pos_ret = train_dmp(t_ret, p_ret, dp_ret, ddp_ret);
dmp_phi_fwd = train_dmp(t_fwd, phi_fwd, dphi_fwd, ddphi_fwd);
dmp_phi_ret = train_dmp(t_ret, phi_ret, dphi_ret, ddphi_ret);

%% 6. Three scenarios (delta, theta_delta)
scen(1).delta = [0; 0; 0];   
scen(1).theta = deg2rad(0);
scen(2).delta = [-0.2; 0.2; 0];   
scen(2).theta = deg2rad(20);
scen(3).delta = [0.4; -0.15; 0];   
scen(3).theta = deg2rad(-25);
n_s = numel(scen);   % number of scenarios

all_q = cell(n_s, 1);     % joint angles 
all_p = cell(n_s, 1);     % end-effector positions
all_phi = cell(n_s, 1);   % end-effector orientations
all_t = cell(n_s, 1);     % time vectors
geo_per_cycle = cell(n_s, 1);    % specific geometry (Part B's pose)

%% 7. Generate one full cycle per scenario
for s = 1:n_s
    g_pos = poses.p_assembly + scen(s).delta;
    g_phi = poses.phi_assembly + scen(s).theta;

    % Forward stroke
    [p_f, ~, ~] = simulate_dmp(dmp_pos_fwd, poses.p_home, g_pos, t_fwd);
    [phi_f, ~, ~] = simulate_dmp(dmp_phi_fwd, poses.phi_home, g_phi, t_fwd);

    % Hold (gripper opens/closes)
    n_h = round(timing.t_hold / timing.dt);
    p_h = repmat(p_f(:, end), 1, n_h);
    phi_h = repmat(phi_f(:, end), 1, n_h);

    % Return stroke (start from current end pose)
    [p_r, ~, ~] = simulate_dmp(dmp_pos_ret, g_pos, poses.p_home, t_ret);
    [phi_r, ~, ~] = simulate_dmp(dmp_phi_ret, g_phi, poses.phi_home, t_ret);

    % Concatenate
    p_full = [p_f, p_h, p_r];
    phi_full = [phi_f, phi_h, phi_r];
    N_full = size(p_full, 2);
    t_full = (0:N_full - 1) * timing.dt;

    % Inverse Kinematics
    q_full = zeros(4, N_full);   
    q_full(:, 1) = q0;
    for k = 2:N_full
        Tk = transl(p_full(:, k).') * trotz(phi_full(k)) * trotx(pi);
        q_full(:, k) = scara_ikine(Tk, q_full(:, k - 1));
    end

    % Per-cycle scene geometry (Part B is shifted and rotated)
    geo_s = geo;
    geo_s.beltB_x0 = geo.beltB_x0 + scen(s).delta(1);
    geo_s.beltB_y = geo.beltB_y + scen(s).delta(2);
    geo_s.partB_rot = scen(s).theta;

    % Store data
    all_q{s} = q_full;
    all_p{s} = p_full;
    all_phi{s} = phi_full;
    all_t{s} = t_full;
    geo_per_cycle{s} = geo_s;
end

%% 8. Plots
plot_dmp_results(all_t, all_p, all_phi, all_q, demo, scen);

%% 9. Animation: 3 cycles, one per scenario
q_concat = cat(2, all_q{:});
t_concat = (0:size(q_concat, 2) - 1) * timing.dt;
animate_scene(scara, q_concat, t_concat, geo_per_cycle, timing, n_s);

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

function plot_dmp_results(all_t, all_p, all_phi, all_q, demo, scen)
% Visualize simulation results
n = numel(all_t);
colors = lines(n);
labs = arrayfun(@(s) sprintf('S%d  \\delta=[%.2f,%.2f]  \\theta_\\delta=%d°', ...
                s, scen(s).delta(1), scen(s).delta(2), ...
                round(rad2deg(scen(s).theta))), ...
                1:n, 'UniformOutput', false);

figure('Name','EE position (DMP)','Color','w');
ylabs = {'x [m]','y [m]','z [m]'};
for ax_i = 1:3
    subplot(3, 1, ax_i); hold on; grid on;
    plot(demo.t, demo.p(ax_i, :), 'k:', 'LineWidth', 1.0);
    for s = 1:n
        plot(all_t{s}, all_p{s}(ax_i, :), 'Color', colors(s, :), 'LineWidth', 1.3);
    end
    xlabel('t [s]'); ylabel(ylabs{ax_i});
    if ax_i == 1, legend(['Demo', labs], 'Location', 'best'); end
end
sgtitle('End-effector position - DMP scenarios');

figure('Name','EE orientation (DMP)','Color','w');
hold on; grid on;
plot(demo.t, rad2deg(demo.phi), 'k:', 'LineWidth', 1.0);
for s = 1:n
    plot(all_t{s}, rad2deg(all_phi{s}), 'Color', colors(s, :), 'LineWidth', 1.3);
end
xlabel('t [s]'); ylabel('\phi [deg]');
legend(['Demo', labs], 'Location', 'best');
title('End-effector orientation - DMP scenarios');

figure('Name','Joint positions (DMP)','Color','w');
ylabs = {'q_1 [rad]','q_2 [rad]','q_3 [m]','q_4 [rad]'};
for ax_i = 1:4
    subplot(4, 1, ax_i); hold on; grid on;
    for s = 1:n
        plot(all_t{s}, all_q{s}(ax_i, :), 'Color', colors(s, :), 'LineWidth', 1.3);
    end
    xlabel('t [s]'); ylabel(ylabs{ax_i});
    if ax_i == 1, legend(labs, 'Location', 'best'); end
end
sgtitle('Joint positions - DMP scenarios');

end
