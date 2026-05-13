function traj = plan_trajectory(poses, geo, timing)
% Generates the end-effector trajectory across 6 phases:
% Approach -> Track -> Transit -> Descend (90° rot) -> Hold -> Return
%
% Args:
%   poses: Struct with the key target waypoints
%   geo: Struct with scene geometry and conveyor velocities
%   timing: Struct with time durations for each phase
%
% Returns:
%   traj: Struct containing the continuous trajectory data 
%         (t, p, v, phi, w, T) generated via 5th-order polynomials

dt = timing.dt;

% Phase boundary times
T1 = timing.t_appr;
T2 = T1 + timing.t_track;
T3 = T2 + timing.t_trans;
T4 = T3 + timing.t_desc;
T5 = T4 + timing.t_hold;
T6 = timing.T_total - T5;

% Safety checks
assert(T6 > 0, 'Phase durations exceed T_total.');
assert(T3 >= geo.beltB_dur, 'Assembly starts before belt B has stopped.');

% Velocity vector of Conveyor A
vA = [geo.beltA_vx; 0; 0];

% TCP position at the end of tracking phase
p_track_end = poses.p_grasp + vA * timing.t_track;

% Time vector generation and memory pre-allocation
t = 0:dt:timing.T_total;
N = numel(t);
p = zeros(3, N);
v = zeros(3, N);
phi = zeros(1, N);
w = zeros(1, N);

% Construct the trajectory point-by-point across all 6 phases
for k = 1:N
    tk = t(k);

    if tk <= T1
        % Phase 1: approach (end velocity matches belt A)
        [p(:,k), v(:,k)] = poly5_vec(poses.p_home, poses.p_grasp, [0;0;0], vA, T1, tk);
        [phi(k), w(k)] = poly5_scalar(poses.phi_home, poses.phi_grasp, 0, 0, T1, tk);

    elseif tk <= T2
        % Phase 2: linear tracking at belt velocity
        t_phase = tk - T1;
        p(:,k) = poses.p_grasp + vA * t_phase;
        v(:,k) = vA;
        phi(k) = poses.phi_grasp;

    elseif tk <= T3
        % Phase 3: transit
        t_phase = tk - T2;
        [p(:,k), v(:,k)] = poly5_vec(p_track_end, poses.p_above_B, ...
                                     vA, [0;0;0], timing.t_trans, t_phase);
        phi(k) = poses.phi_above_B;
        w(k) = 0;

    elseif tk <= T4
        % Phase 4: descend and screw
        t_phase = tk - T3;
        [p(:,k), v(:,k)] = poly5_vec(poses.p_above_B, poses.p_assembly, ...
                                     [0;0;0], [0;0;0], timing.t_desc, t_phase);
        [phi(k), w(k)] = poly5_scalar(poses.phi_above_B, poses.phi_assembly, ...
                                      0, 0, timing.t_desc, t_phase);
    elseif tk <= T5
        % Phase 5: hold at assembly
        p(:,k) = poses.p_assembly;
        phi(k) = poses.phi_assembly;

    else
        % Phase 6: return home
        t_phase = tk - T5;
        [p(:,k), v(:,k)] = poly5_vec(poses.p_assembly, poses.p_home, ...
                                     [0;0;0], [0;0;0], T6, t_phase);
        [phi(k), w(k)] = poly5_scalar(poses.phi_assembly, poses.phi_home, ...
                                      0, 0, T6, t_phase);
    end
end

% Build homogeneous transforms (tool z-axis pointing down)
T = zeros(4, 4, N);
for k = 1:N
    T(:,:,k) = transl(p(:,k).') * trotz(phi(k)) * trotx(pi);
end

traj.t = t;
traj.p = p;
traj.v = v;
traj.phi = phi;
traj.w = w;
traj.T = T;

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


function [s, v] = poly5_vec(s0, sf, v0, vf, T, t)
n = numel(s0);
s = zeros(n,1);
v = zeros(n,1);
for i = 1:n
    [s(i), v(i)] = poly5_scalar(s0(i), sf(i), v0(i), vf(i), T, t);
end
end
