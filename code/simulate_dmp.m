function [y, dy, ddy] = simulate_dmp(dmp, y0, g, t, dy0)
% Integrate a trained DMP forward with new start and goal
%
% Args:
%   dmp: (struct) trained DMP model containing weights, centers, parameters
%   y0: new starting position
%   g: new goal position
%   t: time vector for the simulation
%   dy0: (optional) initial spatial velocity vector
%
% Returns:
%   y: generated position trajectory
%   dy: generated velocity trajectory
%   ddy: generated acceleration trajectory

dt = t(2) - t(1);
N_t = length(t);
y0 = y0(:);
g = g(:);
D = numel(y0);
scale = g - y0;   % the new spatial scale

% Default to zero initial velocity if not provided
if nargin < 5, dy0 = zeros(D, 1); end

% State variables
x = 1;   % canonical phase clock (starts at 1, decays to 0)
y_curr = y0(:);   % initial position
z_curr = dmp.tau * dy0(:);   % initial scaled velocity (allows non-zero starting velocity)

y = zeros(D, N_t);
dy = zeros(D, N_t);
ddy = zeros(D, N_t);

% Forward Simulation Loop (Euler Integration)
for k = 1:N_t
    % 1. Calculate the non-linear forcing term f(x)
    psi = exp(-dmp.h .* (x - dmp.c).^2);
    psi_sum = sum(psi);
    
    if psi_sum > 1e-10
        % Multiply activations with learned weights, normalize, and decay with 'x'
        f = (dmp.w * psi.') * x / psi_sum;
    else
        f = zeros(D, 1);
    end

    % Scale the learned force to fit the new distance robot has to cover
    f_scaled = f .* scale;

    % 2. Calculate Acceleration (Transformation System dynamics)
    z_acc = dmp.alpha_z * (dmp.beta_z * (g - y_curr) - z_curr) + f_scaled;

    % 3. Record Outputs for the current step
    y(:, k) = y_curr;
    dy(:, k) = z_curr / dmp.tau;     % true spatial velocity
    ddy(:, k) = z_acc / dmp.tau^2;   % true spatial acceleration

    % 4. Euler Integration Step (Move 1 time-step into the future)
    if k < N_t
        z_curr = z_curr + dt * z_acc / dmp.tau;
        y_curr = y_curr + dt * dy(:, k);
        x = x + dt * (-dmp.alpha_x * x / dmp.tau);
    end
end

end
