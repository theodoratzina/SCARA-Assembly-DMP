function dmp = train_dmp(t, y, dy, ddy, params)
% Train a Dynamic Movement Primitive on a vector demonstration
%
% Args:
%   t: 1 x N time vector
%   y: D x N demonstration trajectory (D = 3 for position, 1 for orientation)
%   dy: D x N demonstration velocity
%   ddy: D x N demonstration acceleration
%   params: (Optional) Struct overriding default DMP parameters
%
% Returns:
%   dmp: struct containing the trained model parameters

% 1. Parameter Initialization
if nargin < 5, params = struct(); end
if ~isfield(params,'alpha_z'), params.alpha_z = 25; end
if ~isfield(params,'beta_z'), params.beta_z = params.alpha_z / 4; end
if ~isfield(params,'alpha_x'), params.alpha_x = 1; end
if ~isfield(params,'n_basis'), params.n_basis = 50; end

tau = t(end) - t(1);   % total duration of demonstration
y0 = y(:, 1);          % initial state (start)
g = y(:, end);         % final state (goal)
N = params.n_basis;

% 2. Canonical phase x(t)
xt = exp(-params.alpha_x * (t - t(1)) / tau);

% 3. Gaussian Basis Functions Setup
c = exp(-params.alpha_x * linspace(0, 1, N));
diffs = diff(c);
h = 1.5 ./ [diffs, diffs(end)].^2;

% 4. Target Forcing Term Calculation
f_target = tau^2 * ddy - params.alpha_z * (params.beta_z * (g - y) - tau * dy);

% 5. Spatial Scaling & Safety Check
scale_safe = g - y0;
small_dim = abs(scale_safe) < 1e-8;
scale_safe(small_dim) = 1;

% Normalize the forcing term by the spatial scale
s_target = f_target ./ scale_safe;

% 6. Activation & Least Squares Learning
Psi = exp(-h .* (xt(:) - c).^2);   % time_steps x basis_functions
Phi = (Psi ./ sum(Psi, 2)) .* xt(:);

% Solve Linear Least Squares to find optimal weights
w = (Phi \ s_target')';
w(small_dim, :) = 0;

% 7. Pack the Learned Model
dmp.w = w;   % learned weights
dmp.c = c;   % gaussian centers
dmp.h = h;   % gaussian widths
dmp.alpha_z = params.alpha_z;
dmp.beta_z = params.beta_z;
dmp.alpha_x = params.alpha_x;
dmp.tau = tau;   % original timing
dmp.y0 = y0;     % original start
dmp.g = g;       % original goal

end
