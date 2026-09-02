function q = scara_ikine(T, q_prev)
% Analytic inverse kinematics for the SCARA robot
%
% Args:
%   T: 4x4 homogeneous transformation matrix of the desired pose
%   q_prev: (Optional) 4x1 vector of the previous joint configuration
%           Used to maintain elbow branch continuity and unwrap q4
%
% Returns:
%   q: 4x1 vector of computed joint positions [q1; q2; q3; q4]

% Robot constants (matching create_robot.m)
base_z = 2.4;   % linkLengths(1)
a1 = 1;         % linkLengths(2)
a2 = 1;         % linkLengths(3)
d4 = 0.25;      % linkLengths(4)
tool_offset = 0.1 + 0.5*0.2;   % gripperLength + 0.5*fingerLength

% Set default previous joint state if not provided
if nargin < 2, q_prev = [0; pi/2; 0; 0]; end

% Extract position and rotation
px = T(1, 4);
py = T(2, 4);
pz = T(3, 4);
phi = atan2(T(2,1), T(1,1));

% 1. Planar 2R IK (q1, q2): Calculate shoulder and elbow angles
r2 = px^2 + py^2;
c2 = (r2 - a1^2 - a2^2) / (2*a1*a2);
c2 = max(min(c2, 1), -1);    % prevent float errors

% Maintain the same elbow configuration as the previous step
if q_prev(2) >= 0
    s2 = sqrt(1 - c2^2);    % elbow up
else
    s2 = -sqrt(1 - c2^2);   % elbow down
end
q2 = atan2(s2, c2);
q1 = atan2(py, px) - atan2(a2*s2, a1 + a2*c2);

% 2. Prismatic Joint (q3): Calculate vertical extension
q3 = base_z - tool_offset - d4 - pz;

% 3. Wrist Orientation (q4): Calculate final rotation
q4 = q1 + q2 - phi;
q4 = q4 + 2*pi * round((q_prev(4) - q4) / (2*pi));   % prevent 360-degree jumps

% Compile the final joint vector
q = [q1; q2; q3; q4];

end
