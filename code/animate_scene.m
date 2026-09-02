function animate_scene(scara, q_one, t_one, geo, timing, n_cycles)
% Animates the SCARA robot performing N consecutive assembly cycles
%
% This function renders a 3D environment containing two conveyors (A and B) 
% and visualizes the full pick-and-place sequence (part A -> B)
%
% Args:
%   scara: SerialLink object representing the SCARA robot
%   q_one: 4xN matrix of joint positions for a single cycle
%   t_one: 1xN time vector for a single cycle
%   geo: Struct containing scene geometry and object dimensions
%   timing: Struct containing phase durations
%   n_cycles: (Optional) Number of consecutive cycles to animate, default is 2

if nargin < 6, n_cycles = 2; end

% Determine the number of data points per cycle
T_cycle = timing.T_total;
pts_cycle = round(T_cycle / timing.dt) + 1;

% Prepare joint trajectory (q_full)
if size(q_one, 2) > pts_cycle + 10
    q_full = q_one;   % part 2
else   % 'q_one' contains a single cycle, replicate it
    q_full = repmat(q_one, 1, n_cycles);   %part 1
end

% Prepare scene geometry (geo)
if ~iscell(geo)   % convert to cell array for part 2
    geo_in = geo;
    geo = repmat({geo_in}, 1, n_cycles);
end

% Define key animation trigger times within a single cycle
T2 = timing.t_appr + timing.t_track;   % gripper closes
T5 = T2 + timing.t_trans + timing.t_lower + timing.t_screw + timing.t_hold;   % gripper opens

% Initialize 3D animation figure and workspace
ws = [-1 3 -2 2 0 3];   % workspace limits
fig = figure('Name','SCARA assembly animation','Color', ...
             'w', 'Position', [80 80 1000 750]);
ax = axes('Parent', fig); hold(ax,'on'); grid(ax,'on'); axis(ax,'equal');
xlabel(ax,'x [m]'); ylabel(ax,'y [m]'); zlabel(ax,'z [m]');
view(ax, 35, 25);
xlim(ax, ws(1:2)); ylim(ax, ws(3:4)); zlim(ax, ws(5:6));

% Create a visual clone of the robot and zero out its tool offset.
scara_vis = SerialLink(scara);
if isa(scara_vis.tool, 'SE3')
    scara_vis.tool = SE3();   % newer Toolbox version (v10)
else
    scara_vis.tool = eye(4);  % older Toolbox version (v9)
end

% Use the visual clone for rendering
scara_vis.plot(q_full(:,1).', 'workspace', ws, 'noshadow', ...
           'nobase', 'notiles', 'delay', 0, 'nowrist');
n_skip = 25;   % draw every 25th frame for smooth, real-time playback

for k = 1:n_skip:size(q_full, 2)
    % Safety check: for closed figure, stop the animation
    if ~ishandle(ax)
        break;
    end

    % Clear previous frame's dynamic objects
    delete(findobj(ax, 'Tag', 'sceneObj'));

    % Time and Cycle Tracking
    t_global = (k-1) * timing.dt;
    cycle_idx = min(floor(t_global / T_cycle), n_cycles - 1);
    t_in_cycle = t_global - cycle_idx * T_cycle;

    g = geo{cycle_idx + 1};   % per-cycle geometry
    xB_final = g.beltB_x0 + g.beltB_vx * g.beltB_dur;
    partB_rot = 0;
    if isfield(g, 'partB_rot'), partB_rot = g.partB_rot; end

    % Draw static/moving environment (Conveyors)
    draw_belt(ax, g.beltA_y, g.beltA_z, [-1, 3], [0.85 0.65 0.65]);
    draw_belt(ax, g.beltB_y, g.beltB_z, [-1, 3], [0.65 0.65 0.85]);

    % Forward kinematics (computed once, used for Part A + Gripper)
    T_tcp = scara.fkine(q_full(:,k).');
    if isa(T_tcp, 'SE3'), T_tcp = T_tcp.T; end
    pTCP = T_tcp(1:3, 4);
    phi_tcp = atan2(T_tcp(2,1), T_tcp(1,1));

    % Draw Part A
    if t_in_cycle < T2
        % State 1: Moving on Belt A
        xA = g.beltA_x0 + g.beltA_vx * t_in_cycle;
        draw_partA(ax, [xA; g.beltA_y; g.beltA_z], 0, g);

    elseif t_in_cycle < T5
        % State 2: Attached to the robot gripper
        pA = pTCP - [0; 0; g.partA_offset];
        draw_partA(ax, pA, phi_tcp, g);

    else
        % State 3: Assembled into Part B
        z_bot = g.beltB_z + g.partB_h - g.partB_socket_d;
        draw_partA(ax, [xB_final; g.beltB_y; z_bot], pi/2 + partB_rot, g);
    end

    % Draw Part B
    if t_in_cycle <= g.beltB_dur   % moving
        xB = g.beltB_x0 + g.beltB_vx * t_in_cycle;
    else   % stopped
        xB = xB_final;
    end
    draw_partB(ax, [xB; g.beltB_y; g.beltB_z], g, partB_rot);

    % Draw Dynamic Gripper
    gripper_closed = (t_in_cycle >= T2) && (t_in_cycle < T5);
    draw_gripper(ax, T_tcp, gripper_closed);

    % Draw custom coordinate axes at the real TCP (Frame E)
    draw_axes(ax, T_tcp, 0.4);

    % Update Robot pose
    scara_vis.animate(q_full(:,k).');
    title(ax, sprintf('Cycle %d   t = %.2f s', cycle_idx+1, t_global));
    drawnow;
end
end


% Helper Drawing Functions
function draw_gripper(ax, T_tcp, is_closed)
% Draws the dynamic gripper (body + two fingers)
pTCP = T_tcp(1:3, 4);
phi  = atan2(T_tcp(2,1), T_tcp(1,1));

gColor = [0.6 0.6 0.6];

body_w = 0.25;   % width along perp
body_d = 0.10;   % depth along phi
body_h = 0.10;   % height
finger_len = 0.20;
finger_thick = 0.025;

% Body: wx = body_d (along phi), wy = body_w (along perp)
p_body = [pTCP(1); pTCP(2); pTCP(3) + 0.10];
draw_box(ax, p_body, body_d, body_w, body_h, phi, gColor);

% Perpendicular direction to phi (finger opening axis)
perp_x = -sin(phi);
perp_y = cos(phi);

if is_closed
    % Fingers pointing DOWN from body edges
    offset = body_w/2 + finger_thick/2;
    f_z_bot = pTCP(3) - 0.10;

    for side = [-1, 1]
        fx = pTCP(1) + side * offset * perp_x;
        fy = pTCP(2) + side * offset * perp_y;
        draw_box(ax, [fx; fy; f_z_bot], body_d, finger_thick, finger_len, phi, gColor);
    end

else
    % Fingers HORIZONTAL, extending outward from body along perp
    offset = body_w/2 + finger_len/2;
    f_z_bot = pTCP(3) + 0.10;

    for side = [-1, 1]
        fx = pTCP(1) + side * offset * perp_x;
        fy = pTCP(2) + side * offset * perp_y;
        draw_box(ax, [fx; fy; f_z_bot], body_d, finger_len, finger_thick, phi, gColor);
    end
end
end


function draw_axes(ax, T, len)
% Manually draws RGB coordinate axes for a given pose T
p = T(1:3, 4);      % origin of the frame
R = T(1:3, 1:3);    % rotation matrix

% Define axis colors (Standard: X=Red, Y=Green, Z=Blue)
colors = {'r', 'g', 'b'};
labels = {'X_E', 'Y_E', 'Z_E'};

for i = 1:3
    % Direction vector for the current axis
    dir = R(1:3, i) * len;
    
    % Draw the arrow
    quiver3(ax, p(1), p(2), p(3), dir(1), dir(2), dir(3), ...
            'Color', colors{i}, 'LineWidth', 1.0, 'MaxHeadSize', 1.0, ...
            'AutoScale', 'off', 'Tag', 'sceneObj');
    p_label = p + R(1:3, i) * (len * 1.1);   % label position
    
    % Add text label
    text(p_label(1), p_label(2), p_label(3), labels{i}, ...
         'Color', colors{i}, 'FontWeight', 'bold', 'FontSize', 10, 'Tag', 'sceneObj');
end
end


function draw_belt(ax, yc, zc, x_range, color)
% Draws a flat rectangular patch to represent a conveyor belt
w = 0.30;
xs = [x_range(1), x_range(2), x_range(2), x_range(1)];
ys = [yc-w/2, yc-w/2, yc+w/2, yc+w/2];
zs = [zc, zc, zc, zc];
patch(ax, xs, ys, zs, color, 'FaceAlpha', 0.6, ...
      'EdgeColor','k', 'Tag', 'sceneObj');
end


function draw_partA(ax, p, rot, geo)
% Draws Part A (the peg), p = bottom-center of the screw
% Lower part: Cylindrical peg
draw_cylinder(ax, p, geo.partA_screw_w/2, geo.partA_screw_h, ...
              rot, [0.55 0.30 0.10]);

% Upper part: Square Block (Head)
p_head = p + [0; 0; geo.partA_screw_h];
draw_box(ax, p_head, geo.partA_head_w, geo.partA_head_w, ...
              geo.partA_head_h, rot, [1.00 0.55 0.10]);
end


function draw_partB(ax, p, geo, partB_rot)
% Draws Part B (the socket block), p = bottom-center
if nargin < 4, partB_rot = 0; end

% Rectangular body
draw_box(ax, p, geo.partB_w, geo.partB_w, geo.partB_h, partB_rot, [0.25 0.35 0.75]);

% Cylindrical socket (dark disc inside the top)
sx = p(1) + 0.00 * cos(partB_rot);
sy = p(2) + 0.00 * sin(partB_rot);
z_hole = p(3) + geo.partB_h + 0.002;
draw_disc(ax, sx, sy, z_hole, geo.partB_socket_w / 2, ...
          0, [0.10 0.10 0.20]);
end


function draw_box(ax, p, wx, wy, h, rot, color)
% Draws a 3D rectangular box centered at p(x,y)
hw = wx/2;
hd = wy/2;

% Define the 8 vertices of the box (centered at origin)
x_base = [-hw, hw, hw, -hw, -hw, hw, hw, -hw]';
y_base = [-hd, -hd, hd, hd, -hd, -hd, hd, hd]';
z_base = [0, 0, 0, 0, h, h, h, h]';

% Apply 2D rotation to the XY coordinates
[xr, yr] = rotate_xy(x_base, y_base, rot);

% Shift coordinates to the target position p
V = [xr + p(1), yr + p(2), z_base + p(3)];

% Define the 6 rectangular faces connecting the vertices
F = [1 2 3 4;    % 1: bottom
     5 6 7 8;    % 2: top
     1 2 6 5;    % 3: front
     2 3 7 6;    % 4: right
     3 4 8 7;    % 5: back
     4 1 5 8];   % 6: left

% Create a "shading" effect array for 3D depth
face_colors = [color * 0.70;    % bottom
               color;           % top
               color * 0.85;    % side 1
               color * 0.85;    % side 2
               color * 0.85;    % side 3
               color * 0.85];   % side 4
patch(ax, 'Vertices', V, 'Faces', F, 'FaceVertexCData', face_colors, ...
      'FaceColor', 'flat', 'EdgeColor', 'k', 'Tag', 'sceneObj');
end


function draw_cylinder(ax, p, r, h, rot, color)
% Draws a solid 3D cylinder
n = 24;
[X, Y, Z] = cylinder(r, n);
Z = Z * h;
[X, Y] = rotate_xy(X, Y, rot);
surf(ax, X+p(1), Y+p(2), Z+p(3), 'FaceColor', color, ...
     'EdgeColor','none', 'Tag','sceneObj');

% Add top and bottom caps
draw_disc(ax, p(1), p(2), p(3),     r, rot, color);
draw_disc(ax, p(1), p(2), p(3)+h,   r, rot, color);
end


function draw_disc(ax, x0, y0, z0, r, rot, color)
% Draws a filled 2D polygon (circle)
n = 24;
th = linspace(0, 2*pi, n+1);
xs = r*cos(th);
ys = r*sin(th);
[xs, ys] = rotate_xy(xs, ys, rot);
patch(ax, xs+x0, ys+y0, z0*ones(size(xs)), color, ...
      'EdgeColor','none', 'Tag','sceneObj');
end

function [Xr, Yr] = rotate_xy(X, Y, ang)
% Applies 2D rotation matrix to X and Y coordinates
c = cos(ang); s = sin(ang);
Xr = c*X - s*Y;
Yr = s*X + c*Y;
end