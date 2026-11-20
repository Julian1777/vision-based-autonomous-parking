clear; clc; close all;

import casadi.*
test_dir = fileparts(mfilename('fullpath'));

mpc_dir = fileparts(test_dir);

addpath(mpc_dir);

N  = 35;    % 3.5 seconds into the future
dt = 0.1;   % Timestep (10 Hz control loop)
track_width = 0.120; % CHANGE

% LEGO Mindstorms physical parameters (used for wheel speed conversion)
wheel_radius = 0.028; % CHANGE
track_width  = 0.120; % CHANGE

% Initial state: [px; py; v; theta; omega] robot facing left
state = [0.0; 0.0; 0.0; pi; 0.0];

% coords returned by detect_parking_space.m 4x2 coordinate matrix and the heading
bay_corners = [-0.95, 0.25;   % Front-left
               -0.65, 0.25;   % Front-right
               -0.65, 0.65;   % Back-right
               -0.95, 0.65];  % Back-left

% Calculate the center of the parking space
x_center = mean(bay_corners(:, 1));
y_center = mean(bay_corners(:, 2));

theta_bay = -pi/2; 

% traget pose
target_pose = [x_center; y_center; 0.0; theta_bay; 0.0];
reference_traj = repmat(target_pose, 1, N + 1);

% Initialize MPC
mpc = MPCController(N, dt, track_width);


num_steps = 120; % 8 seconds total
traj      = zeros(5, num_steps + 1);
controls  = zeros(2, num_steps);
wheel_cmd = zeros(2, num_steps); % [omega_L; omega_R] in rad/s for Motion Control

traj(:, 1) = state;

u_prev = [0; 0]; % Initialize previous control input

for i = 1:num_steps
    % Solve MPC
    [u_opt, v_target] = mpc.solve(state, reference_traj, u_prev);
   omega_cmd = state(5) + u_opt(2) * dt;

    % Convert (v_target, omega_cmd) to Left/Right wheel angular velocities (rad/s)
    v_left  = v_target - (omega_cmd * track_width / 2);
    v_right = v_target + (omega_cmd * track_width / 2);
    wheel_cmd(:, i) = [v_left / wheel_radius; v_right / wheel_radius];

    % Step robot physics forward
    controls(:, i) = u_opt;
    state = rk4_step(state, u_opt, dt);
    traj(:, i + 1) = state;

    u_prev = u_opt;

    fprintf('Step %2d | Pos: (%.2f, %.2f) | Theta: %.2f rad | v: %.2f m/s | w: %.2f rad/s\n', ...
            i, state(1), state(2), state(3), state(4), omega_cmd);
end

% plot results
robot_length = 0.20; % m (adjust to your Mindstorms chassis length)
robot_width  = 0.15; % m (adjust to your Mindstorms chassis width)

figure('Name', 'LEGO Mindstorms Box-Parking Trajectory', 'Position', [100, 100, 900, 400]);

% XY Trajectory + Parking Lot + Robot Footprint + Heading Arrows
subplot(1, 2, 1); hold on;

% Target parking bay & adjacent spots (from 4 perceived corners)
fill(bay_corners(:, 1), bay_corners(:, 2), [0.85, 0.95, 0.85], ...
     'EdgeColor', [0.1, 0.6, 0.1], 'LineWidth', 2, 'DisplayName', 'Target Bay');

bay_w     = norm(bay_corners(2, :) - bay_corners(1, :));
left_bay  = bay_corners - [bay_w, 0];
right_bay = bay_corners + [bay_w, 0];
fill(left_bay(:, 1),  left_bay(:, 2),  [0.95, 0.85, 0.85], ...
     'EdgeColor', 'r', 'LineStyle', '--', 'DisplayName', 'Adjacent Spots');
fill(right_bay(:, 1), right_bay(:, 2), [0.95, 0.85, 0.85], ...
     'EdgeColor', 'r', 'LineStyle', '--', 'HandleVisibility', 'off');

% Rotated robot box footprint every 10 steps + final resting pose
L = robot_length / 2;
W = robot_width / 2;
local_box = [-L, -W; L, -W; L, W; -L, W; -L, -W]';

plot_steps = unique([1:10:size(traj, 2), size(traj, 2)]);
for idx = plot_steps
    px = traj(1, idx);
    py = traj(2, idx);
    th = traj(4, idx);
    R_mat = [cos(th), -sin(th); sin(th), cos(th)];
    world_box = R_mat * local_box + [px; py];

    if idx == size(traj, 2)
        plot(world_box(1, :), world_box(2, :), 'b-', 'LineWidth', 2, 'DisplayName', 'Final Robot Pose');
    else
        plot(world_box(1, :), world_box(2, :), 'Color', [0.5, 0.5, 0.8], 'LineWidth', 1, 'HandleVisibility', 'off');
    end
end

% Center trajectory, heading arrows, start, and target
plot(traj(1, :), traj(2, :), 'b-o', 'LineWidth', 1.5, 'MarkerSize', 4, 'DisplayName', 'Trajectory');
scatter(traj(1, 1), traj(2, 1), 80, 'g', 'filled', 'DisplayName', 'Start');
scatter(target_pose(1), target_pose(2), 100, 'r', 'filled', 'DisplayName', 'Parking Target');

xlabel('X Position (m)');
ylabel('Y Position (m)');
title('Robot Parking Path & Heading');
legend('Location', 'best');
grid on; axis equal;

% States & Control Inputs
t_state = (0:num_steps) * dt;
t_ctrl  = (0:num_steps-1) * dt;

subplot(3, 2, 2);
plot(t_state, traj(3, :), 'LineWidth', 1.5);
ylabel('Speed v (m/s)'); title('Velocity & MPC Controls'); grid on;

subplot(3, 2, 4);
plot(t_ctrl, controls(1, :), 'r', 'LineWidth', 1.5);
ylabel('Accel a (m/s^2)'); grid on;

subplot(3, 2, 6);
plot(t_ctrl, controls(2, :), 'm', 'LineWidth', 1.5);
ylabel('Ang. Accel \alpha (rad/s^2)'); xlabel('Time (s)'); grid on;