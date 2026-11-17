clear; clc; close all;

% Ensure CasADi is on your MATLAB path (adjust path to where you unzipped CasADi)
% addpath('C:\path\to\casadi-matlab');
import casadi.*
test_dir = fileparts(mfilename('fullpath'));

mpc_dir = fileparts(test_dir);

addpath(mpc_dir);

%% 1. Robot & MPC Parameters
N  = 35;    % 3.5 seconds into the future
dt = 0.1;   % Timestep (10 Hz control loop)

% LEGO Mindstorms physical parameters (used for wheel speed conversion)
wheel_radius = 0.028; % CHANGE
track_width  = 0.120; % CHANGE

% Initial state: [px; py; theta; v; omega]
state = [0.0; 0.0; 0.0; 0.0; 0.0];

% Target parking pose from Perception: [x_center; y_center; theta_bay; v_stop; omega_straight]
target_pose = [1.2; 0.8; pi/2; 0.0; 0.0];
reference_traj = repmat(target_pose, 1, N + 1);

% Initialize MPC
mpc = MPCController(N, dt);

%% 2. Simulation Loop
num_steps = 80; % 8 seconds total
traj      = zeros(5, num_steps + 1);
controls  = zeros(2, num_steps);
wheel_cmd = zeros(2, num_steps); % [omega_L; omega_R] in rad/s for Motion Control

traj(:, 1) = state;

for i = 1:num_steps
    % Solve MPC
    [u_opt, v_target] = mpc.solve(state, reference_traj);
    omega_cmd = u_opt(1);

    % Convert (v_target, omega_cmd) to Left/Right wheel angular velocities (rad/s)
    % This is the exact handoff to Motion Control (Ibrahim)
    v_left  = v_target - (omega_cmd * track_width / 2);
    v_right = v_target + (omega_cmd * track_width / 2);
    wheel_cmd(:, i) = [v_left / wheel_radius; v_right / wheel_radius];

    % Step robot physics forward
    controls(:, i) = u_opt;
    state = rk4_step(state, u_opt, dt);
    traj(:, i + 1) = state;

    fprintf('Step %2d | Pos: (%.2f, %.2f) | Theta: %.2f rad | v: %.2f m/s | w: %.2f rad/s\n', ...
            i, state(1), state(2), state(3), state(4), omega_cmd);
end

%% 3. Plot Results
figure('Name', 'LEGO Mindstorms Box-Parking Trajectory', 'Position', [100, 100, 900, 400]);

% Subplot 1: XY Trajectory + Heading Arrows
subplot(1, 2, 1);
plot(traj(1, :), traj(2, :), 'b-o', 'LineWidth', 1.5, 'MarkerSize', 4); hold on;
quiver(traj(1, 1:4:end), traj(2, 1:4:end), ...
       cos(traj(3, 1:4:end)), sin(traj(3, 1:4:end)), 0.25, 'k', 'LineWidth', 1);
scatter(traj(1, 1), traj(2, 1), 80, 'g', 'filled', 'DisplayName', 'Start');
scatter(target_pose(1), target_pose(2), 100, 'r', 'filled', 'DisplayName', 'Parking Target');
xlabel('X Position (m)');
ylabel('Y Position (m)');
title('Robot Parking Path & Heading');
legend('Trajectory', 'Heading', 'Start', 'Target', 'Location', 'best');
grid on; axis equal;

% Subplot 2: States & Control Inputs
t_state = (0:num_steps) * dt;
t_ctrl  = (0:num_steps-1) * dt;

subplot(3, 2, 2);
plot(t_state, traj(4, :), 'LineWidth', 1.5);
ylabel('Speed v (m/s)'); title('Velocity & MPC Controls'); grid on;

subplot(3, 2, 4);
plot(t_ctrl, controls(1, :), 'r', 'LineWidth', 1.5);
ylabel('Ang. Vel \omega (rad/s)'); grid on;

subplot(3, 2, 6);
plot(t_ctrl, controls(2, :), 'm', 'LineWidth', 1.5);
ylabel('Accel a (m/s^2)'); xlabel('Time (s)'); grid on;