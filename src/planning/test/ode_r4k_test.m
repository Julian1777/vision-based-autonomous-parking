

% Test robot_ode and rk4_step functions

clc;

clear;

close all;

test_dir = fileparts(mfilename('fullpath'));

planning_dir = fileparts(test_dir);

addpath(planning_dir);

% Simulation parameters

dt = 0.1;

% Initial state [x, y, v, theta, omega]

state = [0.0; 0.0; 0.0; 15.0; 0.0];

% Control inputs [a, alpha]

% a     = linear acceleration

% alpha = angular acceleration

control = [0.03; 0.0];

% Display initial state

disp('Initial state:');

disp(state);

% Perform one RK4 integration step

next_state = rk4_step(state, control, dt);

% Display resulting state

disp('State after one time step:');

disp(next_state);