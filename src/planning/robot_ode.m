% state vector consists of [x, y, theta, v, ang_vel]
% x pos, y pos, theta angle, velocity, angular vel (yaw_rate)

% control vector linear accel and angular accel (change in yaw rate) [a,
% ang_a]


% ode ordinary differential equation
function output = robot_ode(state, control)

    % State variables
    x     = state(1);
    y     = state(2);
    theta = state(3);
    v     = state(4);
    omega = state(5);

    % Control inputs
    a     = control(1);
    alpha = control(2);

    % Differential drive equations
    dx     = v * cos(theta);
    dy     = v * sin(theta);
    dtheta = omega;
    dv     = a;
    domega = alpha;

    % State derivatives
    output = [dx; dy; dtheta; dv; domega];

end