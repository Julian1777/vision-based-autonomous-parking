function next_state = rk4_step(state, control, dt)
    % k1: Estimate the initial slope

    k1 = robot_ode(state, control);

    % k2: Estimate the slope at the midpoint using k1

    k2 = robot_ode(state + (dt/2) * k1, control);

    % k3: Improve the midpoint estimate using k2

    k3 = robot_ode(state + (dt/2) * k2, control);

    % k4: Estimate the slope at the endpoint using k3

    k4 = robot_ode(state + dt * k3, control);

    % Calculate the next state using RK4 weighted average

    next_state = state + (dt/6) * (k1 + 2*k2 + 2*k3 + k4);

end