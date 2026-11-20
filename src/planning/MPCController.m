classdef MPCController < handle
    properties
        opti
        N
        dt
        track_width % MEASURE distance between left and right wheels
        X
        U
        x0
        x_ref
        u_prev
    end

    methods
        function obj = MPCController(N, dt, track_width)
            import casadi.*

            obj.opti = casadi.Opti();
            obj.N = N;
            obj.dt = dt;
            obj.track_width = track_width;

            % Decision variables
            obj.X = obj.opti.variable(5, N + 1); % [px; py; v; theta; omega]
            obj.U = obj.opti.variable(2, N);     % [a; alpha] across N steps

            % Parameters passed at each timestep
            obj.x0    = obj.opti.parameter(5, 1);     % Current robot state
            obj.x_ref = obj.opti.parameter(5, N + 1); % Reference parking trajectory
            obj.u_prev = obj.opti.parameter(2, 1);     % Previous control input a, alpha

            % Build optimization problem
            obj.add_constraints();
            obj.set_cost();

            % Configure IPOPT solver (silent output for fast loop execution)
            opts = struct();
            opts.ipopt.print_level = 0;
            opts.print_time = 0;
            obj.opti.solver('ipopt', opts);
        end

        function add_constraints(obj)

            l_half = obj.track_width / 2;

            v_wheel_max = 0.30; % m/s
            a_wheel_max = 0.45; % m/s^2

            % Anchor initial predicted state to current measured state
            obj.opti.subject_to(obj.X(:, 1) == obj.x0);

            % Enforce robot dynamics across the horizon
            for k = 1:obj.N
                next_state = rk4_step(obj.X(:, k), obj.U(:, k), obj.dt);
                obj.opti.subject_to(obj.X(:, k + 1) == next_state);
            end
        
            % Coupled diff drive wheel velocity limits:
            v = obj.X(3, :);
            omega = obj.X(5, :);
            % Ensure the robot doesn't exceed its wheel velocity limits
            obj.opti.subject_to(-v_wheel_max <= v + l_half * omega <= v_wheel_max);
            obj.opti.subject_to(-v_wheel_max <= v - l_half * omega <= v_wheel_max);

            % Coupled diff drive wheel acceleration limits:
            a = obj.U(1, :);
            alpha = obj.U(2, :);
            obj.opti.subject_to(-a_wheel_max <= a + l_half * alpha <= a_wheel_max);
            obj.opti.subject_to(-a_wheel_max <= a - l_half * alpha <= a_wheel_max);

            % standalone bounds  for linear and angular accel
            obj.opti.subject_to(-2.0 <= omega <= 2.0);
            obj.opti.subject_to(-3.0 <= alpha <= 3.0);
        end

        function set_cost(obj)
            % State weights: [px, py, v, theta, omega]
            Q = diag([60.0, 60.0, 0.8, 1.0, 0.5]);
            % Control effort weights: [a, alpha]
            R = diag([0.5, 0.5]);

            % Jerk (control rate) penalty weights, delta_a, delta_alpha. higher values smoother curves
            R_jerk = diag([2500.0, 100.0]);

            total_cost = 0;
            for k = 1:obj.N
                state_error = obj.X(:, k) - obj.x_ref(:, k);
                u_k = obj.U(:, k);

                % Jerk term: difference between consecutive accelerations
                if k == 1
                    du = u_k - obj.u_prev;
                else
                    du = u_k - obj.U(:, k - 1);
                end

                total_cost = total_cost ...
                           + state_error' * Q * state_error ...
                           + u_k' * R * u_k ...
                           + du' * R_jerk * du;
            end

            % Final step state cost
            final_error = obj.X(:, obj.N + 1) - obj.x_ref(:, obj.N + 1);
            total_cost = total_cost + final_error' * Q * final_error;

            obj.opti.minimize(total_cost);
        end

        function [u_opt, v_target] = solve(obj, current_state, reference_traj, prev_u)
            if nargin < 4
                prev_u = [0; 0]; % Default previous control input if not provided
            end

            % Set parameter values
            obj.opti.set_value(obj.x0, current_state);
            obj.opti.set_value(obj.x_ref, reference_traj);
            obj.opti.set_value(obj.u_prev, prev_u);

            % Warm-start X at the current state so IPOPT doesn't start at all-zeros
            obj.opti.set_initial(obj.X, repmat(current_state, 1, obj.N + 1));
            obj.opti.set_initial(obj.U, zeros(2, obj.N));

            % Solve NLP
            sol = obj.opti.solve();
            

            % Extract optimal control sequence and predicted states
            U_sol = full(sol.value(obj.U));
            X_sol = full(sol.value(obj.X));

            % First optimal control input [a, alpha]
            u_opt = U_sol(:, 1);

            % Next desired linear velocity
            v_target = X_sol(3, 2);
        end
    end
end