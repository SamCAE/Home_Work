function [dxdt, V, w_1] = ExtractVarJohnHauser(t, x, q, qd, qdd, qddd)
% JohnHauser Function for Fixed Wing Coordinated Flight Control Dynamics
    % This function computes the time derivative of the state vector (dxdt)
    % for a flight control system using a given control law. It is designed
    % to simulate the dynamics of an aircraft or similar vehicle under
    % controlled flight conditions.
    %
    % Inputs:
    %   t    - Time (s), scalar. Although not directly used in calculations,
    %          it's included to adhere to the standard function signature for
    %          ODE solvers in MATLAB.
    %   x    - State vector, consisting of 9 elements:
    %          [x(1) = position in east (m), 
    %           x(2) = position in north (m), 
    %           x(3) = altitude (m),
    %           x(4) = velocity east (m/s),    
    %           x(5) = velocity north (m/s), 
    %           x(6) = velocity up (m/s),
    %           x(7) = banking angle (rad), 
    %           x(8) = acceleration in wind frame x-axis (m/s^2),
    %           x(9) = acceleration in wind frame z-axis (m/s^2)].
    %   q    - Desired position vector [east (m), north (m), altitude (m)].
    %   qd   - Desired velocity vector [east (m/s), north (m/s), up (m/s)].
    %   qdd  - Desired acceleration vector [east (m/s^2), north (m/s^2), up (m/s^2)].
    %   qddd - Desired jerk vector (time derivative of acceleration) [east (m/s^3),
    %          north (m/s^3), up (m/s^3)].
    %
    % Outputs:
    %   dxdt - Derivative of the state vector, a 9-element vector:
    %          [derivative of position in east, north, altitude,
    %          derivative of velocity in east, north, up,
    %          derivative of banking angle, derivative of acceleration in
    %          wind frame x and z axes].
    %
    % The function integrates several physical principles, including rotation
    % transformations and control laws, to simulate the behavior of the vehicle
    % in response to the control inputs. It is suitable for use with MATLAB's
    % numerical solvers for ordinary differential equations (ODEs), such as ode45.

    % Initialize the derivative of the state vector
    dxdt = zeros(9,1); % Derivative of state vector

    % Gravity vector in the Earth frame
    g = [0; 0; 9.81];

    % Assign the derivative of the position states directly from the velocity states
    dxdt(1:3) = x(4:6);

    % Calculate the magnitude of the velocity vector
    V = norm(x(4:6));

    % Calculate the flight path angle and heading angle from the velocity components
    gamma = asin(-x(6)/V); % Flight path angle, radians
    xi = atan2(x(5), x(4)); % Heading angle, radians

    % Rotation matrices for transforming between wind and Earth frames
    R1 = [1,        0,       0;
          0,  cos(x(7)), sin(x(7));
          0, -sin(x(7)), cos(x(7))];
    R2 = [cos(gamma), 0, -sin(gamma);
                   0, 1,           0;
          sin(gamma), 0,  cos(gamma)];
    R3 = [ cos(xi), sin(xi), 0;
          -sin(xi), cos(xi), 0;
                 0,       0, 1];
    R = R3' * R2' * R1'; % Combined rotation matrix

    % Calculate the derivative of the velocity states
    dxdt(4:6) = g + R * [x(8); 0; x(9)]; % Acceleration due to gravity and control

    % Gravity components in the wind frame
    g_w = R' * g;

    % Wind frame angular velocities required for control
    w_2 = -(x(9) + g_w(3))/V;
    w_3 = g_w(2)/V;
    
    % Control law based on differential flatness
    u = qddd;

    % Compute the right-hand side of the control equation
    first_vector = [-w_2 * x(9); 
                     w_3 * x(8) / x(9); 
                     w_2 * x(8)];
    second_matrix = [1,         0, 0;
                     0, -1 / x(9), 0;
                     0,         0, 1];
    result = first_vector + second_matrix * R' * u;

    % Update the derivatives of acceleration and angular velocities in the wind frame
    dxdt(8) = result(1);
    w_1     = result(2);
    dxdt(9) = result(3);
    
    % Derivatives of the banking angle, flight path angle, and heading angle
    T = [1, sin(x(7))*tan(gamma), cos(x(7))*tan(gamma);
         0,            cos(x(7)),           -sin(x(7));
         0, sin(x(7))/cos(gamma), cos(x(7))/cos(gamma)] * [w_1; w_2; w_3];

    dxdt(7) = T(1); % Derivative of banking angle
    gamma_dot = T(2); % Derivative of flight path angle (unused)
    xi_dot = T(3); % Derivative of heading angle (unused)

end
