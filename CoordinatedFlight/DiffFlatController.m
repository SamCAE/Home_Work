function [dU1, U2] = DiffFlatController(q, qd, qdd, qddd, x, X, dX, a_w)
    % DIFFFLATCONTROLLER Computes the control input derivatives for a fixed-wing aircraft.
    %
    % This function calculates the derivatives of control inputs required to
    % achieve desired flight dynamics based on differential flatness.
    %
    % Inputs:
    %   q, qd, qdd, qddd - Desired position, velocity, acceleration, and jerk in the trajectory.
    %   x                - Current position.
    %   X                - Current state vector, [Vx; Vy; Vz; zeta; gamma; mu].
    %   dX               - Current state derivatives.
    %   a_w              - Wind acceleration vector.
    %
    % Outputs:
    %   dU1 - Derivative of control inputs [a_wx_dot; a_wz_dot].
    %   U2  - Control input (angular rate wind x-direction).

    % Controller gains
    K0 = 20; % Proportional gain
    K1 = 20; % Derivative gain
    K2 = 20; % Second derivative gain

    % Unpack state variables
    zeta = X(4);   % Heading angle
    gamma = X(5);  % Flight path angle
    mu = X(6);     % Bank angle

    % Obtain transformation matrix from inertial to wind frame
    R = inertial_to_wind(zeta, gamma, mu);

    % Compute tracking errors
    error = q - x;              % Position error
    errord = qd - X(1:3);       % Velocity error
    errordd = qdd - dX(1:3);    % Acceleration error
    
    % Control law based on differential flatness
    u = qddd + K2 * errordd + K1 * errord + K0 * error;

    % Compute the right-hand side of the control equation
    first_vector = [-dX(5) * a_w(2); 
                     dX(4) * a_w(1) / a_w(2); 
                     dX(5) * a_w(1)];
    
    second_matrix = [1,           0, 0;
                     0, -1 / a_w(2), 0;
                     0,           0, 1];
    
    % Final control input computation
    result = first_vector + second_matrix * R * u;

    % Extract control input derivatives
    a_wx_dot = result(1);     % Derivative of wind acceleration in x-direction
    ang_rate_x = result(2);   % Angular rate in x-direction
    a_wz_dot = result(3);     % Derivative of wind acceleration in z-direction

    % Output
    dU1 = [a_wx_dot; a_wz_dot];
    U2 = ang_rate_x;
end