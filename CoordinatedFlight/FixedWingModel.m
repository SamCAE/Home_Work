function dX = FixedWingModel(U, X)
    % FIXEDWINGMODEL Computes the derivative of the state for a fixed-wing aircraft.
    %
    %   Inputs:
    %       U - Control inputs vector, [a_wx; a_wz; w_wx]
    %       X - Current state vector, [Vx; Vy; Vz; zeta; gamma; mu]
    %
    %   Outputs:
    %       dX - Derivative of the state vector

    % Unpack control inputs
    accel_wind_x = U(1);  % Acceleration in wind x direction
    accel_wind_z = U(2);  % Acceleration in wind z direction

    % Unpack state variables
    zeta = X(4);          % Heading angle
    gamma = X(5);         % Flight path angle
    mu = X(6);            % Bank angle
    V = norm(X(1:3));     % Velocity magnitude

    % Transformation matrix from inertial to wind frame
    R = inertial_to_wind(zeta, gamma, mu);

    % Gravity vector
    g = [0; 0; 9.81];

    % Gravity in the wind frame
    gravity_wind = R * g;

    % Angular rates calculation
    ang_rate_x = U(3);                % Angular rate in wind x direction
    ang_rate_y = -(accel_wind_z + gravity_wind(3)) / V; % Angular rate in wind y direction
    ang_rate_z = gravity_wind(2) / V; % Angular rate in wind z direction

    % Aircraft acceleration in inertial frame
    acceleration_inertial = g + R' * [accel_wind_x; 0; accel_wind_z];

    % Derivatives of angles
    T = [1, sin(mu)*tan(gamma), cos(mu)*tan(gamma);
         0,            cos(mu),           -sin(mu);
         0, sin(mu)/cos(gamma), cos(mu)/cos(gamma)] * [ang_rate_x; ang_rate_y; ang_rate_z];

    mu_dot = T(1);
    gamma_dot = T(2);
    zeta_dot = T(3);

    % Construct derivative of state
    dX = [acceleration_inertial; zeta_dot; gamma_dot; mu_dot];
end