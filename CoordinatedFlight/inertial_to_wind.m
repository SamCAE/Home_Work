function R = inertial_to_wind(zeta, gamma, mu)
    % INERTIAL_TO_WIND Computes the transformation matrix from inertial to wind frame.
    %
    %   Inputs:
    %       zeta  - Heading angle
    %       gamma - Flight path angle
    %       mu    - Bank angle
    %
    %   Outputs:
    %       R - Transformation matrix from inertial to wind frame

    % Rotation matrices for each angle
    R1 = [1,        0,       0;
          0,  cos(mu), sin(mu);
          0, -sin(mu), cos(mu)];

    R2 = [cos(gamma), 0, -sin(gamma);
                   0, 1,           0;
          sin(gamma), 0,  cos(gamma)];

    R3 = [ cos(zeta), sin(zeta), 0;
          -sin(zeta), cos(zeta), 0;
                   0,         0, 1];

    % Combined rotation matrix
    R = R1 * R2 * R3;
end