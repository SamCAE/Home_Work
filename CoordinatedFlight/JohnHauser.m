function dxdt = JohnHauser(t, x, q, qd, qdd, qddd)
    dxdt = zeros(9,1);
    
    g = [0; 0; 9.81];
     % Controller gains
    K0 = 20; % Proportional gain
    K1 = 20; % Derivative gain
    K2 = 20; % Second derivative gain

    dxdt(1:3) = x(4:6);

    V = norm(x(4:6));
    
    gamma = asin(-x(6)/V);
    xi = atan2(x(5), x(4));

    R1 = [1,        0,       0;
          0,  cos(x(7)), sin(x(7));
          0, -sin(x(7)), cos(x(7))];

    R2 = [cos(gamma), 0, -sin(gamma);
                   0, 1,           0;
          sin(gamma), 0,  cos(gamma)];
    
    R3 = [ cos(xi), sin(xi), 0;
          -sin(xi), cos(xi), 0;
                 0,       0, 1];

    R = R3' * R2' * R1';

    CheckOnly = R(:,1) == [x(4); x(5); x(6)]./V;
    check = all(CheckOnly(:) == 1);

    dxdt(4:6) = g + R * [x(8); 0; x(9)];

    g_w = R' * g;

    w_2 = -(x(9) + g_w(3))/V;
    w_3 = g_w(2)/V;

    % Compute tracking errors
    error = q - x(1:3);              % Position error
    errord = qd - x(4:6);       % Velocity error
    errordd = qdd - dxdt(4:6);    % Acceleration error
    
    % Control law based on differential flatness
    u = qddd + K2 * errordd + K1 * errord + K0 * error;

    % Compute the right-hand side of the control equation
    first_vector = [-w_2 * x(9); 
                     w_3 * x(8) / x(9); 
                     w_2 * x(8)];
    
    second_matrix = [1,         0, 0;
                     0, -1 / x(9), 0;
                     0,         0, 1];
    
    % Final control input computation
    result = first_vector + second_matrix * R' * u;

    dxdt(8) = result(1);
    w_1     = result(2);
    dxdt(9) = result(3);
    
    % Derivatives of angles
    T = [1, sin(x(7))*tan(gamma), cos(x(7))*tan(gamma);
         0,            cos(x(7)),           -sin(x(7));
         0, sin(x(7))/cos(gamma), cos(x(7))/cos(gamma)] * [w_1; w_2; w_3];

    dxdt(7) = T(1);
    gamma_dot = T(2);
    xi_dot = T(3);

end