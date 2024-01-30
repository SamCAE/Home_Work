clear; clc; close all;

waypoints = [
    0 5 10;    % X-coordinates (EAST)
    0 5 10;     % Y-coordinates (NORTH)
    0 5 10;       % Z-coordinates (UP)
];

% Trajectory settings
tf = 10; % Final time of the simulation (seconds)
% Time settings
dt = 0.01; % Time step
time = 0:dt:tf;
numWaypoints = size(waypoints,2);  % Number of waypoints
numSamples = length(time);
% Define the time points for each waypoint
timePoints = linspace(0, tf, numWaypoints);

[q,qd,qdd,qddd,pp,tPoints,tSamples] = minjerkpolytraj(waypoints,timePoints,numSamples);

% Initial conditions
x = [0; 0; 0]; % Initial position
X = [0.1; 0.1; 0.1; 0; 0; 0]; % Initial state vector [Vx; Vy; Vz; zeta; gamma; mu]
dX = [0; 0; 0; 0; 0; 0]; % Initial state derivatives
a_w = [0.1; 0.1]; % Initial wind acceleration

% Initialize arrays for storing simulation data
X_data = zeros(6, numSamples);
x_data = zeros(3, numSamples);

for i = 1:numSamples
    % Call the controller
    [dU1, U2] = DiffFlatController(q(:,i), qd(:,i), qdd(:,i), qddd(:,i), x, X, dX, a_w);

    % Integrate dU1 to get a_w
    a_w = a_w + dU1 * dt;

    % Construct control input for the model
    U = [a_w; U2];

    % Call the model
    dX = FixedWingModel(U, X);

    % Integrate dX to get X
    X = X + dX * dt;

    % Integrate first 3 elements of X to get x
    x = x + X(1:3) * dt;

    % Store data for plotting
    X_data(:, i) = X;
    x_data(:, i) = x;
end

% Plotting the trajectories
figure;
hold on; grid on;

% Plot desired trajectory
plot3(q(1,:), q(2,:), q(3,:), 'b--', 'LineWidth', 2);

% Plot actual trajectory
plot3(x_data(1,:), x_data(2,:), x_data(3,:), 'r-', 'LineWidth', 2);

% Adding labels and legend
xlabel('X-axis (East)');
ylabel('Y-axis (North)');
zlabel('Z-axis (Up)');
title('Comparison of Desired and Actual Trajectories');
legend('Desired Trajectory', 'Actual Trajectory');

% Setting axes for better visualization
axis equal;

hold off;

