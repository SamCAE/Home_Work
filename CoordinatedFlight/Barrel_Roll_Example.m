clear; clc; close all;

% Trajectory settings
tf = 4*pi; % Final time of the simulation (seconds)
% Time settings
dt = 0.01; % Time step
time = 0:dt:tf;
numSamples = length(time);

% Initial conditions
x = [0; 0; 50]; % Initial position
X = [210; 25; 0; 0; 0; 0]; % Initial state vector [Vx; Vy; Vz; zeta; gamma; mu]
dX = [0; 0; -12.5; 0; 0; 0]; % Initial state derivatives
a_w = [0; -12.5]; % Initial wind acceleration

% Initialize arrays for storing simulation data
X_data = zeros(6, numSamples);
x_data = zeros(3, numSamples);
q = zeros(3, numSamples);
qd = zeros(3, numSamples);
qdd = zeros(3, numSamples);
qddd = zeros(3, numSamples);
U2_data = zeros(1, numSamples); % Initialize array for storing U2 data

for i = 1:numSamples
    % Call the controller
    q(:,i) = [210*time(i); 50*sin(time(i)/2); 50*cos(time(i)/2)];
    qd(:,i) = [210; 25*cos(time(i)/2); -25*sin(time(i)/2)];
    qdd(:,i) = [0; -12.5*sin(time(i)/2); -12.5*cos(time(i)/2)];
    qddd(:,i) = [0; -6.25*cos(time(i)/2); 6.25*sin(time(i)/2)];

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
    U2_data(i) = U2; % Store U2 value
end

% Plotting the trajectories
figure;
subplot(2,1,1); % Plot trajectories in the first subplot
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

subplot(2,1,2); % Plot U2 in the second subplot
plot(time, U2_data, 'g-', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('U2');
title('Control Input U2 Over Time');
grid on;
xlim([0 4*pi]);