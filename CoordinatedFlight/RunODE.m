% Clear the workspace, command window, and close all figures
clear; clc; close all;

% Define the final time of the simulation in seconds
tf = 4*pi; 

% Define the desired trajectory as a function of time, t
% q, qd, qdd, and qddd represent the desired position, velocity, acceleration,
% and jerk vectors of the trajectory, respectively.
q = @(t) [210*t; 50*sin(t/2); 50*cos(t/2)]; % Position as a function of time
qd = @(t) [210; 25*cos(t/2); -25*sin(t/2)]; % Velocity as a function of time
qdd = @(t) [0; -12.5*sin(t/2); -12.5*cos(t/2)]; % Acceleration as a function of time
qddd = @(t) [0; -6.25*cos(t/2); 6.25*sin(t/2)]; % Jerk as a function of time

% Initial conditions for position and velocity
qi = q(0); % Initial position
qdi = qd(0); % Initial velocity

% Solve the system of differential equations using the ode45 solver
% The solver calls the JohnHauser function to get the derivatives at each time step
[t, x] = ode45(@(t, x) JohnHauser(t, x, q(t), qd(t), qdd(t), qddd(t)), [0 tf], [qi(1); qi(2); qi(3); qdi(1); qdi(2); qdi(3); 0; 0; -12.5]);

% Generate a set of 100 evenly spaced time points from 0 to final time for plotting
tSpan = linspace(0, tf, 100);

% Compute the desired trajectory at each time point in tSpan
qDesired = arrayfun(@(t) q(t), tSpan, 'UniformOutput', false);
qDesiredMat = cell2mat(qDesired)'; % Convert the array of cells to a matrix for easy plotting

% Extract the actual position trajectory from the solver's output
xActual = x(:, 1:3);

% Interpolate the actual trajectory data to match the desired time points for plotting
% This is necessary because ode45 uses adaptive time-stepping
xActualInterp = interp1(t, xActual, tSpan);

% Plot the desired trajectory in red solid line
plot3(qDesiredMat(:,1), qDesiredMat(:,2), qDesiredMat(:,3), 'r-', 'LineWidth', 2);
hold on; % Keep the figure open to overlay the actual trajectory plot

% Plot the actual trajectory in blue dashed line
plot3(xActualInterp(:,1), xActualInterp(:,2), xActualInterp(:,3), 'b--', 'LineWidth', 2);

% Add plot decorations
legend('Desired Trajectory', 'Actual Trajectory'); % Add a legend
title('Desired vs Actual Trajectory'); % Add a title
xlabel('X'); % Label the x-axis
ylabel('Y'); % Label the y-axis
zlabel('Z'); % Label the z-axis
grid on; % Enable the grid for better visualization
hold off; % Close the plotting context
