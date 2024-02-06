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
ic = [qi(1); qi(2); qi(3); qdi(1); qdi(2); qdi(3); 0; 0; -12.5];
% Solve the system of differential equations using the ode45 solver
% The solver calls the JohnHauser function to get the derivatives at each time step
[t, x] = ode45(@(t, x) JohnHauser(t, x, q(t), qd(t), qdd(t), qddd(t)), [0 tf], ic);

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

V = zeros(1, length(t));
w_1 = zeros(1, length(t));
for i = 1:length(t)
    [~, V(i), w_1(i)] = ExtractVarJohnHauser(t(i), x(i,:), q(t(i)), qd(t(i)), qdd(t(i)), qddd(t(i)));
end

a_w1 = x(:,8);
a_w3 = x(:,9);

syms alpha T
rho = 1.225; % Air density [kg/m^3]
S = 27.87; % Wing area [m²]
m = 92e3/9.81; % Aircraft mass [kg]
b = 9.144; % Span [m]
e = 0.7;
AR = b^2/S;
CD0 = 0.02;
Cl0 = 0.1;
Cla = 5.73;
a_L0 = 0.3;
k = 1/(pi*e*AR);

% Your equation definitions
Cl = Cla*alpha + Cl0;
Cd = CD0 + k*Cl^2;

sol_alpha = zeros(size(V));
sol_T = zeros(size(V)); 
for i=1:length(V)
    fprintf('\n%d out of %d',i,length(V));
    a_L = 0.5*rho*V(i)^2 *S*Cl/m + a_L0;
    a_D = 0.5*rho*V(i)^2 *S*Cd/m;
    a_T = T/m;
    eq1 = a_w1(i) == a_T*cos(alpha) - a_D;
    eq2 = a_w3(i) == -a_T*sin(alpha) - a_L;
    
    % Solve for alpha and T
    [sol_alpha(i), sol_T(i)] = vpasolve([eq1, eq2], [alpha, T]);
end
Thrust = vpa(sol_T);
AOA = vpa(sol_alpha);
%%
% Create a new figure window for the subplots
figure;
% Subplot 1: Thrust over Time subplot
subplot(2, 2, 1); 
plot(t, Thrust, 'LineWidth', 2); % Plot Thrust over time
title('Thrust over Time'); % Title
xlabel('Time (s)'); % X-axis label
ylabel('Thrust (N)'); % Y-axis label
grid on; % Enable grid
xlim([0 4*pi]);
%ylim([0 20000]);
% Customize x-axis to show multiples of pi
xticks([0 pi 2*pi 3*pi 4*pi]);
xticklabels({'0', '\pi', '2\pi', '3\pi', '4\pi'});

% Subplot 2: Angle of Attack over Time subplot
subplot(2, 2, 3);
plot(t, AOA*(180/pi), 'LineWidth', 2); % Plot AOA over time, converted to degrees
title('Angle of Attack over Time'); % Title
xlabel('Time (s)'); % X-axis label
ylabel('Angle of Attack (deg)'); % Y-axis label
grid on; % Enable grid
xlim([0 4*pi]);
%ylim([0 4]);
% Customize x-axis to show multiples of pi
xticks([0 pi 2*pi 3*pi 4*pi]);
xticklabels({'0', '\pi', '2\pi', '3\pi', '4\pi'});

% Subplot 3: Magnitude of the Velocity Vector (V) over Time
subplot(2, 2, 2); 
plot(t, V, 'LineWidth', 2); % Plot V over time
title('Magnitude of the Velocity Vector (V) over Time'); % Title
xlabel('Time (s)'); % X-axis label
ylabel('Velocity Magnitude (m/s)'); % Y-axis label
grid on; % Enable grid
xlim([0 4*pi]);
ylim([206 216]);
% Customize x-axis to show multiples of pi
xticks([0 pi 2*pi 3*pi 4*pi]);
xticklabels({'0', '\pi', '2\pi', '3\pi', '4\pi'});

% Subplot 4: Control Parameter (w_1) over Time
subplot(2, 2, 4); 
plot(t, w_1*(180/pi), 'LineWidth', 2); % Plot w_1 over time, converted to degrees
title('Control Parameter (w_1) over Time'); % Title
xlabel('Time (s)'); % X-axis label
ylabel('w_1 (deg/s)'); % Y-axis label
grid on; % Enable grid
xlim([0 4*pi]);
ylim([-150 0]);
% Customize x-axis to show multiples of pi
xticks([0 pi 2*pi 3*pi 4*pi]);
xticklabels({'0', '\pi', '2\pi', '3\pi', '4\pi'});


