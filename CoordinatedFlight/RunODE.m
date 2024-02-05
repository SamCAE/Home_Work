clear; clc; close all;

% waypoints = [
%     0 2 10;    % X-coordinates (EAST)
%     0 5 10;     % Y-coordinates (NORTH)
%     0 5 5;       % Z-coordinates (UP)
% ];
% 
% % Trajectory settings
% tf = 10; % Final time of the simulation (seconds)
% % Time settings
% dt = 0.01; % Time step
% time = 0:dt:tf;
% numWaypoints = size(waypoints,2);  % Number of waypoints
% numSamples = length(time);
% % Define the time points for each waypoint
% timePoints = linspace(0, tf, numWaypoints);
% 
% [q,qd,qdd,qddd,pp,tPoints,tSamples] = minjerkpolytraj(waypoints,timePoints,numSamples);

% Trajectory settings
tf = 4*pi; % Final time of the simulation (seconds)
% Time settings
dt = 0.01; % Time step
time = 0:dt:tf;
numSamples = length(time);

q = [210*time; 50*sin(time/2); 50*cos(time/2)];
qd = [210*ones(1,numSamples); 25*cos(time/2); -25*sin(time/2)];
qdd = [0*ones(1,numSamples); -12.5*sin(time/2); -12.5*cos(time/2)];
qddd = [0*ones(1,numSamples); -6.25*cos(time/2); 6.25*sin(time/2)];

[t, x] = ode15s(@(t, x) JohnHauser(t, x, q, qd, qdd, qddd), time,[q(1,1); q(2,1); q(3,1); qd(1,1); qd(2,1); qd(3,1); 1; 1; 1]);

figure;
hold on; grid on;

% Plot desired trajectory
plot3(q(1,:), q(2,:), q(3,:), 'b--', 'LineWidth', 2);

% Plot actual trajectory
plot3(x(:,1), x(:,2), x(:,3), 'r-', 'LineWidth', 2);

% Adding labels and legend
xlabel('X-axis (East)');
ylabel('Y-axis (North)');
zlabel('Z-axis (Up)');
title('Comparison of Desired and Actual Trajectories');
legend('Desired Trajectory', 'Actual Trajectory');

% Setting axes for better visualization
axis equal;

hold off;

% figure;
% hold on; grid on;
% 
% % Plot desired trajectory
% plot3(qd(1,:), qd(2,:), qd(3,:), 'b--', 'LineWidth', 2);
% 
% % Plot actual trajectory
% plot3(x(:,4), x(:,5), x(:,6), 'r-', 'LineWidth', 2);
% 
% % Adding labels and legend
% xlabel('X-axis (East)');
% ylabel('Y-axis (North)');
% zlabel('Z-axis (Up)');
% title('Comparison of Desired and Actual Trajectories');
% legend('Desired Trajectory', 'Actual Trajectory');
% 
% % Setting axes for better visualization
% axis equal;
% 
% hold off;