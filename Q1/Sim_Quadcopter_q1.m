% Differential drive robot
clc;
clear all;
close all;

% Simulation parameters
TOTAL_TIME  = 50;
dt          = 0.1;
TIME_SCALE  = 0.001; % slows down simulation if > 1, speeds up if < 1 (and if computation allows...)


% Initialise plot
figure;
ax1 = axes;
hold(ax1,'on');
view(ax1, 3);
axis('equal')
axis([-5 5 -5 5 0 10])
axis('manual')
xlabel('x');
ylabel('y');
ylabel('z');
axis vis3d
grid ON
grid MINOR
ax1.Toolbar.Visible = 'off';
ax1.Interactions = [];
counter = 0
% Initialise Simulation
drone1 = Quadcopter(ax1);
% mhj: Initialise
ref = zeros(12,1);

Q = 1.1; % 1.1, 1.2 or 1.3
Q1_init;


result_position=[];
    

for t = 0:dt:TOTAL_TIME
    tic
    cla
    
    % _______ IMPLEMENT CONTROLLER + SIMULATION PHYSICS HERE ______ %
    % Q1:
    [theta_now, x_now, xdot, thetadot] = drone1.update(theta_now, x_now, xdot, thetadot, dt, Q);
    %[theta_now, x_now, xdot, omega] = drone1.q2_update(t, x_now, xdot, theta_now, omega, dt, Aa, Ba, ref, Q);
    result_position=[result_position,x_now];
    drone1.plot;
    % _______ IMPLEMENT CONTROLLER + SIMULATION PHYSICS HERE ______ %
    drawnow nocallbacks limitrate
    pause(5*TIME_SCALE*dt-toc); 
end
