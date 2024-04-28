% Differential drive robot
clc;
clear all;
close all;

% Simulation parameters
TOTAL_TIME  = 5;
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
% Initialise Simulation
drone1 = Quadcopter(ax1);


figure;
ax2 = axes;
hold(ax2,'on');
view(ax2, 3);
axis('equal')
axis([-5 5 -5 5 0 10])
axis('manual')
xlabel('x');
ylabel('y');
ylabel('z');
axis vis3d
grid ON
grid MINOR
ax2.Toolbar.Visible = 'off';
ax2.Interactions = [];
% Initialise Simulation
drone2 = Quadcopter(ax2);

figure;
ax3 = axes;
hold(ax3,'on');
view(ax3, 3);
axis('equal')
axis([-5 5 -5 5 0 10])
axis('manual')
xlabel('x');
ylabel('y');
ylabel('z');
axis vis3d
grid ON
grid MINOR
ax3.Toolbar.Visible = 'off';
ax3.Interactions = [];
% Initialise Simulation
drone3 = Quadcopter(ax3);

figure;
ax4 = axes;
hold(ax4,'on');
view(ax4, 3);
axis('equal')
axis([-5 5 -5 5 0 10])
axis('manual')
xlabel('x');
ylabel('y');
ylabel('z');
axis vis3d
grid ON
grid MINOR
ax4.Toolbar.Visible = 'off';
ax4.Interactions = [];
% Initialise Simulation
drone4 = Quadcopter(ax4);
% mhj: Initialise
ref = zeros(12,1);

figure;
ax = axes;
hold(ax,'on');
view(ax, 3);
axis('equal')
axis([-5 5 -5 5 0 10])
axis('manual')
xlabel('x');
ylabel('y');
ylabel('z');
axis vis3d
grid ON
grid MINOR
ax.Toolbar.Visible = 'off';
ax.Interactions = [];
% Initialise Simulation
drone = Quadcopter(ax);
Q = 1; % 1, 2 or 3
small_error = 0.01; % 0.01 or 0.1
large_error = 0.5;
Q2_init;
Q2;

result_position = [x_now_1];

result_position_1=[x_now_1];
result_position_2=[x_now_2];

result_position_3=[x_now_3];
result_position_4=[x_now_4];
    
pause(5);
for t = 0:dt:TOTAL_TIME
    tic
    cla
    
    % _______ IMPLEMENT CONTROLLER + SIMULATION PHYSICS HERE ______ %
    % Q1:
    [theta_now, x_now, xdot, omega] = drone.q2_update(x_now, xdot, theta_now, omega, dt, Aa, Ba, ref, Q);
    result_position=[result_position,x_now];
    drone.plot;
    t
    % _______ IMPLEMENT CONTROLLER + SIMULATION PHYSICS HERE ______ %
    drawnow nocallbacks limitrate
    pause(5*TIME_SCALE*dt-toc); 
end

for t = 0:dt:TOTAL_TIME
    tic
    cla
    
    % _______ IMPLEMENT CONTROLLER + SIMULATION PHYSICS HERE ______ %
    % Q1:
    [theta_now_1, x_now_1, xdot_1, thetadot_1] = drone1.update_error(theta_now_1, x_now_1, xdot_1, thetadot_1, dt, Q, small_error);
    
    result_position_1=[result_position_1,x_now_1];
    drone1.plot;
    t
    % _______ IMPLEMENT CONTROLLER + SIMULATION PHYSICS HERE ______ %
    drawnow nocallbacks limitrate
    pause(5*TIME_SCALE*dt-toc); 
end

for t = 0:dt:TOTAL_TIME
    tic
    cla
    
    % _______ IMPLEMENT CONTROLLER + SIMULATION PHYSICS HERE ______ %
    % Q1:
    [theta_now_2, x_now_2, xdot_2, omega_2] = drone2.q2_update_error(x_now_2, xdot_2, theta_now_2, omega_2, dt, Aa, Ba, ref, Q, small_error);

    result_position_2=[result_position_2,x_now_2];
    drone2.plot;
    t
    % _______ IMPLEMENT CONTROLLER + SIMULATION PHYSICS HERE ______ %
    drawnow nocallbacks limitrate
    pause(5*TIME_SCALE*dt-toc); 
end

for t = 0:dt:TOTAL_TIME
    tic
    cla
    
    % _______ IMPLEMENT CONTROLLER + SIMULATION PHYSICS HERE ______ %
    % Q1:
    [theta_now_3, x_now_3, xdot_3, thetadot_3] = drone3.update_error(theta_now_3, x_now_3, xdot_3, thetadot_3, dt, Q, large_error);
    result_position_3=[result_position_3,x_now_3];
    drone3.plot;
    t
    % _______ IMPLEMENT CONTROLLER + SIMULATION PHYSICS HERE ______ %
    drawnow nocallbacks limitrate
    pause(5*TIME_SCALE*dt-toc); 
end

for t = 0:dt:TOTAL_TIME
    tic
    cla
    
    % _______ IMPLEMENT CONTROLLER + SIMULATION PHYSICS HERE ______ %
    % Q1:
    [theta_now_4, x_now_4, xdot_4, omega_4] = drone4.q2_update_error(x_now_4, xdot_4, theta_now_4, omega_4, dt, Aa, Ba, ref, Q, large_error);
    
    result_position_4=[result_position_4,x_now_4];
    
    drone4.plot;
    t
    % _______ IMPLEMENT CONTROLLER + SIMULATION PHYSICS HERE ______ %
    drawnow nocallbacks limitrate
    pause(5*TIME_SCALE*dt-toc); 
end

figure;
scatter3(result_position(1,:), result_position(2,:), result_position(3,:), 'filled');  
xlabel('X-axix');
ylabel('Y-axix');
zlabel('Z-axix');
grid on;
title("Linear Tajectory of the Quadcopter with No Error")

figure;
scatter3(result_position_1(1,:), result_position_1(2,:), result_position_1(3,:), 'filled');  
xlabel('X-axix');
ylabel('Y-axix');
zlabel('Z-axix');
grid on;
title("Nonlinear Tajectory of the Quadcopter with Small Error")


figure;
scatter3(result_position_2(1,:), result_position_2(2,:), result_position_2(3,:), 'filled');  
xlabel('X-axix');
ylabel('Y-axix');
zlabel('Z-axix');
grid on;
title("Linear Tajectory of the Quadcopter with Small Error")

figure;
scatter3(result_position_3(1,:), result_position_3(2,:), result_position_3(3,:), 'filled');  
xlabel('X-axix');
ylabel('Y-axix');
zlabel('Z-axix');
grid on;
title("Nonlinear Tajectory of the Quadcopter with Large Error")

figure;
scatter3(result_position_4(1,:), result_position_4(2,:), result_position_4(3,:), 'filled');  
xlabel('X-axix');
ylabel('Y-axix');
zlabel('Z-axix');
grid on;
title("Linear Tajectory of the Quadcopter with Large Error")
