% Differential drive robot
clc;
clear all;
close all;

% Simulation parameters
TOTAL_TIME  = 3000;
dt          = 0.1;
TIME_SCALE  = 0.0001; % slows down simulation if > 1, speeds up if < 1 (and if computation allows...)


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


task3_flag = 0;
Q3a4_init;
ref(1:6) = targets(:,1);
target = ref(1:6)
Q3_full_state_fb_controller;
result_position=[];
    

for t = 0:dt:TOTAL_TIME
    tic
    cla
    
    % _______ IMPLEMENT CONTROLLER + SIMULATION PHYSICS HERE ______ %
    if task3_flag == 9
        break
    end
    result_position=[result_position,x_now];
    x_state = [x_now; xdot; theta_now; omega];
    [task3_flag, counter, target, ref, points] = drone1.move_task_flag_q3(x_state, task3_flag, ref, counter, targets, target, dt, points); 
    
    % update states
    [theta_now, x_now, xdot, thetadot, omega] = drone1.update_q3(t, theta_now, x_now, xdot, thetadot, dt/10, ref, Kd);
    t
    
    drone1.plot;
    % _______ IMPLEMENT CONTROLLER + SIMULATION PHYSICS HERE ______ %
    drawnow nocallbacks limitrate
    pause(5*TIME_SCALE*dt-toc); 
end
plot_drone;