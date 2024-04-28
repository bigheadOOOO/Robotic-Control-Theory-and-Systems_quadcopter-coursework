% Differential drive robot
clc;
clear all;
close all;

% Simulation parameters
TOTAL_TIME  = 1000;
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


Q3a4_init;
ref(1:6) = targets(:,1);
target = ref(1:6)

Q4_observer_a_controller;
states_measure = [x_now;xdot;theta_now; omega];
states_measure_old = states_measure;
y_measure = Cd*[x_now;xdot;theta_now; omega];
result_position=[];
    

for t = 0:dt:TOTAL_TIME
    tic
    cla
    
    % _______ IMPLEMENT CONTROLLER + SIMULATION PHYSICS HERE ______ %
    if task_flag == 9
        break;
    end
    result_position=[result_position,x_now];
    x_state = [x_now; xdot; theta_now; omega]
    
    [task_flag,states_measure, counter, target, ref, points] = drone1.move_task_flag_q4(x_state, task_flag, ref, states_measure, states_measure_old, counter, targets, target, dt, points)
    % Kalman
    P = O*P0*O' + G_var
    L = (P*Cd')/(Cd*P*Cd'+R)
    P0 = (eye(12)-L*Cd)*P % Error covariance matrix
    % Update
    [theta_now, x_now, xdot, thetadot, states_measure, states_measure_old, y_measure] = drone1.q4_update(theta_now, x_now, xdot, thetadot, dt, t, ref, states_measure, y_measure, Ad, Bd, Cd, Kd, L)
    t
    drone1.plot;
    % _______ IMPLEMENT CONTROLLER + SIMULATION PHYSICS HERE ______ %
    drawnow nocallbacks limitrate
    pause(5*TIME_SCALE*dt-toc); 
end
plot_drone;
