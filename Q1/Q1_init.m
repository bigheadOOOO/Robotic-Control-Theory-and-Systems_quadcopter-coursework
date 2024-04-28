% ???? 需要初始化带error吗
% no error
xdot = zeros(3,1);
xDdot_now = zeros(3,1);
theta_now = zeros(3,1);
thetadot = zeros(3,1);
omega = zeros(3,1);


x_now = [0;0;5];
ref(1:3) = x_now;