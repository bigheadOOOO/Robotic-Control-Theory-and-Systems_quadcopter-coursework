classdef Quadcopter < handle
    
    % Define robot fixed parameters
    properties (Access=public, Constant)
        
        %width, length, height offset between centre and rotors
        body = [0.6 0.6 0.0];
        %colours of each component of drone model
        colours = [[.8 .3 .1];[.2 .2 .5];[.8 .1 .3];[.9 .6 .8];[.9 .2 .4]]; 

        m = 0.3;
        I = [1, 0, 0; 0, 1, 0; 0, 0, 0.4];
        g = 9.8;
        kd = 0.2;
        k = 1;
        b = 0.2;
        L =  0.25;
        ref_phi = 0;
        ref_th = 0;
        ref_psi = 0;
        v_wconstant = 0.01;
        v_wgradient = 0.001;
        t0_wgradient = 100;
        t1_wgradient = 500;
        t2_wgradient = 900;
        v_wrandom = 0.01;
        rho = 1.225;
        cd = 0.02;
        A = 0.1;
        
    end
    
    
    % Define robot variable parameters (incl. state, output, input, etc)
    properties (Access=public)   
        % plotting
        ax (1,1) matlab.graphics.axis.Axes;
        
        % Robot parameters needed for plotting
        pos (3,1) double; % 3D position: x-y-z 
        rot (3,1) double; % 3D orientation: yaw-pitch-roll 

    end

    methods
        % Class constructor
        function obj = Quadcopter(ax)
            obj.ax = ax;
            
            
        end        
        
        function [theta, p, xdot, thetadot] = update(obj, theta, x_now, xdot, thetadot, dt, Q)
            
            if Q == 1.1 % equilibrium
                gamma = obj.m*obj.g/(4*obj.k*cos(theta(1)*cos(theta(2))));
                gammas = [gamma;gamma;gamma;gamma];
            elseif Q == 1.2 % free fall
                gammas = [0;0;0;0];
            elseif Q == 1.3 % constant altitude
                gamma = (obj.m*obj.g)/(4*obj.k);  
                gammas = [gamma+0.5;gamma-0.5;gamma+0.5;gamma-0.5]; % force -> dot_omega
            end
            omega = obj.thetadot2omega(thetadot, theta);
            omegadot = obj.angle_accelaration(omega, gammas);
            omega = omega + omegadot*dt; 
            a = obj.line_accelaration(theta, xdot, obj.thrust(gammas));
            thetadot = (obj.rotation(theta))\omega;

            theta = wrapToPi(theta+thetadot*dt);
            xdot = xdot + dt*a;
            
            dx_1 = xdot;
            dx_2 = a;
            dx_3 = thetadot;
            dx_4 = omegadot;
            p = x_now + xdot*dt;
            phi = theta(1);
            th = theta(2);
            psi = theta(3);
            obj.pos = p; %??????????????Rz*Ry*Rx
            obj.rot = [psi;phi;th];
            % update input
        end
        
        function [theta_new, x_new, xdot_new, omega_new] = q2_update(obj, x_now, xdot_now, theta_now, omega_now, dt, A, B, ref, Q)
            % theta_now and position are observable
            syms omega_x omega_y omega_z x y z x_dot y_dot z_dot;
            phi = sym('phi');
            th = sym('th');
            psi = sym('psi');
            gamma1 = sym('gamma1');
            gamma2 = sym('gamma2');
            gamma3 = sym('gamma3');
            gamma4 = sym('gamma4');
            if Q == 1 % equilibrium
                gamma = (obj.m*obj.g)/(4*obj.k);
                gammas = [gamma;gamma;gamma;gamma];
            elseif Q==2 % free fall
                gammas = [0;0;0;0];
            else % constant altitude
                gamma = (obj.m*obj.g)/(4*obj.k);
                gammas = [gamma+0.5;gamma-0.5;gamma+0.5;gamma-0.5]; % force -> dot_omega 
            end
            c = obj.m*obj.g/(4*obj.k);
            u = gammas-[c;c;c;c];
            deltax_dot = A*([x_now; xdot_now; theta_now; omega_now]-ref)+B*u;
            deltax_dot = double(subs(deltax_dot, ...
                [x,y,z,x_dot,y_dot,z_dot,psi,th,phi,omega_x,omega_y,omega_z], ...
                [x_now(1),x_now(2),x_now(3), xdot_now(1), xdot_now(2), xdot_now(3), theta_now(1), theta_now(2), theta_now(3),omega_now(1),omega_now(2),omega_now(3)]));
            x_1 = x_now+deltax_dot(1:3)*dt;
            x_2 = xdot_now+deltax_dot(4:6)*dt;
            x_3 = theta_now+deltax_dot(7:9)*dt;
            x_4 = omega_now+deltax_dot(10:12)*dt;
            x_new = x_1;
            xdot_new = x_2;
            theta_new = x_3;
            omega_new = x_4;
            obj.plot_update(x_new, theta_new);
        end

        function [theta, p, xdot, thetadot] = update_error(obj, theta, x_now, xdot, thetadot, dt, Q, error)
            
            if Q == 1 % equilibrium
                gamma = obj.m*obj.g/(4*obj.k*cos(theta(1)*cos(theta(2))));
                gammas = [gamma+error;gamma;gamma;gamma];
            elseif Q == 2 % free fall
                gammas = [error;0;0;0];
            elseif Q == 3 % constant altitude
                gamma = (obj.m*obj.g)/(4*obj.k);  
                gammas = [gamma+0.5+error;gamma-0.5;gamma+0.5;gamma-0.5]; % force -> dot_omega
            end
            omega = obj.thetadot2omega(thetadot, theta);
            omegadot = obj.angle_accelaration(omega, gammas);
            omega = omega + omegadot*dt; 
            a = obj.line_accelaration(theta, xdot, obj.thrust(gammas));
            thetadot = (obj.rotation(theta))\omega;

            theta = wrapToPi(theta+thetadot*dt);
            xdot = xdot + dt*a;
            
            dx_1 = xdot;
            dx_2 = a;
            dx_3 = thetadot;
            dx_4 = omegadot;
            p = x_now + xdot*dt;
            phi = theta(1);
            th = theta(2);
            psi = theta(3);
            obj.pos = p; %??????????????Rz*Ry*Rx
            obj.rot = [psi;phi;th];
            obj.plot_update(p, theta);
            % update input
        end
        
        function [theta_new, x_new, xdot_new, omega_new] = q2_update_error(obj, x_now, xdot_now, theta_now, omega_now, dt, A, B, ref, Q, error)
            % theta_now and position are observable
            syms omega_x omega_y omega_z x y z x_dot y_dot z_dot;
            phi = sym('phi');
            th = sym('th');
            psi = sym('psi');
            gamma1 = sym('gamma1');
            gamma2 = sym('gamma2');
            gamma3 = sym('gamma3');
            gamma4 = sym('gamma4');
            if Q == 1 % equilibrium
                gamma = (obj.m*obj.g)/(4*obj.k);
                gammas = [gamma+error;gamma;gamma;gamma];
            elseif Q==2 % free fall
                gammas = [error;0;0;0];
            else % constant altitude
                gamma = (obj.m*obj.g)/(4*obj.k);
                gammas = [gamma+0.5+error;gamma-0.5;gamma+0.5;gamma-0.5]; % force -> dot_omega 
            end
            c = obj.m*obj.g/(4*obj.k);
            u = gammas-[c;c;c;c];
            deltax_dot = A*([x_now; xdot_now; theta_now; omega_now]-ref)+B*u;
            deltax_dot = double(subs(deltax_dot, ...
                [x,y,z,x_dot,y_dot,z_dot,psi,th,phi,omega_x,omega_y,omega_z], ...
                [x_now(1),x_now(2),x_now(3), xdot_now(1), xdot_now(2), xdot_now(3), theta_now(1), theta_now(2), theta_now(3),omega_now(1),omega_now(2),omega_now(3)]));
            x_1 = x_now+deltax_dot(1:3)*dt;
            x_2 = xdot_now+deltax_dot(4:6)*dt;
            x_3 = theta_now+deltax_dot(7:9)*dt;
            x_4 = omega_now+deltax_dot(10:12)*dt;
            x_new = x_1;
            xdot_new = x_2;
            theta_new = x_3;
            omega_new = x_4;
            obj.plot_update(x_new, theta_new);
        end

        
        function [flag,states_measure, counter, target, ref, points] = move_task_flag_q4(obj, x_state, flag, ref, states_measure, states_measure_old, counter, targets, target, dt, points)
            if flag == 0
                if obj.achieve(x_state, ref, 0.2, 0.3) %all(abs(x_state - ref) < 0.2)
                    flag = 1;
                end
            elseif flag == 1 %all(abs(x_state - ref) < 0.2))
                counter = counter + dt;
                if abs(counter - 5) < 1e-2
                    flag = 2;
                end
            elseif flag == 2 % go [0;2.5;5]
                ref(1:6) = targets(:,2);
                target = targets(:,2)
                if obj.achieve_m(states_measure, states_measure_old, ref, 1e-1, 0.02)
                    flag = 3;
                    target(1:3) = points(1,:)
                    points(1,:) = []
                end
            elseif flag == 3 % [0;2.5;5] -> [0;0;7.5]
                f = targets(:,flag)
                judge = obj.achieve_m(states_measure, states_measure_old, ref, 1e-1, 0.08);
                if judge
                    if size(points,1) > 1
                        points(1,:) = []
                        target(1:3) = points(1,:)
                        ref(1:3) = target(1:3)
                    elseif obj.achieve_m(states_measure, states_measure_old, ref, 1e-1, 0.07)
                        flag = 7;
                    end
                end
                
            elseif and(flag >= 7, flag < 9)
                target = targets(:,flag);
                ref(1:6) = target;
                if obj.achieve_m(states_measure, states_measure_old, ref, 0.05, 0.15)
                    flag = flag + 1;
                end
            
            end
        end

        function [flag, counter, target, ref, points] = move_task_flag_q3(obj, x_state, flag, ref, counter, targets, target, dt, points)
            if flag == 0
                if obj.achieve(x_state, ref, 0.1, 0.3)
                    flag = 1;
                end
            elseif flag == 1
                counter = counter + dt;
                if abs(counter - 5) < 1e-1
                    flag = 2;
                end
            elseif flag == 2 % go [0;2.5;5]
                ref(1:6) = targets(:,2);
                target = targets(:,2)
                if obj.achieve(x_state, ref, 0.05, 0.02)
                    flag = 3;
                    target(1:3) = points(1,:)
                    points(1,:) = []
                end
            elseif flag == 3 % [0;2.5;5] -> [0;0;7.5]
                f = targets(:,flag)
                judge = obj.achieve(x_state,ref, 0.1, 0.08);
                if judge
                    if size(points,1) > 1
                        points(1,:) = []
                        target(1:3) = points(1,:)
                        ref(1:3) = target(1:3);
                    elseif obj.achieve(x_state,ref, 0.1, 0.04)
                        flag = 7;
                        target = targets(:,flag);
                        ref(1:6) = target;
                    end
                end
            elseif and(flag >= 7, flag < 9)
                target = targets(:,flag);
                ref(1:6) = target;
                if obj.achieve(x_state,ref, 1e-1, 0.15)
                    flag = flag + 1;
                end
            end
        end

        function [y,z] = move_round_task(obj, ref, x_now, radius)
            % if ref(3) > 
            %     z = ref(3) - 0.001;
            %     y = radius^2 - y^2;
            % end
        end

        function judge = achieve(obj, x_state, ref, threshold_1, threshold_2)
            judge = and((pdist([x_state(1:3)'; ref(1:3)']) <= threshold_1), all(abs(x_state-ref) < threshold_2));
        end
        
        function judge_measure = achieve_m(obj, x_state, x_state_measure, ref, threshold_1, threshold_2)
            judge_measure = (and(pdist([x_state(1:3)'; ref(1:3)']) < threshold_1, all(x_state-ref < threshold_2)))
        end

        function plot_update(obj, x_new, theta_new)
            phi = theta_new(1); % φ
            th = theta_new(2);
            psi = theta_new(3); % ψ
            obj.pos = x_new; 
            obj.rot = [psi;phi;th];
        end

        function a = line_accelaration(obj, theta_now, xdot, thrust)
            gravity = [0;0;-obj.g];
            phi = theta_now(1);
            th = theta_now(2);
            psi = theta_now(3);
            Rx = [1, 0, 0; 0, cos(phi), -sin(phi); 0, sin(phi), cos(phi)];
            Ry = [cos(th), 0, sin(th); 0, 1, 0; -sin(th), 0, cos(th)];
            Rz = [cos(psi), -sin(psi), 0; sin(psi), cos(psi), 0; 0, 0, 1];
            
            Fd = -obj.kd * xdot;
            a = gravity + 1/obj.m*Rz*Ry*Rx*thrust+1/obj.m*Fd;
        end
        function T = thrust(obj, gammas)
            T = [0;0;obj.k*sum(gammas)];
        end
        function torque = cal_torque(obj, gammas)
            gamma1 = gammas(1);
            gamma2 = gammas(2);
            gamma3 = gammas(3);
            gamma4 = gammas(4);
            Lk = obj.L*obj.k;
            torque_phi = Lk*(gamma1-gamma3);
            torque_theta = Lk*(gamma2-gamma4);
            torque_psi = obj.b*(gamma1-gamma2+gamma3-gamma4);
            torque = [torque_phi;torque_theta;torque_psi]; % torque
            
        end
        function omegadot = angle_accelaration(obj, omega, gammas)
            torque = obj.cal_torque(gammas);
            omegadot = (obj.I)\(torque-cross(omega, obj.I*omega));
        end
        function omega = thetadot2omega(obj, thetadot, theta)
            omega = obj.rotation(theta)*thetadot;
        end
        function R0 = rotation(obj, theta)
            phi = theta(1); % φ
            th = theta(2);
            psi = theta(3); % ψ
            R0 = [1,0,-sin(th); 0, cos(phi), cos(th)*sin(phi);0,-sin(phi), cos(th)*cos(phi)];
        end
        function plot(obj)
            %create middle sphere
            [X Y Z] = sphere(8);
            X = (obj.body(1)/5.).*X + obj.pos(1);
            Y = (obj.body(1)/5.).*Y + obj.pos(2);
            Z = (obj.body(1)/5.).*Z + obj.pos(3);
            s = surf(obj.ax,X,Y,Z);
            set(s,'edgecolor','none','facecolor',obj.colours(1,:));
            
            %create side spheres
            %front, right, back, left
            hOff = obj.body(3)/2;
            Lx = obj.body(1)/2;
            Ly = obj.body(2)/2;
            rotorsPosBody = [...
                0    Ly    0    -Ly;
                Lx    0    -Lx   0;
                hOff hOff hOff hOff];
            rotorsPosInertial = zeros(3,4);
            rot_mat           = eul2rotm(obj.rot.');
            for i = 1:4
                rotorPosBody = rotorsPosBody(:,i);
                rotorsPosInertial(:,i) = rot_mat*rotorPosBody;
                [X Y Z] = sphere(8);
                X = (obj.body(1)/8.).*X + obj.pos(1) + rotorsPosInertial(1,i);
                Y = (obj.body(1)/8.).*Y + obj.pos(2) + rotorsPosInertial(2,i);
                Z = (obj.body(1)/8.).*Z + obj.pos(3) + rotorsPosInertial(3,i);
                s = surf(obj.ax,X,Y,Z);
                set(s,'edgecolor','none','facecolor',obj.colours(i+1,:));
            end      
            
            
        end
        function [theta, p, xdot, thetadot, omega] = update_q3(obj, t, theta, x_now, xdot, thetadot, dt, ref, Kd)
            e = [x_now; xdot; theta; thetadot]-(ref); % error = (x-ref_x) - 0 = x-ref_x
            u_fsfc = -Kd*e;
            gamma = (obj.m*obj.g)/(4*obj.k);
            gammas = [gamma;gamma;gamma;gamma]+u_fsfc;
            gammas = max(min(gammas,1.5),-1.5);
            omega = obj.thetadot2omega(thetadot, theta);
            omegadot = obj.angle_accelaration(omega, gammas);
            omega = omega + omegadot*dt; 
            a = obj.line_accelaration(theta, xdot, obj.thrust(gammas));
            thetadot = inv(obj.rotation(theta))*omega;

            theta = (theta+thetadot*dt);
            xdot = xdot + dt*a;
            
            p = x_now + xdot*dt;
            if p(3) < 1e-8
                p(3) = 0; % ground
                xdot(3) = 0;
            end
            p_show = p'
            phi = theta(1);
            th = theta(2);
            psi = theta(3);
            obj.pos = p; 
            obj.rot = [psi;phi;th];
        end

        function [theta, p, xdot, thetadot, states_measure_new, states_measure_old, y_measure] = q4_update(obj, theta, x_now, xdot, thetadot, dt, t, ref, states_measure,y_measure, Ad, Bd, Cd, Kd, L)
            
            % use non-linear model
            omega = obj.thetadot2omega(thetadot, theta);
            e = states_measure-ref; % error = (x-ref_x) - 0 = x-ref_x
            
            watch = e(1:3)
            u_fsfc = -Kd*e;
            
            x_real = [x_now; xdot; theta; omega]
            y_real = Cd*x_real;
            
            % 
            v = 1e-2*randn(size(y_real)) % unit: m
            y_observed = y_real + v;
            states_measure_old = states_measure
            % add wind
            states_measure_new = Ad*states_measure+Bd*u_fsfc+L*(y_observed-y_measure)
            y_measure = Cd*states_measure_new
            gamma = (obj.m*obj.g)/(4*obj.k);
            gammas = [gamma;gamma;gamma;gamma]+u_fsfc;
            gammas = max(min(gammas,1.5),-1.5);
            omega = obj.thetadot2omega(thetadot, theta);
            a = obj.line_accelaration_wind(theta, xdot, obj.thrust(gammas), t, omega);
            omegadot = obj.angle_accelaration(omega, gammas);
            omega = omega + omegadot*dt; 
            thetadot = (obj.rotation(theta))\omega;

            theta = wrapToPi(theta+thetadot*dt);
            xdot = xdot + dt*a;
            
            dx_1 = xdot;
            dx_2 = a;
            dx_3 = thetadot;
            dx_4 = omegadot;
            p = x_now + xdot*dt
            if p(3) < 1e-8
                p(3) = 0; % ground
                xdot(3) = 0;
            end
            theta_m = states_measure(7:9);
            phi = theta(1);
            th = theta(2);
            psi = theta(3);
            obj.pos = p; %??????????????Rz*Ry*Rx
            obj.rot = [psi;phi;th];
            % update input
        end
        function a = line_accelaration_wind(obj, theta_now, xdot, thrust, t, omega)
            gravity = [0;0;-obj.g];
            phi = theta_now(1);
            th = theta_now(2);
            psi = theta_now(3);
            Rx = [1, 0, 0; 0, cos(phi), -sin(phi); 0, sin(phi), cos(phi)];
            Ry = [cos(th), 0, sin(th); 0, 1, 0; -sin(th), 0, cos(th)];
            Rz = [cos(psi), -sin(psi), 0; sin(psi), cos(psi), 0; 0, 0, 1];
            
            Fd = -obj.kd * xdot;
            v_wind = obj.cal_wind_velocity(t, omega);
            a_F_wind = 1/2*obj.rho*(xdot+v_wind)*obj.A*obj.cd;
            a = gravity + 1/obj.m*Rz*Ry*Rx*thrust+1/obj.m*Fd+a_F_wind
        end
        function wind_velocity = cal_wind_velocity(obj, t, omega)
            if or(t < obj.t0_wgradient, t > obj.t2_wgradient)
                wind_velocity = obj.v_wconstant+obj.v_wrandom*(rand(1)-0.5)*sin(2*pi*t+omega);
            elseif t < obj.t1_wgradient
                wind_velocity = obj.v_wconstant+obj.v_wrandom*(rand(1)-0.5)*sin(2*pi*t+omega)+obj.v_wgradient*(t-obj.t0_wgradient)/(obj.t1_wgradient-obj.t0_wgradient);
            elseif t <= obj.t2_wgradient
                wind_velocity = obj.v_wconstant+obj.v_wrandom*(rand(1)-0.5)*sin(2*pi*t+omega)+obj.v_wgradient*(t-obj.t1_wgradient)/(obj.t2_wgradient-obj.t1_wgradient);
            end
        end
    end
end