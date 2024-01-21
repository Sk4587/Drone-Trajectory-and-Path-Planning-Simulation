classdef Quadcopter3 < handle
    
    % Define robot fixed parameters
    properties (Access=private, Constant)
        
        %width, length, height offset between centre and rotors
        body = [0.6 0.6 0.0];

         

        g = 9.8;
        m = 0.3;
        L = 0.25;
        k = 1;
        kd = 0.2;
        I = diag([1 1 0.4]);
        b = 0.2;

    end
    
    % Define robot variable parameters (incl. state, output, input, etc)
    properties (Access=private)   
        % plotting
        ax (1,1) matlab.graphics.axis.Axes;
        colours;
        
        % Robot parameters needed for plotting
        pos (3,1) double; % 3D position: x-y-z 
        rot (3,1) double; % 3D orientation: yaw-pitch-roll 
        integral = zeros(3,1);
        integral2 = zeros(3,1);

        fsf_K; %control variable for fsf controller

        % E=[0.992   0.951    0.995    0.914  0.953  0.926 0.943    0.912    0.943    0.984    0.977   0.926];

        % E=[0.972   0.981    0.992    0.87  0.862  0.856 0.873   0.893
        % 0.91    0.899    0.8346   0.882]; Best till now

        E=[0.9657   0.9591    0.9725    0.85  0.822  0.856 0.943   0.959    0.971    0.849    0.8346   0.802];

        Ad;
        Bd;
        C;
        D;

    end    
    properties (Access=public)   
        dt;
        % state = zeros(12,1);
        state;
        output=zeros(12,1);
        % input = zeros(4,1);
        input=[0.735;0.735;0.735;0.735]

        equilibrium_inputs;
        equilibrium_state;

        STATE=[]
        TIME=[]
    end 

    methods
        % Class constructor
        function obj = Quadcopter3(ax,colours)
            obj.ax = ax;
            %colours of each component of drone model
            obj.colours = colours; 
            obj.equilibrium_inputs=obj.getEquilibrium();
        end        
        function set_dt(obj,dt)
            obj.dt = dt;

        end

        function equilibrium_inputs=getEquilibrium(obj)
            equilibrium_inputs=[obj.m*obj.g/4;obj.m*obj.g/4;obj.m*obj.g/4;obj.m*obj.g/4]./obj.k;
        end
        % you can modify this to model Quadcopter3 physics
        function update(obj,t)
            obj.output;
            obj.pos=[obj.state(1);obj.state(2);obj.state(3)];

            obj.rot=[obj.state(9);obj.state(8);obj.state(7)];
            obj.STATE=[obj.STATE obj.state];
            obj.TIME=[obj.TIME t];
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
        function omega = thetadot2omega(obj,thetadot, angles)

            phi = angles(1);
            theta = angles(2);
            psi = angles(3);
            
            W = [
                1,             0,             -sin(theta);
                0,             cos(phi),  cos(theta) * sin(phi);
                0,             -sin(phi),  cos(theta) * cos(phi)
                ];
            
            omega = W * thetadot;
        end
        function a = acceleration(obj,inputs, theta, xdot)

            gravity = [0; 0; -obj.g];
            R = obj.rotation(theta);
            T = R * obj.thrust(inputs);
            Fd = -obj.kd * xdot;
            a = gravity + 1 / obj.m * T + Fd/obj.m;
        end

        function T = thrust(obj,inputs)
            % Inputs are values for ${\omega_i}^2$
            T = [0; 0; obj.k * sum(inputs)];
        end
        

        function R = rotation(obj,angles)
            phi = angles(1);
            theta = angles(2);
            psi = angles(3);
            
            Rx = [1 0 0;
                  0 cos(phi) -sin(phi);
                  0 sin(phi) cos(phi)];
            Ry = [ cos(theta) 0 sin(theta);
                   0 1 0;
                   -sin(theta) 0 cos(theta)]; 
                     
            Rz = [cos(psi) -sin(psi) 0;
                  sin(psi) cos(psi) 0;
                  0 0 1]; 
            R = Rz*Ry*Rx;
            
        end

        function omegadot = angular_acceleration(obj,inputs, omega)
            tau = obj.torques(inputs);
            omegadot = obj.I \ (tau - cross(omega, obj.I * omega));
        end
        function tau = torques(obj,inputs)
            tau = [
                obj.L * obj.k * (inputs(1) - inputs(3))
                obj.L * obj.k * (inputs(2) - inputs(4))
                obj.b * (inputs(1) - inputs(2) + inputs(3) - inputs(4))
            ];
        end
        function thetadot = omega2thetadot(obj,omega, angles)
            phi = angles(1);
            theta = angles(2);
            psi = angles(3);
            
            W = [
                    1,             0,             -sin(theta);
                    0,             cos(phi),  cos(theta) * sin(phi);
                    0,             -sin(phi),  cos(theta) * cos(phi)
                    ];
                
             thetadot = W \ omega;
        end
        function input = pid_controller(obj,thetadot)
            Kd = 8;
            kp = 15;
            ki = 5.5;
            
            if max(abs(obj.integral2) > 0.01)
                obj.integral2(:) = 0;
            end
            
            i1 = obj.integral;
            i2 = obj.integral2;
            
            total = obj.m * obj.g / obj.k / (cos(i1(1)) * cos(i1(2)));
            
            e =Kd * thetadot + kp * i1 - ki * i2;
            
            input = obj.error2inputs(e, total);
            
        
            
             % Update the state
             obj.integral = obj.integral + obj.dt .* thetadot;
             obj.integral2 = obj.integral2 + obj.dt .* obj.integral;
        end
        function inputs = error2inputs(obj,e, total)
            Ix = obj.I(1,1);
            Iy = obj.I(2,2);
            Iz = obj.I(3,3);
            
            e1 = e(1);
            e2 = e(2);
            e3 = e(3);
            
            inputs = zeros(4,1);
            
            inputs(1) = total / 4 - (2 * obj.b * e1 * Ix + e3 * Iz * obj.k * obj.L) / (4 * obj.b * obj.k * obj.L);
            inputs(2) = total / 4 + e3 * Iz / (4 * obj.b) - (e2 * Iy) / (2 * obj.k * obj.L);
            inputs(3) = total / 4 - (-2 * obj.b * e1 * Ix + e3 * Iz * obj.k * obj.L) / (4 * obj.b * obj.k * obj.L);
            inputs(4) = total / 4 + e3 * Iz / (4 * obj.b) + e2 * Iy / (2 * obj.k * obj.L);
        end
        function discretize(obj)

            syms x [12,1];
            syms u [4,1];
            syms xdot [12,1];
            
            % syms gamma1 gamma2 gamma3 gamma4;
            % u=[gamma1 gamma2 gamma3 gamma4];
            
            %x1d...............................
            % x1d=x2

            xdot(1:3)=x(4:6);
            
            %x2d...............................


            R=obj.rotation(x(7:9));
            TB=obj.thrust(u);
            FD= -obj.kd*x(4:6);
            
            xdot(4:6)=[0;0;-obj.g]+((R*TB)/obj.m)+(FD/obj.m);
            
            %x3d...............................
            
            temp_mat=[
                        1,             0,             -sin(x(8));
                        0,             cos(x(7)),  cos(x(8)) * sin(x(7));
                        0,             -sin(x(7)),  cos(x(8)) * cos(x(7))
                    ];
            xdot(7:9)=temp_mat \ x(10:12);
            
            %x4d...............................
            tau = obj.torques(u);
            
            xdot(10:12)=obj.I \ (tau-cross((x(10:12)),obj.I*x(10:12)));
            
            %f................................
            
            % f=[x1d;x2d;x3d;x4d]
            xdot;
            gradfx=jacobian(xdot,x);
            gradfu=jacobian(xdot,u);
            
            x;
            A = double(subs(gradfx,[x1,x2,x3,x4,x5,x6,x7,x8,x9,x10,x11,x12,u1,u2,u3,u4], [obj.equilibrium_state.',obj.equilibrium_inputs']));
            B = double(subs(gradfu,[x1,x2,x3,x4,x5,x6,x7,x8,x9,x10,x11,x12], obj.equilibrium_state.'));

           
            
            obj.C = [1 1 1 1 1 1 1 1 1 1 1 1];
            % obj.C=eye(12)
            % obj.D = zeros(12,4);
            D=0;

            cont_ss = ss(A,B,obj.C,obj.D);
            disc_ss = c2d(cont_ss,obj.dt,'zoh');

            obj.Ad=disc_ss.A;
            obj.Bd=disc_ss.B;

            Co = ctrb(obj.Ad,obj.Bd);
            sizeCo=size(Co);
            if rank(Co) == sizeCo(1)
                display('System is Reachable')
            else
                display('System is not Reachable')
            end

            obj.fsf_K = place(obj.Ad,obj.Bd,obj.E);
            obj.fsf_K;
        end

        function fsf_controller(obj,rx)

            
            % ;
            % 
            % % obj.state=(Ad*obj.state)+(Bd*obj.input);
            % obj.state=((obj.Ad-obj.Bd*obj.fsf_K)*obj.state)+(obj.Bd*obj.fsf_K*rx);
            % obj.state(3)=max(0,obj.state(3));
            % obj.equilibrium_state=obj.state;
            % obj.equilibrium_state(4:12)=0;
            % 
            obj.input=-obj.fsf_K*(obj.state-rx);
            % 
            % 
            % 
            % obj.input =min(max(obj.input,-1.5),1.5);

            
            % obj.output=obj.C.'.*obj.state;
            % obj.output=obj.C*obj.state;
            % 
            % display('Current Drone Position')
            % display(obj.output(1:3)')

        end

        function nonLinearModelDynamics(obj,dt)
            input=obj.input+[0.735 0.735 0.735 0.735].' ;
            input =min(max(input,-1.5),1.5);
            omega = obj.state(10:12);
            % Compute linear and angular accelerations.
            a = obj.acceleration(input, obj.state(7:9), obj.state(4:6));
            omegadot = obj.angular_acceleration(input, omega);
        
            omega = omega + dt * omegadot;
            thetadot = obj.omega2thetadot(omega, obj.state(7:9)) ;
            obj.state(7:9) = obj.state(7:9) + dt * thetadot;
            obj.state(4:6) = obj.state(4:6) + dt * a;
            obj.state(1:3) = obj.state(1:3) + dt * obj.state(4:6);
            obj.state(10:12) = obj.thetadot2omega(thetadot, obj.state(7:9));
            obj.output=obj.C.'.*obj.state;
        end

        function traj=getCircleTrajectory(obj,center,radius,num_points)
            traj=[]
            theta = linspace(0, 2*pi, num_points);
            x = zeros(size(theta));
            y = center(2) + radius * cos(theta);
            z = center(3) + radius * sin(theta);

            traj=[x' y' z' repmat([0 0 0 0 0 0 deg2rad(0) deg2rad(0) 0] , [num_points,1])];

            % for theta = linspace(0, 2*pi, num_points)
            %     x=0;
            %     y=center(2) + radius * cos(theta);
            %     z = center(3) + radius * sin(theta);
            % 
            %     if theta<pi/2
            %         traj = [traj; [x y z zeros(1,3) repmat([0 deg2rad(5) 0] , [1,1]) zeros(1,3)]]
            %     else
            %         traj = [traj; [x y z repmat([0 0.1 0.1] , [1,1]) repmat([0 deg2rad(-5) 0] , [1,1]) zeros(1,3)]]
            %     end
            % end
        end



    end
end
