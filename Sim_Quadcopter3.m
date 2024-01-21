% Differential drive robot
clc;
clear all;
close all;


% Simulation parameters
TOTAL_TIME  = 1000;
dt          = 0.1;
dynamics_dt = dt/10;

TIME_SCALE  = 0.1;


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
colours=[[.8 .3 .1];[.2 .2 .5];[.8 .1 .3];[.9 .6 .8];[.9 .2 .4]];
% colours=[[.7 .3 .1];[.3 .2 .5];[.6 .1 .3];[.7 .6 .8];[.4 .2 .4]]

drone1 = Quadcopter3(ax1,colours);


% Initial simulation state.
x = [0; 0; 0];
xdot = zeros(3, 1);
theta = zeros(3, 1);


ref=[0;0;5;zeros(9,1)];


% thetadot = zeros(3, 1);
thetadot=[0;0;0];
% drone1.set_dt(dynamics_dt)
drone1.dt=dt;
drone1.state=[x;xdot;zeros(6,1)];
drone1.equilibrium_state=drone1.state
% drone1.output=[x;theta]
% drone1.discretize();
% input = drone1.fsf_controller(ref)

% Run Simulation
count=0;

STATE=[];
INPUTS=[];

threshold=0.1;
drone1.discretize();
drone1.fsf_controller(zeros(12,1));
drone_state=0;

traj=drone1.getCircleTrajectory([0,0,5],2.5,360);
point=1;
checkpoints=[0 0 0;0 0 5;0 2.5 5; 0 0 7.5; 0 -2.5 5;0 0 2.5;2.5 2.5 2.5;2.5 2.5 0];

hold on;

for t = 0:dt/10:TOTAL_TIME
    tic
    cla

    % _______ IMPLEMENT CONTROLLER + SIMULATION PHYSICS HERE ______ %


    % Take input from our controller.
    

    if rem(t,dt)==0
        
        
        switch drone_state
            case 0
                drone1.output';
                ref=[0;0;5;0;0;0.5;zeros(6,1)];
                if pdist([drone1.output(1:3).';0 0 5])<threshold
                    drone_state=1;
                    start_time=t;
                end
            case 1
                ref=[0;0;5;zeros(9,1)];
                if t-start_time>5
                    drone_state=2;
                end
            case 2
                ref=[0;2.5;5;0;0;0;zeros(6,1)];
                if pdist([drone1.output(1:3).';0 2.5 5])<threshold
                        drone_state=3;
                end
            case 3
                 
                ref=traj(point,:).';
                if point<size(traj,1)
                    if pdist([drone1.output(1:3).';ref(1:3).'])<0.6
                        point=point+1;
                    end
                else
                    if pdist([drone1.output(1:3).';ref(1:3).'])<threshold
                        point=point+1;
                    end
                end
                if point>size(traj,1)
                    drone_state=4;
                end
            case 4
                ref=[2.5;2.5;2.5;zeros(9,1)];
                if pdist([drone1.output(1:3).';ref(1:3).'])<threshold
                        drone_state=5;
                end

            case 5
                ref=[2.5;2.5;drone1.state(3)-(0.11);0;0;-0.11;zeros(6,1)];
                disp('Vertical Velocity');
                disp(drone1.state(6));
                if pdist([drone1.output(1:3).';[2.5,2.5,0]])<0.1*dt
                        drone_state=6;
                        end_time=t;
                end
            case 6
                ref=[2.5;2.5;0;zeros(9,1)];
                if t>end_time+5
                    break;
                end
                
        end
        display('Desired next position:')
        display(ref(1:3).')
        drone1.fsf_controller(ref);
        display('Current position:')
        display(drone1.output(1:3)')


    end

    
    drone1.update(t);
    drone1.nonLinearModelDynamics(dt/10); 
    

    drone1.plot;
    plot3(drone1.STATE(1,:),drone1.STATE(2,:),drone1.STATE(3,:));
    plot3(checkpoints(:,1),checkpoints(:,2),checkpoints(:,3), 'o');

    
    title(strcat('Simulation Time:',num2str(t),'  State :',num2str(drone_state)))
    % title(strcat('Simulation Time:',num2str(t)));
    % subtitle(strcat('State :',num2str(drone_state)));
    % _______ IMPLEMENT CONTROLLER + SIMULATION PHYSICS HERE ______ %


    drawnow nocallbacks limitrate
    pause(TIME_SCALE*dt-toc); 
end
drone1.plot;
figure;
plot3(drone1.STATE(1,:),drone1.STATE(2,:),drone1.STATE(3,:));
axis('equal')
axis([-5 5 -5 5 0 10])
grid on;
hold on;
plot3(checkpoints(:,1),checkpoints(:,2),checkpoints(:,3), 'o')
hold off;
legend('drone trajectory','Checkpoints')
title('Trajectory of the drone')

figure;
plot(drone1.TIME,drone1.STATE(6,:));
title('Vertical velocity (Z dot) accross time(t)')
save('drone1','drone1');
xlabel('time(s)');
ylabel('Z dot (m/s^2)');

figure;
subplot(2,2,1);
plot(drone1.TIME,drone1.STATE(1,:),drone1.TIME,drone1.STATE(2,:),drone1.TIME,drone1.STATE(3,:));
hold on;
xlabel('time(s)');
ylabel('distance(m)');
title('Drone Position (X,Y,Z) over time');
legend('X','Y','Z')

hold off;

subplot(2,2,2);
plot(drone1.TIME,drone1.STATE(4,:),drone1.TIME,drone1.STATE(5,:),drone1.TIME,drone1.STATE(6,:));
hold on;
xlabel('time(s)');
ylabel('velocity(m/s)');
title('Drone Linear Velocities at time t');
legend('Xdot','Ydot','Zdot')

hold off;

subplot(2,2,3);
plot(drone1.TIME,drone1.STATE(7,:),drone1.TIME,drone1.STATE(8,:),drone1.TIME,drone1.STATE(9,:));
hold on;
xlabel('time(s)');
ylabel('Angle(radian)');
title('Drone Orientation at time t');
legend('Phi','Theta','Psi')

hold off;

subplot(2,2,4);
plot(drone1.TIME,drone1.STATE(10,:),drone1.TIME,drone1.STATE(11,:),drone1.TIME,drone1.STATE(12,:));
hold on;
xlabel('time(s)');
ylabel('Angle(radian)');
title('Drone Angular Velocities at time t');
legend('Omega X','Omega Y','Omega Z')

hold off;
