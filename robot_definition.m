
% clear all
% close all
% clc
set(0,'DefaultFigureWindowStyle','docked')

%% Link definition:
D = [2 9 5 5 5 5 5 8];

L1a_val = 0.9 - 0.04*D(2); 
L2a_val = 0.4 + 0.02*D(6); 
L3a_val = 0.4 + 0.01*D(3); 
L4a_val = 0.5 - 0.02*D(7);

% Kinematic configuration:
L(1) = Link('d', L1a_val, 'a', 0,                           'alpha', pi/2, 'offset', 0);
L(2) = Link('d', 0,       'a', - sqrt(L2a_val^2 + L3a_val^2), 'alpha', 0, 'offset', atan2(L2a_val,L3a_val) + pi);
L(3) = Link('d', 0,       'a', L4a_val,             'alpha', 0, 'offset', -atan2(L2a_val,L3a_val) + pi);

% Link masses:
L(1).m = 4;
L(2).m = 2.7;
L(3).m = 2;


% Position of the centre of gravity with respect to the local coordinate frame (CF): 
%            rx      ry                 rz
L(1).r = [   0       0                  0     	]; % Centre of gravity of link 1 with respect to CF 1
L(2).r = [   0       0                  0    	]; % Centre of gravity of link 2 with respect to CF 2
L(3).r = [   0       0                  0    	]; % Centre of gravity of link 3 with respect to CF 3

% Additional inertia parameters with respect to the local coordinate frame: 
L(1).I = zeros(1,6);
L(2).I = zeros(1,6);
L(3).I = zeros(1,6);

% Inertia of the joint motor: 
L(1).Jm = 0.15;
L(2).Jm = 0.25;
L(3).Jm = 0.08;

% Gear ratio of the reduction gearbox of every joint:
L(1).G = 25;  
L(2).G = 20;
L(3).G = 25;

% Viscous friction coefficient of the joint motor: 
L(1).B = 0.0014;  
L(2).B = 0.0019;
L(3).B = 0.0022;

%% Robot creation:

robot_3dof = SerialLink(L, 'name', 'Robot 3-DOF');
robot_3dof.qlim = [-pi pi;-pi pi;-pi pi];  % No deja que los motores del robot den mas de una vuelta

%% Random pose:

qr = [0 0 0];

% Representation: 

figure(1);
robot_3dof.plot(qr);%,'workspace',2*[-1 1 -1 1 -1 1]);%,'jvec');
grid on;
axis(2*[-1 1 -1 1 -1 1]);

%% Open-loop simulation:

TF = 6;     % Final simulation time
Tm = 0.1;   % Sample time for data storage and representation

% Sinusoidal inputs:

qi = qr;

offset = [0 20 0];
amplitud = [15 15 20];

sim('Open_loop_robot');

q = [q1 q2 q3];
qdot = [q1dot q2dot q3dot];
tau = [tau1 tau2 tau3];

%% Representation of the open-loop results:

figure(11);
subplot(2,1,1);
plot(t,q(:,1));
grid on;
xlabel('Time (s)');
ylabel('q_1 (rad)');
title('q_1 joint trajectory');

subplot(2,1,2);
plot(t,qdot(:,1));
grid on;
xlabel('Time (s)');
ylabel('$\dot{q}_1$ (rad/s)','Interpreter','latex');
title('q_1 joint velocity');

figure(12);
subplot(2,1,1);
plot(t,q(:,2));
grid on;
xlabel('Time (s)');
ylabel('q_2 (rad)');
title('q_2 joint trajectory');

subplot(2,1,2);
plot(t,qdot(:,2));
grid on;
xlabel('Time (s)');
ylabel('$\dot{q}_2$ (rad/s)','Interpreter','latex');
title('q_2 joint velocity');

figure(13);
subplot(2,1,1);
plot(t,q(:,3));
grid on;
xlabel('Time (s)');
ylabel('q_3 (rad)');
title('q_3 joint trajectory');

subplot(2,1,2);
plot(t,qdot(:,3));
grid on;
xlabel('Time (s)');
ylabel('$\dot{q}_3$ (rad/s)','Interpreter','latex');
title('q_3 joint velocity');

figure(21);
subplot(3,1,1);
plot(t,tau(:,1));
grid on;
xlabel('Time (s)');
ylabel('$\tau_1$ (Nm)','Interpreter','latex');
title('Torque Applied in q_1');

subplot(3,1,2);
plot(t,tau(:,2));
grid on;
xlabel('Time (s)');
ylabel('$\tau_2$ (Nm)','Interpreter','latex');
title('Torque Applied in q_2');

subplot(3,1,3);
plot(t,tau(:,3));
grid on;
xlabel('Time (s)');
ylabel('$\tau_3$ (Nm)','Interpreter','latex');
title('Torque Applied in q_3');


%% Cartesian Trajectories Representation 

HTMS = robot_3dof.fkine(q); % HTMS almacena todas las htms del mov del rob

tras = [HTMS.t];

figure(50);

subplot(3,1,1);
plot(t,tras(1,:));
grid on;
xlabel('Time (s)');
ylabel('X (m)','Interpreter','latex');
title("Position of robot's end effector in x");

subplot(3,1,2);
plot(t,tras(2,:));
grid on;
xlabel('Time (s)');
ylabel('Y (m)','Interpreter','latex');
title("Position of robot's end effector in Y");

subplot(3,1,3);
plot(t,tras(1,:));
grid on;
xlabel('Time (s)');
ylabel('Z (m)','Interpreter','latex');
title("Position of robot's end effector in Z");

%% Robot motion animation:

figure(100);
robot_3dof.plot(q,'workspace',2*[-1 1 -1 1 -1 1],'jvec','trail','r-');
grid on;
axis(2*[-1 1 -1 1 -1 1]);




