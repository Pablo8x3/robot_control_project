% Robot Object 

D = [2 9 5 5 5 5 5 8];

L1a_val = 0.9 - 0.04*D(2); 
L2a_val = 0.4 + 0.02*D(6); 
L3a_val = 0.4 + 0.01*D(3); 
L4a_val = 0.5 - 0.02*D(7);


% 
% L(1) = Link('d', L1a_val, 'a', 0,                           'alpha', pi/2, 'offset', 0);
% L(2) = Link('d', 0,       'a', - sqrt(L2a_val^2 + L3a_val^2), 'alpha', 0, 'offset', atan2(L2a_val,L3a_val) + pi);
% L(3) = Link('d', 0,       'a', L4a_val,             'alpha', 0, 'offset', -atan2(L2a_val,L3a_val) + pi);

% Kinematic configuration:
L(1) = Link('offset', 0,                                   'd', L1a_val,        'a', 0,                                 'alpha', pi/2);
L(2) = Link('offset', atan2(L2a_val,L3a_val) + pi,         'd', 0,              'a', - sqrt(L2a_val^2 + L3a_val^2),     'alpha', 0);
L(3) = Link('offset', -atan2(L2a_val,L3a_val) + pi,        'd', 0,              'a', L4a_val,                           'alpha', 0);


robot_3dof = SerialLink(L, 'name', 'Robot 3-DOF');


disp(robot_3dof);


% Trajectory Planning 

q_inicial = [0 0 0];

T_inicial = robot_3dof.fkine(q_inicial);

centro = [0 0 L1a_val + L2a_val];

num_puntos = 100;                   % Número de puntos en el círculo
radio = L3a_val + L4a_val;          % Radio del círculo (en metros)
t = linspace(0, 2*pi, num_puntos);  % Vector de ángulos

x_local = radio * cos(t);
y_local = radio * sin(t);

path_X = centro(1) + radio * cos(t); % (1x100)
path_Y = centro(2) + radio * sin(t); % (1x100)
path_Z = ones(1, num_puntos) * centro(3); % (1x100)

puntos_camino = [path_X; 
                 path_Y; 
                 path_Z];

%mstraj 

T_path = transl(puntos_camino');


% Simulacion y Cinematica Inversa 

mascara = [1 1 1 0 0 0];

q_path = robot_3dof.ikine(T_path, 'q0', q_inicial, 'mask', mascara);

figure;
robot_3dof.plot(q_path, 'trail', {'r', 'LineWidth', 2});

% Hacer continua la trayectoria de q
q_continuo = unwrap(q_path, [], 1);  % unwrap a lo largo de las filas

figure;
plot(t, q_continuo(:,1), 'r', 'LineWidth', 1.5); hold on;
plot(t, q_continuo(:,2), 'g', 'LineWidth', 1.5);
plot(t, q_continuo(:,3), 'b', 'LineWidth', 1.5);
grid on;
xlabel('Time [s]');
ylabel('Angle of the joints [rad]');
title('Continuous position of the joints q1, q2, q3');
legend('q1', 'q2', 'q3');
