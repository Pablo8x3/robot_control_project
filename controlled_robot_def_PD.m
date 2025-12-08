% 1. Robot Object 
% 2. Apply formulas in order to get controller parameters 
% 3. Check in simulink 

% Robot Object 

D = [2 9 5 5 5 5 5 8];

perturbacion = 1; % 0 es no, 1 es si

L1a_val = 0.9 - 0.04*D(2); 
L2a_val = 0.4 + 0.02*D(6); 
L3a_val = 0.4 + 0.01*D(3); 
L4a_val = 0.5 - 0.02*D(7);

m1_val = 4;
m2_val = 2.7;
m3_val = 2;


L(1) = Link('d', L1a_val, 'a', 0,                           'alpha', pi/2, 'offset', 0);
L(2) = Link('d', 0,       'a', - sqrt(L2a_val^2 + L3a_val^2), 'alpha', 0, 'offset', atan2(L2a_val,L3a_val) + pi);
L(3) = Link('d', 0,       'a', L4a_val,             'alpha', 0, 'offset', -atan2(L2a_val,L3a_val) + pi);



% Masa de los brazos 
L(1).m = m1_val;
L(2).m = m2_val;
L(3).m = m3_val;

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


robot_3dof = SerialLink(L, 'name', 'Robot 3-DOF');

Gear = [25 20 25];
km = [10 10 10];
Bm = [0.0014 0.0019 0.0022];
Jm = [L(1).Jm L(2).Jm L(3).Jm];
N = [100 100 100];

Tmax = 5;
Tm = 0.001;
q_inicial = [0 0 0];
qf = [pi/2 pi/4 pi/4];

unc = 0.2;

D = [Jm(1)/Bm(1) Jm(2)/Bm(2) Jm(3)/Bm(3)];
P = [1000*Bm(1)/km(1) 1000*Bm(2)/km(2) 1000*Bm(3)/km(3)];

%% Simular 

open("cl_PD_bueno.slx")
out = sim("cl_PD_bueno.slx");

%% Resultados

figure

subplot(3,1,1)
plot(out.q1.Time,     out.q1.Data, 'LineWidth', 2.5); 
hold on
plot(out.q1_ref.Time, out.q1_ref.Data,'--', 'LineWidth', 2.5); 
hold off
grid on
xlabel('Time')
ylabel('Angle (rads)')
legend('q1','q1\_ref')
title('Comparison q1 vs reference')

subplot(3,1,2)
plot(out.q2.Time,     out.q2.Data, 'LineWidth', 2.5);  
hold on
plot(out.q2_ref.Time, out.q2_ref.Data,'--', 'LineWidth', 2.5); 
hold off
grid on
xlabel('Time')
ylabel('Angle (rads)')
legend('q2','q2\_ref')
title('Comparison q2 vs reference')

subplot(3,1,3)
plot(out.q3.Time,     out.q3.Data, 'LineWidth', 2.5);  
hold on
plot(out.q3_ref.Time, out.q3_ref.Data,'--', 'LineWidth', 2.5); 
hold off
grid on
xlabel('Time')
ylabel('Angle (rads)')
legend('q3','q3\_ref')
title('COmparison q3 vs reference')

% ---------- DIferencias--------------------

figure 

subplot(3,1,1)
subplot(3,1,1)
mod_diff_q1 = abs(out.q1.Data - out.q1_ref.Data);
plot(out.q1.Time, mod_diff_q1, 'LineWidth', 2.5, 'Color', 'm');
grid on
xlabel('Time')
ylabel('|q1 - q1_{ref}| (rads)')
title('Absolute error of q1')

subplot(3,1,2)
mod_diff_q2 = abs(out.q2.Data - out.q2_ref.Data);
plot(out.q2.Time, mod_diff_q2, 'LineWidth', 2.5, 'Color', 'm');
grid on
xlabel('Time')
ylabel('|q2 - q2_{ref}| (rads)')
title('Absolute error of q2')

subplot(3,1,3)
mod_diff_q3 = abs(out.q3.Data - out.q3_ref.Data);
plot(out.q3.Time, mod_diff_q3, 'LineWidth', 2.5, 'Color', 'm');
grid on
xlabel('Time')
ylabel('|q3 - q3_{ref}| (rads)')
title('Absolute error of q3')

% ---------- Acciones de Control--------------------

figure 

subplot(3,1,1)
plot(out.u1.Time, out.u1.Data, 'LineWidth', 2.5);
grid on
xlabel('Time')
ylabel('u1 (Nm)')
title('Control Actions in u1')

subplot(3,1,2)
plot(out.u2.Time, out.u2.Data, 'LineWidth', 2.5);
grid on
xlabel('Time')
ylabel('u2 (Nm)')
title('Control Actions in u2')

subplot(3,1,3)
plot(out.u3.Time, out.u3.Data, 'LineWidth', 2.5);
grid on
xlabel('Time')
ylabel('u3 (Nm)')
title('Control Actions in u3')
