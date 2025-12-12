clear all
close all
clc

DNI = [2 9 5 5 5 5 5 8];

L1a_val = 0.9 - 0.04*DNI(2); 
L2a_val = 0.4 + 0.02*DNI(6); 
L3a_val = 0.4 + 0.01*DNI(3); 
L4a_val = 0.5 - 0.02*DNI(7);

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


q_inicial = [0 0 0];
qf = [pi/2 pi/8 pi/8];
Tmax = 5;% s




% Parametros de Controladores
s = tf ("s");

Kc = [10 10 10];
% Td = [1 1 1];
% Ti = [2 2 2];

G1 = tf([km(1)],[Jm(1) Bm(1) 0]);
G2 = tf([km(2)],[Jm(2) Bm(2) 0]);
G3 = tf([km(3)],[Jm(3) Bm(3) 0]);
% C1 = Kc(1)*(1 + Td(1)*s + 1/(Ti(1)*s));
% L1 = C1*G1;

wcp = 50; % En rad/s Frec deseada

[mag1, phase1] = bode(G1, wcp);
[mag2, phase2] = bode(G2, wcp);
[mag3, phase3] = bode(G3, wcp);

mag = [mag1; mag2; mag3];
phase = [phase1; phase2; phase3];

Mf_wcp = 180 + phase; % PMa
AMf = 80 - Mf_wcp + 10;

% Cálculo de T1 y T2 para cada modelo
T1 = 10 ./ wcp .* ones(3,1);
T2 = tand(AMf) ./ wcp;

% Ajuste de ganancia para lograr la magnitud deseada
K_db = -mag2db(mag) - mag2db(wcp .* T2);
K_natural = 10.^(K_db./20);

% Tiempos integral y derivativo
Ti = T1 + T2;
Td = (T1 .* T2) ./ (T1 + T2);
Kc = K_natural.*((T1+T2)./T1);

P = Kc;
I = 1./Ti;
D = Td;

unc = 0;


C1 = Kc(1)*(1 + Td(1)*s + 1/(Ti(1)*s));
L1 = C1*G1;

C2 = Kc(2)*(2 + Td(2)*s + 1/(Ti(2)*s));
L2 = C2*G2;

C3 = Kc(3)*(3 + Td(3)*s + 1/(Ti(3)*s));
L3 = C3*G3;

%% Calculate Appropriate Sample Time
fprintf('\n=== SAMPLE TIME CALCULATION ===\n');

% Method 1: Based on natural frequencies
wn = (km ./ Jm);
fprintf('Natural frequencies: [%.2f, %.2f, %.2f] rad/s\n', wn);
Ts_wn = (2*pi) / (10 * max(wn));

% Method 2: Based on time constants
tau = D;
fprintf('Time constants: [%.4f, %.4f, %.4f] s\n', tau);
Ts_tau = min(tau) / 20;

% Method 3: Based on settling time
ts = 4 * D;
fprintf('Settling times: [%.4f, %.4f, %.4f] s\n', ts);
Ts_ts = min(ts) / 100;

% Select the most conservative (smallest) sample time
Ts_calculated = min([Ts_wn, Ts_tau, Ts_ts]);
fprintf('\nRecommended sample times:\n');
fprintf('  - Based on bandwidth: %.5f s (%.1f Hz)\n', Ts_wn, 1/Ts_wn);
fprintf('  - Based on time constants: %.5f s (%.1f Hz)\n', Ts_tau, 1/Ts_tau);
fprintf('  - Based on settling time: %.5f s (%.1f Hz)\n', Ts_ts, 1/Ts_ts);
fprintf('\n>>> RECOMMENDED Ts = %.5f s (%.1f Hz) <<<\n', Ts_calculated, 1/Ts_calculated);

% Update Tm with calculated value (or keep 0.001 if it's appropriate)
Tm = Ts_calculated;  % Or use: Tm = 0.001 if you prefer
fprintf('Using Ts = %.5f s in simulation\n', Tm);


% Tm = 0.001;


%% Ejecución de la Simulación

out = sim("cl_PID.slx");

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
ylabel('u1 (V)')
title('Control Actions in u1')

subplot(3,1,2)
plot(out.u2.Time, out.u2.Data, 'LineWidth', 2.5);
grid on
xlabel('Time')
ylabel('u2 (V)')
title('Control Actions in u2')

subplot(3,1,3)
plot(out.u3.Time, out.u3.Data, 'LineWidth', 2.5);
grid on
xlabel('Time')
ylabel('u3 (V)')
title('Control Actions in u3')


%% Export figures as PNG

% Define folder to save images
folderName = 'results_PID';
if ~exist(folderName, 'dir')
    mkdir(folderName);
end

% Get all figure handles
figHandles = findall(0, 'Type', 'figure');

% Loop through figures and save each as PNG
for k = 1:length(figHandles)
    fig = figHandles(k);
    % Create filename
    fileName = fullfile(folderName, ['pid' num2str(k) '.png']);
    % Save as PNG
    saveas(fig, fileName);
end

disp(['Figures saved to folder: ' folderName]);
