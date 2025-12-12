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
Tmax = 10; % s

% Parametros de Controladores


Kp = 20;
Kd = 2;
unc = 0.2;
%% Calculate Appropriate Sample Time
fprintf('\n=== SAMPLE TIME CALCULATION ===\n');

% Method 1: Based on natural frequencies
wn = (km ./ Jm);
fprintf('Natural frequencies: [%.2f, %.2f, %.2f] rad/s\n', wn);
Ts_wn = (2*pi) / (10 * max(wn));




% Select the most conservative (smallest) sample time
Ts_calculated = min([Ts_wn]);
fprintf('\nRecommended sample times:\n');
fprintf('  - Based on bandwidth: %.5f s (%.1f Hz)\n', Ts_wn, 1/Ts_wn);

fprintf('\n>>> RECOMMENDED Ts = %.5f s (%.1f Hz) <<<\n', Ts_calculated, 1/Ts_calculated);

% Update Tm with calculated value (or keep 0.001 if it's appropriate)
Tm = Ts_calculated;  % Or use: Tm = 0.001 if you prefer
fprintf('Using Ts = %.5f s in simulation\n', Tm);


% Tm = 0.001;


%% Ejecución de la Simulación

fforward = sim("cl_ctorque.slx");

%% Resultados

figure

subplot(3,1,1)
plot(fforward.q1.Time,     fforward.q1.Data, 'LineWidth', 2.5); 
hold on
plot(fforward.q1_ref.Time, fforward.q1_ref.Data,'--', 'LineWidth', 2.5); 
hold off
grid on
xlabel('Time (s)')
ylabel('Ángulo (rads)')
legend('q1','q1\_ref')
title('Comparison q1 vs Reference')

subplot(3,1,2)
plot(fforward.q2.Time,     fforward.q2.Data, 'LineWidth', 2.5);  
hold on
plot(fforward.q2_ref.Time, fforward.q2_ref.Data,'--', 'LineWidth', 2.5); 
hold off
grid on
xlabel('Time (s)')
ylabel('Ángulo (rads)')
legend('q2','q2\_ref')
title('Comparison q2 vs Reference')

subplot(3,1,3)
plot(fforward.q3.Time,     fforward.q3.Data, 'LineWidth', 2.5);  
hold on
plot(fforward.q3_ref.Time, fforward.q3_ref.Data,'--', 'LineWidth', 2.5); 
hold off
grid on
xlabel('Time (s)')
ylabel('Ángulo (rads)')
legend('q3','q3\_ref')
title('Comparison q3 vs Reference')

% ---------- DIferencias--------------------

figure 

subplot(3,1,1)
mod_diff_q1 = abs(fforward.q1.Data - fforward.q1_ref.Data);
plot(fforward.q1.Time, mod_diff_q1, 'LineWidth', 2.5, 'Color', 'm');
grid on
xlabel('Time (s)')
ylabel('|q1 - q1_{ref}| (rads)')
title('q1 Absolute Error')

subplot(3,1,2)
mod_diff_q2 = abs(fforward.q2.Data - fforward.q2_ref.Data);
plot(fforward.q2.Time, mod_diff_q2, 'LineWidth', 2.5, 'Color', 'm');
grid on
xlabel('Time (s)')
ylabel('|q2 - q2_{ref}| (rads)')
title('q2 Absolute Error')

subplot(3,1,3)
mod_diff_q3 = abs(fforward.q3.Data - fforward.q3_ref.Data);
plot(fforward.q3.Time, mod_diff_q3, 'LineWidth', 2.5, 'Color', 'm');
grid on
xlabel('Time (s)')
ylabel('|q3 - q3_{ref}| (rads)')
title('q3 Absolute Error')


% ---------- Acciones de Control--------------------

figure 

subplot(3,1,1)
plot(fforward.u1.Time, fforward.u1.Data, 'LineWidth', 2.5);
grid on
xlabel('Time (s)')
ylabel('u1 (V)')
title('u1 Control Actions')

subplot(3,1,2)
plot(fforward.u2.Time, fforward.u2.Data, 'LineWidth', 2.5);
grid on
xlabel('Time (s)')
ylabel('u2 (V)')
title('u2 Control Actions')

subplot(3,1,3)
plot(fforward.u3.Time, fforward.u3.Data, 'LineWidth', 2.5);
grid on
xlabel('Time (s)')
ylabel('u3 (V)')
title('u3 Control Actions')


%% Export figures as PNG

% Define folder to save images
folderName = 'results_ct';
if ~exist(folderName, 'dir')
    mkdir(folderName);
end

% Get all figure handles
figHandles = findall(0, 'Type', 'figure');

% Loop through figures and save each as PNG
for k = 1:length(figHandles)
    fig = figHandles(k);
    % Create filename
    fileName = fullfile(folderName, ['ct' num2str(k) '_unc.png']);
    % Save as PNG
    saveas(fig, fileName);
end

disp(['Figures saved to folder: ' folderName]);