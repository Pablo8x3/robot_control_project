
% clear all
% close all
% clc

%% Definition of symbolic variables:

syms q1 q2 q3 real;
syms q1dot q2dot q3dot real;
syms q1ddot q2ddot q3ddot real;
pi = sym(pi);

syms l1a l2a l3a l4a real;
syms m1 m2 m3 g real;

% Joints si revolute o prismatic 

j1 = "r";
j2 = "r";
j3 = "r";

%% D-H matrices:  ESTO PONERLO TAL Y COMO LO QUE HABÍA EN EL DE AYER

%      theta                                    d               a                                   alpha
A01 = MDH(q1,                                   l1a,              0,                           pi/2);
A12 = MDH(q2 + atan2(l2a,l3a) + pi ,              0,      -sqrt(l2a^2 + l3a^2),                  0);
A23 = MDH(q3 - atan2(l2a,l3a) + pi,               0,              l4a,                           0);

%% Newton-Euler's recursive algorithm:

% Step 1: computation of the remaining Aij HTMs:

A02 = simplify(A01*A12);
A03 = simplify(A01 * A12 * A23);
A13 = simplify(A12*A23);

% Aii matrices should be the identity matrix:

A00 = eye(4);
A11 = eye(4);
A22 = eye(4);
A33 = eye(4);


% Step 2: initial conditions:

w00 = [0 0 0]';
dw00 = [0 0 0]';
v00 = [0 0 0]';
z0 = [0 0 1]';



dv00 = [0 0 g]';  % Esta puede cambiar dependiendo de la configuración del robot


p11 = R10 * A01(1:3,4);                                  %[0 l1a 0]';                      % Vector from the origin of CF 0 to the origin of CF 1, expressed in CF 1 
p22 = R21 * A12(1:3,4);                                  %[-sqrt(l2a^2 + l3a^2) 0 0]';     % Vector from the origin of CF 1 to the origin of CF 2, expressed in CF 2 
p33 = R32 * A23(1:3,4);                                  %[l4a 0 0]';

s11 = [0 0 0]';                    % Coordinates of the COM of link 1 with respect to CF 1
s22 = [0 0 0]';                      % Coordinates of the COM of link 2 with respect to CF 2
s33 = [0 0 0]';

I11 = zeros(3);                    % Inertia matrix of link 1 with respect to its COM, expressed in a CF parallel to CF 1 and with origin at the COM
I22 = zeros(3);                    % Inertia matrix of link 2 with respect to its COM, expressed in a CF parallel to CF 2 and with origin at the COM
I33 = zeros(3);

% Step 3: computation of the rotation matrices and their inverse:

R01 = A01(1:3,1:3);
R02 = A02(1:3,1:3);
R03 = A03(1:3,1:3);


R12 = A12(1:3,1:3);
R13 = A13(1:3,1:3);

% R23 = eye(3);                       % Rotation matrix between CF 2 and the TCP
R23 = A23 (1:3,1:3);

R34 = eye(3);                           % Rotation matrix between CF 3 and the TCP

R10 = R01';
R20 = R02';
R21 = R12';
R32 = R23';
R30 = R03';
R31 = R13';
R43 = R34';

R04 = R03 * R34;
R40 = R04';

% Step 4: computation of the angular velocities:
if j1 == "r"
    w11 = R10*(w00 + z0*q1dot);	
else 
    w11 = R10*w00;
end

if j2 == "r"
    w22 = R21*(w11 + z0*q2dot);
else
    w22 = R21*w11;				% P
end

if j3 == "r"
    w33 = R32 *(w22 + z0*q3dot);
else 
    w33 = R32*w22;				% P
end

% Step 5: computation of the angular accelerations:

if j1 == "r"
    dw11 = R10*(dw00 + z0*q1ddot + cross(w00,z0*q1dot));
else 
    dw11 = R10*dw00;
end

if j2 == "r"
    dw22 = R21*(dw11 + z0*q2ddot + cross(w11,z0*q2dot));
else
    dw22 = R21*dw11;
end

if j3 == "r"
    dw33 = R32*(dw22 + z0*q3ddot + cross(w22,z0*q3dot));
else
    dw33 = R32*dw22;
end

% Step 6: computation of the CF linear accelerations:
if j1 == "r"
    dv11 = cross(dw11,p11) + cross(w11,cross(w11,p11)) + R10*dv00;
else
    dv11 = R10*(z0*q1ddot + dv00) + cross(dw11,p11) + 2*cross(w11,R10*z0*q1dot) + cross(w11,cross(w11,p11)); 
end

if j2 == "r"
    dv22 = cross(dw22,p22) + cross(w22,cross(w22,p22)) + R21*dv11;
else 
    dv22 = R21*(z0*q2ddot + dv11) + cross(dw22,p22) + 2*cross(w22,R21*z0*q2dot) + cross(w22,cross(w22,p22)); 
end

if j3 == "r"
    dv33 = cross(dw33,p33) + cross(w33,cross(w33,p33)) + R32*dv22;
else 
    dv33 = R32*(z0*q3ddot + dv22) + cross(dw33,p33) + 2*cross(w33,R32*z0*q3dot) + cross(w33,cross(w33,p33)); 
end


% Step 7: computation of the COM linear accelerations:  % Esto no se toca 

a11 = cross(dw11,s11) + cross(w11,cross(w11,s11)) + dv11;
a22 = cross(dw22,s22) + cross(w22,cross(w22,s22)) + dv22;
a33 = cross(dw33,s33) + cross(w33,cross(w33,s33)) + dv33;


% Step 8: computation of the FORCES exerted on links:  % No se toca solo si hay un range(peso o en effector) F en el end effector

% Si tenemos una fza en el sistema como 0f4 = [0 F 0]' -> 4f4 = 4R0 * 0f4  || 4R0 = 3R0

% f44 = R40 * [0; F; 0]; % Descomentar esta y comentar la otra si hay
% fuerza en el end effector
f44 = zeros(3,1);


f33 = R34*f44 + m3*a33;
f22 = R23*f33 + m2*a22;
f11 = R12*f22 + m1*a11;

% Step 9: computation of the TORQUES exerted on links:   % No se toca solo si hay un range(peso o en effector) F en el end effector

n44 = zeros(3,1); % Esto cambia si hay algo pegado empujando o siendo llevado por el end effector 

n33 = R34*(n44 + cross((R43*p33),f44)) + cross((p33+s33),m3*a33) + I33*dw33 + cross(w33,(I33*w33));
n22 = R23*(n33 + cross((R32*p22),f33)) + cross((p22+s22),m2*a22) + I22*dw22 + cross(w22,(I22*w22));
n11 = R12*(n22 + cross((R21*p11),f22)) + cross((p11+s11),m1*a11) + I11*dw11 + cross(w11,(I11*w11));

% Step 10: computation of the force/torque exerted on joints:

if j1 == "r"
    tau1_ne = simplify(n11'*R10*z0);
    Q_ne (1,1) = tau1_ne;
else
    F1_ne = simplify(f11'*R10*z0);
    Q_ne (1,1) = F1_ne;
end

if j2 == "r"
    tau2_ne = simplify(n22'*R21*z0);
    Q_ne (1,2) = tau2_ne;
else 
    F2_ne = simplify(f22'*R21*z0);
    Q_ne (1,2) = F2_ne;
end

if j3 == "r"
    tau3_ne = simplify(n33'*R32*z0);
    Q_ne (1,3) = tau3_ne;
else
    F3_ne = simplify(f33'*R32*z0);
    Q_ne (1,3) = F3_ne;
end

% disp(Q_ne)

 function dh = MDH(theta,d,a,alfa)

dh = trotz(theta)*transl([0 0 d])*transl([a 0 0])*trotx(alfa);

 end