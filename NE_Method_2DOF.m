
% clear all
% close all
% clc

%% Definition of symbolic variables:

syms q1 q2 real;
syms q1dot q2dot real;
syms q1ddot q2ddot real;
pi = sym(pi);

syms L1 real;
syms m1 m2 g real;

%% D-H matrices:

%              theta               d               a           alpha
A01 = MDH(q1,              0,              0,            -pi/2   );
A12 = MDH( 0,             q2,              0,                 0   );

%% Newton-Euler's recursive algorithm:

% Step 1: computation of the remaining Aij HTMs:

A02 = simplify(A01*A12);

% Step 2: initial conditions:

w00 = [0 0 0]';
dw00 = [0 0 0]';
v00 = [0 0 0]';
dv00 = [0 0 g]';
z0 = [0 0 1]';

p11 = [0 0 0]';                      % Vector from the origin of CF 0 to the origin of CF 1, expressed in CF 1 
p22 = [0 0 q2]';                    % Vector from the origin of CF 1 to the origin of CF 2, expressed in CF 2 
s11 = [0 0 L1]';                    % Coordinates of the COM of link 1 with respect to CF 1
s22 = [0 0 0]';                      % Coordinates of the COM of link 2 with respect to CF 2
I11 = zeros(3);                    % Inertia matrix of link 1 with respect to its COM, expressed in a CF parallel to CF 1 and with origin at the COM
I22 = zeros(3);                    % Inertia matrix of link 2 with respect to its COM, expressed in a CF parallel to CF 2 and with origin at the COM

% Step 3: computation of the rotation matrices and their inverse:

R01 = A01(1:3,1:3);
R02 = A02(1:3,1:3);
R12 = A12(1:3,1:3);
R23 = eye(3);                       % Rotation matrix between CF 2 and the TCP

R10 = R01';
R20 = R02';
R21 = R12';
R32 = R23';

% Step 4: computation of the angular velocities:

w11 = R10*(w00 + z0*q1dot);
w22 = R21*w11;

% Step 5: computation of the angular accelerations:

dw11 = R10*(dw00 + z0*q1ddot + cross(w00,z0*q1dot));
dw22 = R21*dw11;

% Step 6: computation of the CF linear accelerations:

dv11 = cross(dw11,p11) + cross(w11,cross(w11,p11)) + R10*dv00;
dv22 = R21*(z0*q2ddot + dv11) + cross(dw22,p22) + 2*cross(w22,R21*z0*q2dot) + cross(w22,cross(w22,p22)); 

% Step 7: computation of the COM linear accelerations:

a11 = cross(dw11,s11) + cross(w11,cross(w11,s11)) + dv11;
a22 = cross(dw22,s22) + cross(w22,cross(w22,s22)) + dv22;

% Step 8: computation of the forces exerted on links:

f33 = zeros(3,1);

f22 = R23*f33 + m2*a22;
f11 = R12*f22 + m1*a11;

% Step 9: computation of the torques exerted on links:

n33 = zeros(3,1);

n22 = R23*(n33 + cross((R32*p22),f33)) + cross((p22+s22),m2*a22) + I22*dw22 + cross(w22,(I22*w22));
n11 = R12*(n22 + cross((R21*p11),f22)) + cross((p11+s11),m1*a11) + I11*dw11 + cross(w11,(I11*w11));

% Step 10: computation of the force/torque exerted on joints:

tau1_ne = simplify(n11'*R10*z0)
F2_ne = simplify(f22'*R21*z0)

Q_ne = [tau1_ne;F2_ne];

 function dh = MDH(theta,d,a,alfa)

dh = trotz(theta)*transl([0 0 d])*transl([a 0 0])*trotx(alfa);

 end
