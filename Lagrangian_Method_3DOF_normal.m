%% Lagrangian Method for a 3 DOF robot
% clear all
% close all
% clc

%% Definition of symbolic variables:

syms q1 q2 q3 real;
syms q1dot q2dot q3dot real;
syms q1ddot q2ddot q3ddot real;
pi = sym(pi);

 function dh = MDH(theta,d,a,alfa)

    dh = trotz(theta)*transl([0 0 d])*transl([a 0 0])*trotx(alfa);

 end

 function J = pseudo_inertia(r,m)

x = r(1);
y = r(2);
z = r(3);

J = sym(zeros(4));
J(1,1) = m*x^2;
J(1,2) = m*x*y;
J(1,3) = m*x*z;
J(1,4) = m*x;
J(2,1) = m*x*y;
J(2,2) = m*y^2;
J(2,3) = m*y*z;
J(2,4) = m*y;
J(3,1) = m*x*z;
J(3,2) = m*y*z;
J(3,3) = m*z^2;
J(3,4) = m*z;
J(4,1) = m*x;
J(4,2) = m*y;
J(4,3) = m*z;
J(4,4) = m;

end



syms l1a l2a l3a l4a l5 l6 real;
syms m1 m2 m3 g real;

% assume (q2, "positive"); % Son ángulos, pueden ser negativos 
% assume (q3, "positive");


assume (l1a, "positive");
assume (l2a, "positive");
assume (l3a, "positive");
assume (l4a, "positive");
%assume (l5, "positive");
%assume (l6, "positive");

assume (m1, "positive");
assume (m2, "positive");
assume (m3, "positive");

assume (g, "positive");


%% D-H matrices:

%      theta                                    d               a                                   alpha
A01 = MDH(q1,                                   l1a,              0,                           pi/2);
A12 = MDH(q2 + atan2(l2a,l3a) + pi ,              0,      -sqrt(l2a^2 + l3a^2),                  0);
A23 = MDH(q3 - atan2(l2a,l3a) + pi,               0,              l4a,                           0);

%% Lagrangian algorithm:

% Steps 1 and 2: computation of the remaining Aij HTMs:

A02 = simplify(A01*A12);
A03 = simplify(A01 * A12 * A23);
A13 = simplify(A12*A23);

% Aii matrices should be the identity matrix:

A00 = eye(4);
A11 = eye(4);
A22 = eye(4);
A33 = eye(4);

% Step 3: computation of the Uij matrices

Qr = zeros(4); Qr(1,2) = -1; Qr(2,1) = 1;
Qt = zeros(4); Qt(3,4) = 1;

% =============ESTO CAMBIA=================
Q1 = Qr;
Q2 = Qr;
Q3 = Qr;
%=============FIN DEL CAMBIO===============

% for i = 1:3
%     for j = 1:3 
%         if j <= i 
%             U(i,j) = 

U11 = A00*Q1*A01;
U12 = sym(zeros(4));
U13 = sym(zeros(4));

U21 = A00*Q1*A02;
U22 = A01*Q2*A12;
U23 = sym(zeros(4));

U31 = A00*Q1*A03;
U32 = A01*Q2*A13;
U33 = A02*Q3*A23;

% Step 4: computation of the Uijk matrices (No se toca una vez lista)

U111 = A00*Q1*A00*Q1*A01;
U112 = sym(zeros(4));
U113 = sym(zeros(4));

U121 = sym(zeros(4));
U122 = sym(zeros(4));
U123 = sym(zeros(4));

U131 = sym(zeros(4));
U132 = sym(zeros(4));
U133 = sym(zeros(4));

U211 = A00*Q1*A00*Q1*A02;
U212 = A00*Q1*A01*Q2*A12;
U213 = sym(zeros(4));

U221 = A00*Q1*A01*Q2*A12;
U222 = A01*Q2*A11*Q2*A12;
U223 = sym(zeros(4));

U231 = sym(zeros(4));
U232 = sym(zeros(4));
U233 = sym(zeros(4));

U311 = A00*Q1*A00*Q1*A03;
U312 = A00*Q1*A01*Q2*A13;  % Interacción q1 y q2 sobre eslabón 3
U313 = A00*Q1*A02*Q3*A23;  % Interacción q1 y q3 sobre eslabón 3

U321 = A00*Q1*A01*Q2*A13;  % Simétrico a U312
U322 = A01*Q2*A11*Q2*A13;  % Doble derivada de q2 sobre eslabón 3
U323 = A01*Q2*A12*Q3*A23;  % Interacción q2 y q3 sobre eslabón 3

U331 = A00*Q1*A02*Q3*A23;  % Simétrico a U313
U332 = A01*Q2*A12*Q3*A23;  % Simétrico a U323
U333 = A02*Q3*A22*Q3*A23;  % Doble derivada de q3 sobre eslabón 3

% Step 5: computation of the Ji pseudo-inertia matrices: 

D = [2 9 5 5 5 5 5 8];

r11 = [0 0 0 1]'; % Esto son las coordenadas homogeneas del centro de masas 1 con respecto al eje de coordenadas 1
r22 = [0 0 0 1]';
r33 = [0 0 0 1]';

J1 = pseudo_inertia(r11,m1);
J2 = pseudo_inertia(r22,m2);
J3 = pseudo_inertia(r33,m3);

% Step 6: computation of the inertia matrix: (Una vez hecho no se toca)

M(1,1) = trace(U11*J1*U11') + trace(U21*J2*U21') + trace(U31*J3*U31');
M(1,2) = trace(U22*J2*U21') + trace(U32*J3*U31');
M(1,3) = trace(U33*J3*U31');

M(2,1) = M(1,2);
M(2,2) = trace(U22*J2*U22') + trace(U32*J3*U32');
M(2,3) = trace(U33*J3*U32');

M(3,1) = M(1,3);
M(3,2) = M(2,3);
M(3,3) = trace(U33*J3*U33');


% Step 7: computation of the cijk terms: (Una vez hecho no se toca)


c111 = 0;
c112 = trace(U212*J2*U21') + trace(U312*J3*U31');
c113 = trace(U313*J3*U31');

c121 = c112;
c122 = trace(U222*J2*U21') + trace(U322*J3*U31');
c123 = trace(U323*J3*U31');

c131 = c113;
c132 = c123;
c133 = trace(U333*J3*U31');

c211 = trace(U211*J2*U22') + trace(U311*J3*U32');
c212 = trace(U212*J2*U22') + trace(U312*J3*U32');
c213 = trace(U313*J3*U32');

c221 = c212;
c222 = 0;
c223 = trace(U323*J3*U32');

c231 = c213;
c232 = c223;
c233 = trace(U333*J3*U32');

c311 = trace(U311*J3*U33');
c312 = trace(U312*J3*U33');
c313 = trace(U313*J3*U33');

c321 = c312;
c322 = trace(U322*J3*U33');
c323 = trace(U323*J3*U33');

c331 = c313;
c332 = c323;
c333 = 0;

% Step 8: computation of the Coriolis matrix:

C(1,1) = simplify( c111*q1dot + c112*q2dot + c113*q3dot );
C(1,2) = simplify( c121*q1dot + c122*q2dot + c123*q3dot );
C(1,3) = simplify( c131*q1dot + c132*q2dot + c133*q3dot );

C(2,1) = simplify( c211*q1dot + c212*q2dot + c213*q3dot );
C(2,2) = simplify( c221*q1dot + c222*q2dot + c223*q3dot );
C(2,3) = simplify( c231*q1dot + c232*q2dot + c233*q3dot );

C(3,1) = simplify( c311*q1dot + c312*q2dot + c313*q3dot );
C(3,2) = simplify( c321*q1dot + c322*q2dot + c323*q3dot );
C(3,3) = simplify( c331*q1dot + c332*q2dot + c333*q3dot );

% Step 9: computation of the gravity term:

% g_vec = [0 0 -g 0]';
g_vec = [0 0 -g 0]';% ESTO CAMBIA DEPENDIENDO DEL ROBOT

term1_1 = m1 * g_vec' * U11 * r11;
term1_2 = m2 * g_vec' * U21 * r22;
term1_3 = m3 * g_vec' * U31 * r33;

G(1,1) = - (term1_1 + term1_2 + term1_3);

term2_1 = m1 * g_vec' * U12 * r11;
term2_2 = m2 * g_vec' * U22 * r22;
term2_3 = m3 * g_vec' * U32 * r33;

G(2,1) = - (term2_1 + term2_2 + term2_3);

term3_1 = m1 * g_vec' * U13 * r11;
term3_2 = m2 * g_vec' * U23 * r22;
term3_3 = m3 * g_vec' * U33 * r33;

G(3,1) = - (term3_1 + term3_2 + term3_3);

% Step 10: final dynamic model:

M = simplify(M)
C = simplify(C)
G = simplify(G)

% Separated dynamic equations for comparison with Newton-Euler's recursive algorithm:

Q_lagr = M*[q1ddot q2ddot q3ddot]' + C*[q1dot q2dot q3dot]' + G;

tau1_lagr = Q_lagr(1); % Estos nombres cambian en función de si son fuerzas o pares
F2_lagr = Q_lagr(2);
F3_lagr = Q_lagr(3);


% 
% Cosas a cambiar
% Paso 1, 2, 5 y 9 
