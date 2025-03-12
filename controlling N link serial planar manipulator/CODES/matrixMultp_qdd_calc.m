m1 = 10; m2 = 5;
l1 = 0.2; l2 = 0.1;
g = 9.81;

syms q1; syms q2;
syms q1_dot; syms q2_dot;
syms tau1; syms tau2;

q = [q1; q2];
syms x1; syms x2;

q_dot = [q1_dot; q2_dot];

tau = [tau1; tau2];

M11 = (m1+m2)*(l1^2) + m2*l2*(l2+2*l1*cos(q2));
M12 = m2*l2*(l2+l1*cos(q2));
M22 = m2*(l2^2);

C11 = -m2*l1*l2*sin(q2)*q2_dot;
C12 = -m2*l1*l2*sin(q2)*(q1_dot+q2_dot);
C21 = 0;
C22 = m2*l1*l2*sin(q2)*q2_dot;

G1 = m1*l1*g*cos(q1)+m2*g*(l2*cos(q1+q2)+l1*cos(q1));
G2 = m2*g*l2*cos(q1+q2);

M = [M11, M12;
    M12, M22];

C = [C11, C12;
    C21, C22];

G = [G1; G2];

% tau = M*q_dd + C*q_dot + G;
q_dd = (M^(-1))*(tau - C*q_dot - G);
q1_dd = q_dd(1);
q2_dd = q_dd(2);
disp(q_dd);

q_desired = [0; 0];

e1 = q_desired(1) - q1;
e2 = q_desired(2) - q2;

% To calculate tau1 & tau2 if control variables are given

syms Kp1; syms Kp2;
syms Kd1; syms Kd2;
syms Ki1; syms Ki2;

f1 = Kp1*e1 + Ki1*x1 - Kd1*q1_dot;
f2 = Kp2*e2 + Ki2*x2 - Kd2*q2_dot;
F = [f1; f2];

tau = M*F;
tau1 = tau(1);
tau2 = tau(2);

% disp(f1);

% qdd = 
% [- (5*(tau1 - (981*cos(q1 + q2))/200 - (2943*cos(q1))/100 +
% (q1_dot*q2_dot*sin(q2))/10 + (q2_dot*sin(q2)*(q1_dot +
% q2_dot))/10))/(cos(q2)^2 - 3) - (5*(2*cos(q2) + 1)*((sin(q2)*q2_dot^2)/10 - tau2 + (981*cos(q1 + q2))/200))/(cos(q2)^2 - 3); 
% (5*(2*cos(q2) + 1)*(tau1 - (981*cos(q1 + q2))/200 - (2943*cos(q1))/100 +
% (q1_dot*q2_dot*sin(q2))/10 + (q2_dot*sin(q2)*(q1_dot + q2_dot))/10))/(cos(q2)^2 - 3) + (5*(4*cos(q2) + 13)*((sin(q2)*q2_dot^2)/10 - tau2 + (981*cos(q1 + q2))/200))/(cos(q2)^2 - 3)]
