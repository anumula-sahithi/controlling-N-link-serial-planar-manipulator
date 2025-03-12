clear; clc;
% Initial joint angular velocities (q_dot) for both joints
qdot_initial = [0; 0];
time_span = [0 10];

q_initial = [0.1; 0.1];
% Initial integral errors for both joints (for use in integral control)
x_initial = [0; 0];

qRequired = [0; 0];

% Simulating the system using ode45
[t, states] = ode45(@dynamics_manipulator, time_span, [q_initial; qdot_initial; x_initial]);

%joint angles
q1 = states(:,1);
q2 = states(:,2);

x1 = states(:,5);
x2 = states(:,6);

q1_dot = states(:,3);
q2_dot = states(:,4);



e1 = qRequired(1) - q1;
e2 = qRequired(2) - q2;
% for title
Kp = 250; Kd = 10; Ki = 0;

% Plot q1 and q2 versus time
figure; 
subplot(2,1,1);
plot(t, q1);
title("q1");
xlabel("Time (s)");
ylabel("q1(t)");

% error
subplot(2,1,2);
plot(t, q2);
title("q2");
xlabel("Time (s)");
ylabel("q2(t)");

sgtitle("q1, q2 vs t for Kp = " + Kp + ", Kd = " + Kd);


figure; subplot(2,1,1);
plot(t,e1,'r');
title("Error in q1");
xlabel("Time(s)");
ylabel("e_1(t)");
subplot(2,1,2);
plot(t,e2,'r');
title("Error in q2");
xlabel("Time(s)");
ylabel("e_2(t)");

sgtitle("Error plots for Kp = " + Kp + ", Kd = " + Kd);

function dx_dt = dynamics_manipulator(t, states)
% Extract states from input (joint angles, velocities, and integral errors)
    q1_dot = states(3);
    q2_dot = states(4);

    x1 = states(5);
    x2 = states(6);

    q1 = states(1);
    q2 = states(2);
    
    % PID controller gains for each joint (same for q1 and q2)
    Kp1 = 250; Kd1 = 10; Ki1 = 0;
    Kp2 = 250; Kd2 = 10; Ki2 = 0;
    
    q_desired = [0; 0];

    e1 = q_desired(1) - q1;
    e2 = q_desired(2) - q2;

    f1 = Kp1*e1 + Ki1*x1 - Kd1*q1_dot;
    f2 = Kp2*e2 + Ki2*x2 - Kd2*q2_dot;
    F = [f1; f2];

    m1 = 10; m2 = 5;
    l1 = 0.2; l2 = 0.1;
 % Inertia matrix for the two-link manipulator
    M11 = (m1+m2)*(l1^2) + m2*l2*(l2+2*l1*cos(q2));
    M22 = m2*(l2^2);
    M12 = m2*l2*(l2+l1*cos(q2));
   
    M = [M11, M12; M12, M22];

    %the joint torques based on the control inputs and inertia matrix
    tau = M*F;
    tau1 = tau(1);
    tau2 = tau(2);

    %Dynamics of the system
    dx_dt = zeros(size(states));
    dx_dt(1) = q1_dot;
    dx_dt(2) = q2_dot;
   
    dx_dt(3) = - (5*(tau1 - (981*cos(q1 + q2))/200 - (2943*cos(q1))/100 + (q1_dot*q2_dot*sin(q2))/10 + (q2_dot*sin(q2)*(q1_dot + q2_dot))/10))/(cos(q2)^2 - 3) - (5*(2*cos(q2) + 1)*((sin(q2)*q2_dot^2)/10 - tau2 + (981*cos(q1 + q2))/200))/(cos(q2)^2 - 3);
    dx_dt(4) = (5*(2*cos(q2) + 1)*(tau1 - (981*cos(q1 + q2))/200 - (2943*cos(q1))/100 + (q1_dot*q2_dot*sin(q2))/10 + (q2_dot*sin(q2)*(q1_dot + q2_dot))/10))/(cos(q2)^2 - 3) + (5*(4*cos(q2) + 13)*((sin(q2)*q2_dot^2)/10 - tau2 + (981*cos(q1 + q2))/200))/(cos(q2)^2 - 3);
    
    dx_dt(6) = q_desired(2) - q2;
    dx_dt(5) = q_desired(1) - q1;

end