function [f_state] = Robot(i_state, torques, freq)
%Given the initial state and motor torques for the manipulator in Ch.6,
%p.87 of the notes, compute the final state

%define values
L1 = 1; %m
L2 = 1; %m
m1 = 1; %kg
m2 = 1; %kg
g = 9.81; %m/s^2

%set up angle array
q1i = i_state(1);
q2i = i_state(2);
q1doti = i_state(3);
q2doti = i_state(4);

%set up D,C and G matrices
D = [((m1+m2)*L1^2+m2*L2^2+2*m2*L1*L2*cos(q2i)),(m2*L2^2+m2*L1*L2*cos(q2i));...
    (m2*L2^2+m2*L1*L2*cos(q2i)),(m2*L2^2)];
C = [(-2*m2*L1*L2*(sin(q2i))*q2doti),(-1*m2*L1*L2*(sin(q2i))*q2doti);...
    (m2*L1*L2*(sin(q2i))*q1doti),0];
G = [((m1+m2)*g*L1*cos(q1i)+m2*g*L2*cos(q1i+q2i));(m2*g*L2*cos(q1i+q2i))];

%given inputs solve for initial angular acceleration of joints
accel = D\(torques - G - C*([q1doti;q2doti]));

q1dubdoti = accel(1);
q2dubdoti = accel(2);

% find next state given initial state using kinematics equations
q1f = q1i+(1/freq)*q1doti+0.5*(1/freq)^2*q1dubdoti;
q2f = q2i+(1/freq)*q2doti+0.5*(1/freq)^2*q2dubdoti;
q1dotf = q1doti+(1/freq)*q1dubdoti;
q2dotf = q2doti+(1/freq)*q2dubdoti;

f_state = [q1f;q2f;q1dotf;q2dotf];
end

