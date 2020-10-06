function [torques] = PDG_controller(set_point, Kp, Kv, state)
%given input set point and robot state output the desired torques based on
%the PD + G set point control method

%define values
L1 = 1; %m
L2 = 1; %m
m1 = 1; %kg
m2 = 1; %kg
g = 9.81; %m/s^2

%set q and qdot
q = state(1:2);
qdot = state(3:4);

%set up specfic angles
q1i = state(1);
q2i = state(2);

%set up G(q)
G = [((m1+m2)*g*L1*cos(q1i)+m2*g*L2*cos(q1i+q2i));(m2*g*L2*cos(q1i+q2i))];

torques = G + Kp*(set_point - q) - Kv*qdot;
end

