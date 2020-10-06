function [x,y] = XY(q1, q2)
%Find the x and y position of the end effector given q

%define values
L1 = 1; %m
L2 = 1; %m

%find on
on = [L1*cos(q1)+L2*cos(q1+q2);L1*sin(q1)+L2*sin(q1+q2)];

x = on(1);
y = on(2);
end

