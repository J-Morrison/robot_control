function [J, on, Cn] = Jacobiani(q)
%this function finds jacobian given a set of input angles for the
%manipulator in ch.6 pg.87

%define values
L1 = 1; %m
L2 = 1; %m

%find jacobian
J1a = -1*L1*sin(q(1))-L2*sin(q(1)+q(2));
J1b = L1*cos(q(1))+L2*cos(q(1)+q(2));
J1c = 1;

J2a = -1*L2*sin(q(1)+q(2));
J2b = L2*cos(q(1)+q(2));
J2c = 1;

J1 = [J1a;J1b;J1c];
J2 = [J2a;J2b;J2c];
J = [J1,J2];

%find on
on = [L1*cos(q(1))+L2*cos(q(1)+q(2));L1*sin(q(1))+L2*sin(q(1)+q(2))];

%find Cn
ini = [cos(q(1)+q(2));sin(q(1)+q(2))];
in = ini/norm(ini);
jn = [-1*in(2);in(1)];
Cn = [in,jn];
end

