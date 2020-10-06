function [torques] = Stiffness_controller(set_pos, set_or,state,Kp,Kv)
%given input set point and robot state output the desired torques based on
%the stiffness control method

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

%set up Jacobian and find {on, Cn}
[J, on, Cn] = Jacobiani(q);
in = Cn(:,1);
jn = Cn(:,2);

id = set_or(:,1);
jd = set_or(:,2);

%set up [fn;tn] vector
e = (0.5*(cross([in;0],[id;0])+cross([jn;0],[jd;0])));
od_on = [(set_pos - on);e(3)];
ondot = J*qdot;
fn_tn = Kp*od_on - Kv*ondot;
    
%find torques
torques = G + (J.')*fn_tn;
end

