syms x y theta real
syms v delta_f real
syms L real
dx = v*cos(theta);
dy = v*sin(theta);
dtheta = v*tan(delta_f)/L;
f=[dx,dy,dtheta]';
s=[x y theta];
u=[v delta_f];
Ac = jacobian(f,s);
Bc = jacobian(f,u);
