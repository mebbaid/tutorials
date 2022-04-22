function [t0, x0,u0] = plant(T,t0, x0, u)
% unicycle exact sd model

st = x0;
cn = u(1,:)';
st = st + [ cn(1)/cn(2)*(sin(st(3) + T*cn(2)) - sin(st(3)));
            cn(1)/cn(2)*(cos(st(3)) - cos(st(3) +  T*cn(2)));
            T*cn(2)];
x0 = full(st);
t0 = t0 + T;
u0 = [u(2:size(u,1),:); u(size(u,1),:)];

end