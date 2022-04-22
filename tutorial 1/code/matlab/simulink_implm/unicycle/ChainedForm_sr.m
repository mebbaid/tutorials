function z = ChainedForm_sr(x, u)
delta = 1;
z = zeros(3,1);
z(1) = x(1)+delta*u(1);
z(2) = x(2)+delta*u(2);
z(3) = x(3)+delta*x(2)*u(1);
end

