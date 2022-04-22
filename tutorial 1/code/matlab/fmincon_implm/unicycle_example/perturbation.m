function z = perturbation(phi,t, e,a,b,mu)

a1 = a(1)*e*cos(t+phi);
alpha = a1;
b1 = b(1)*e*cos(t+phi);
bd = -b(1)*e*sin(t+phi);
beta = [b1;0;bd];

z = [beta;mu*(1-mu)*alpha];

end

