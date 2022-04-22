function [poles, Acl] = eig_closedloop(Ad,Bd,Cd,Np,Nc,Ts)
%EIG_CLOSEDLOOP Summary of this function goes here
%   Computation of the prediction matrices, and closed loop poles
%   change with change in Nc
[m1,n1]=size(Cd);
[n1,n_in]=size(Bd);
A_d=sym(Ad);
B_d=sym(Bd);
C_d=sym(Cd);

n=n1;
h=zeros(Np*m1,n);
h=sym(h);
F=zeros(Np*m1,n);
F=sym(F);
h(1:m1,:)=C_d;
F(1:m1,:)=C_d*A_d;
for kk=2:Np
    h((kk-1)*m1+1:kk*m1,:)=h((kk-2)*m1+1:(kk-1)*m1,:)*A_d;
    F((kk-1)*m1+1:kk*m1,:)= F((kk-2)*m1+1:(kk-1)*m1,:)*A_d;
end
v=h*B_d;
Phi=zeros(Np*m1,Nc*n_in); %declare the dimension of Phi
Phi=sym(Phi);
Phi(:,1:n_in)=v; % first column of Phi
for i=2:Nc
    Phi(:,(i-1)*n_in+1:i*n_in)=[zeros((i-1)*m1,n_in);v(1:(Np-i+1)*m1,:)]; %Toeplitz matrix
end
BarRs=repmat(eye(m1),Np,1);
BarRs=sym(BarRs);
Phi_Phi= Phi'*Phi;
Phi_F= Phi'*F;
Phi_R=Phi'*BarRs;
barR=eye(Nc*n_in,Nc*n_in);
RecHor = zeros(1,Nc*m1);
RecHor(1) = 1;
Acl = Ad-Bd*RecHor*(inv(Phi_Phi)*Phi_F);
poles = vpa(eig(Acl));

end

