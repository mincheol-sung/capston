%Tutorial 1.3

Ap=[1 1;0 1];
Bp=[0.5;1];
Cp=[1 0];
Dp=0;
Np=20;
Nc=4;

[Phi_Phi,Phi_F,Phi_R,A_e,B_e,C_e]=mpcgain(Ap,Bp,Cp,Nc,Np);

Kmpc=[1 0 0 0]*inv(Phi_Phi+rw*eye(Nc,Nc))*Phi_F;
Ky=[1 0 0 0]*inv(Phi_Phi+rw*eye(Nc,Nc))*Phi_R;

[n,n_in]=size(B_e);
xm=[0;0];
Xf=zeros(n,1);
N_sim=100;
r=ones(N_sim,1);
u=0;  
y=0;
rw=2;

for kk=1:N_sim;
    deltau=Ky*r(kk)-Kmpc*Xf;
    u=u+deltau;
    u1(kk)=u;
    y1(kk)=y;
    xm_old=xm;
    xm=Ap*xm+Bp*u;
    y=Cp*xm;
    Xf=[xm-xm_old;y];
end

k=0:(N_sim-1);
figure
subplot(211)
plot(k,y1)
xlabel('Sampling Instant')
legend('Output')
subplot(212)
plot(k,u1)
xlabel('Sampling Instant')
legend('Control')