%Tutorial 1.2

Ap=0.8; Bp=0.1; Cp=1; Nc=4; Np=10;

[Phi_Phi,Phi_F,Phi_R,A_e,B_e,C_e]=mpcgain(Ap,Bp,Cp,Nc,Np);

x=[0.1;0.2];
r=1;
r_w=0;
BarR=r_w*eye(Nc,Nc);
delta_U=inv(Phi_Phi+BarR)*(Phi_R*r-Phi_F*x)