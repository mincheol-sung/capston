% Tutorial_1.1

Ac=[0 1 0;3 0 1;0 1 0];
Bc=[1;1;3];
Cc=[0 1 0];
Dc=zeros(1,1);
Delta_t=1;
[Ap,Bp,Cp,Dp]=c2dm(Ac,Bc,Cc,Dc,Delta_t);

[m1,n1]=size(Cp);
[n1,n_in]=size(Bp);
A_e=eye(n1+m1,n1+m1);
A_e(1:n1,1:n1)=Ap;
A_e(n1+1:n1+m1,1:n1)=Cp*Ap;
B_e=zeros(n1+m1,n_in);
B_e(1:n1,:)=Bp;
B_e(n1+1:n1+m1,:)=Cp*Bp;
C_e=zeros(m1,n1+m1);
C_e(:,n1+1:n1+m1)=eye(m1,m1);