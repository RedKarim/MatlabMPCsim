global T;
%LL=L0;
Umn=-7;Umx=2; 
dt=0.5;
T=20; %horizon
cx=3*(T+1)-1; %% X,V and A total dimension
%ub=[]; lb=[];
lb=nan(cx,1);
ub=lb;
lb(1:T+1)=-Inf;  %% Position limits
ub(1:T+1)=Inf;
lb(T+2:2*T+2)=0; %% minimum speed
ub(T+2:2*T+2)=18; %% maximu speed
lb(2*T+3:end)=Umn; %% minimum acceleration
ub(2*T+3:end)=Umx; %% maximu acceleration

%% Equilatiy 
Ax=1.0; Bx=dt; dt2=0.5*dt^2;
Aeq= zeros(cx,cx);

T1=-eye(T+1); T1(2:end,1:T)=eye(T)*Ax+T1(2:end,1:T);
Aeq(1:T+1,1:T+1)=T1;Aeq(T+2:2*T+2,T+2:2*T+2)=T1;
Aeq(2:T+1,T+2:2*T+1)=eye(T)*Bx; 
Aeq(2:T+1,2*T+3:3*T+2)=eye(T)*dt2;
Aeq(T+3:2*T+2,2*T+3:3*T+2)=eye(T)*dt; 
xi=zeros(cx,1); x=xi;

Beq=zeros(cx,1); 

Xp=100; Vp=11;
Xh=80; Vh=0; 
xi(1)=Xh;  xi(T+2)=Vh; 
Beq(1)=-Xh; Beq(T+2)=-Vh;
XPT=[[Xp+((1:T)*Vp*dt)]'];

%% Initial Guess
a0=-0.5;
for J=1:T
    xi(J+1)= xi(J)+ xi(T+1+J)*dt+0.5*a0*dt^2;
    xi(T+1+J+1)=xi(T+1+J)+a0*dt;
end
xi(2*(T+1)+1:end)=a0;
b=zeros(T,1);
b(1:T,1)=XPT-7; %% Gap with PV
b(1:3,1)=b(1:3,1)+4  %% Gap with PV

A=zeros(T,cx); 
A(1:T,2:T+1)=eye(T);
A(1:T,T+3:2*T+2)=0.5*eye(T);

%% Otpmize

options=optimset('Algorithm','sqp','GradObj','off');
[x,fav,exitflag,output]=fmincon(@ObjFn,xi,A,b,Aeq,Beq,lb,ub);%,@NonLinCons,options
fav
x'
