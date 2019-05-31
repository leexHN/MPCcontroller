% 已知连续状态方程为
A=[0 1;-1 -2];
B=[0 1]';
C=[1 0];
D=0;

Ts=0.1;
Ad=(eye(2)+Ts*A/2)/((eye(2)-Ts*A/2));
Bd=Ts*B;
Cd=C;
Dd=D;

Np=10;
Nc=2;
Nu=1;
Ny=1;
Ns=2;

U0=1;
Utest=ones(Nu*Nc,1);

Lb = repmat([-1],[Nc,1]);
Ub =  repmat([2],[Nc,1]);
SLb=repmat([-5],[Nc,1]);
SUb=repmat([0.5],[Nc,1]);
% syms Ad Bd Cd
THETA_cell=cell(Np,Nc);
for i=1:Np
    for j=1:Nc
        if(j<=i)
            THETA_cell{i,j}=Cd*Ad^(i-j)*Bd;
        else
            THETA_cell{i,j}=0;
        end
    end
end
THETA=cell2mat(THETA_cell);

Z=(0.1:0.1:0.1*Nc)';
Z_input=[(0:Ts:(Np-1)*Ts)' [Z;ones(Np-Nc,1)*0]];


PSI_cell=cell(Np,1);
for i=1:Np
    PSI_cell{i,1}=Cd*Ad^i;
end
PSI=cell2mat(PSI_cell);

X0=[1 2]';
Y=PSI*X0+THETA*Z;

% plot(simout.signals.values(2:end)); hold on
% plot(Y);

% QP问题转化 Y=PSI*X0+THETA*Z; 另J=(Y-ref)'Q(Y-ref)+Z'RZ
Q=2*eye(Np,Np); %这里Q不是！！通用格式只是正好输出为1个
R=3*eye(Nc,Nc);% 同上
ref=4*ones(Np,1); % 同上

J=(Y-ref)'*Q*(Y-ref)+Z'*R*Z;
J2=Z'*(THETA'*Q*THETA+R)*Z + 2*(PSI*X0-ref)'*Q*THETA*Z + (PSI*X0-ref)'*Q*(PSI*X0-ref);
disp(J-J2)

H = THETA'*Q*THETA+R;
F= 2*(PSI*X0-ref)'*Q*THETA;
Mu=zeros(2*Nu*Nc,1);
for i=1:2*Nc
    if(i<=Nc)
        Mu(i:i+Nu-1,1)=U0;
    else
        Mu(i:i+Nu-1,1)=-U0;
    end
end
% 边界条件
M1=zeros(2*Nc*Nu,Nc*Nu);
M1=[eye(Nc*Nu,Nc*Nu);-eye(Nc*Nu,Nc*Nu)];
D1 = [Ub;-Lb];
temp_m1=eye(Nc,Nc);
for i=1:Nc
    for j=1:Nc
        if(i>j)
            temp_m1(i,j)=1;
        end
    end
end
M2=[kron(temp_m1,eye(Nu,Nu));-kron(temp_m1,eye(Nu,Nu))];
D2 = [SUb;-SLb];
% NoSlackQpSlover
qp_c=[M1;M2];
qp_d=[D1;D2-Mu];

% qp_c*Utest
[x,fval,exitflag,output,lambda] = quadprog(2*H,F,qp_c,qp_d);