function j = xy_axis_mpc(K,dt,p_0,v_0,a_0,pt,vt,at)
w1 = 100;
w2 = 1;
w3 = 1;
w4 = 1;
%% Construct the prediction matrix
[Tp, Tv, Ta, Bp, Bv, Ba] = getPredictionMatrix(K,dt,p_0,v_0,a_0);

%% Construct the optimization problem
H = w4*eye(K)+w1*(Tp'*Tp)+w2*(Tv'*Tv)+w3*(Ta'*Ta);
F = w1*(Bp-pt)'*Tp+w2*(Bv-vt)'*Tv+w3*(Ba-at)'*Ta;

A = [Tv;-Tv;Ta;-Ta];
b = [6*ones(20,1)-Bv;6*ones(20,1)+Bv;3*ones(20,1)-Ba;3*ones(20,1)+Ba];
%% Solve the optimization problem
J = quadprog(H,F,A,b,[],[],-3*ones(20,1),3*ones(20,1));

%% Apply the control
j = J(1);
end