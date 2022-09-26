%% feedback controller
A=[0 -pi/16 0;pi/16 0 3*pi/16;0 0 0];
B=[1/2 1/2;0 0;1/0.15 -1/0.15];
C=[1 0 0;0 1 0];
D=[0 0;0 0];

F=[-1 4.899 0;-4.899 -1 0;0 0 -30];
K=[1 0 0;0 0 1];
n = rank(obsv(F, K))
p =lyap(A,-F,-B*K)
New_K = K* inv(p);
SYS= ss(A-B*New_K,B,C,D);
pole(SYS);

K1 = place(A, B, [-30 -1+4.899*i -1-4.899*i]) ;
t = 0 : 0.001 : 32 ;
%% static Controller
%syms s;
G_closeLoop = inv(C*inv(-(A-B*K1))*B)
%% integral control
% syms k11 k12 k21 k22 s
% Ka = [k11 k12; k21 k22];
% delta_f = [(s*eye(3)-A+B*K1) -B*Ka; C s]
% K_X = []
A_bar = [0 -pi/16 0 0 0;pi/16 0 3*pi/16 0 0; 0 0 0 0 0; 1 0 0 0 0; 0 1 0 0 0];
B_bar = [1/2 1/2;0 0;1/0.15 -1/0.15; 0 0; 0 0];
C_bar = [1 0 0 0 0; 0 1 0 0 0];
D_bar = [0 0 0 0 0; 0 0 0 0 0];
K_bar = place(A_bar, B_bar, [-60 -55 -70 -1+4.899*i -1-4.899*i ]);
Ka = K_bar(:,4:5);
K_f = K_bar(:,1:3);
%% desired path
a_m = 3;
dt = 1/1e3;
StopTime = 10;
t = (0:dt:StopTime-dt);
X_ref = a_m * cos(pi/16 *t);
Y_ref = -a_m*sin(pi/16 *t)
path = [X_ref; Y_ref];
%figure(1)
% plot(X_ref, Y_ref)
% xlim([-10 10])
% ylim([-10 10])

%% full-order observer
L = place(A', C', [-20, -20, -100]);
L = L'
%% reduced-order observer
F = -40;
L_R = [1 -400];
T = lyap(-F, A, -L_R*C);
Q = inv([1 0 0; 0 1 0; T]);
Q1 = [1 0; 0 1; -0.5031 67.9061];
Q2 = [0 0 6.7909]';
%% phase4
A_original1 = 1.2*A;
A_original2 = 5*A;
%% static controller-modifiedSys
%G_closeLoop_modify = inv((C*inv(-(A-B*K1))*B)/(1+C*inv(-(A-B*K1))*B))
s = 0;
G_closeLoop_modify = (C*(s-(A-B*K1))^(-1)*B)/(1+C*(s-(A-B*K1))^(-1)*B);

staticGain = inv(G_closeLoop_modify)




