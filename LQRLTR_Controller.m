% Design of Unity Feedback control System
% for a given plant model, using LQR/LTR Method.
% in completion of a project for the course
% EE7501 - Advanced Control Systems
% for Dr. Guoxiang Gu
% by Haranath Manikonda
% to be submitted on Dec 4th 2018
% Given Plant model
s = tf('s');
P = 1 / (s*(s-1))
W = 0.6;
Pw = P * W
% Weighting Function
a = 5;
epi = 9;
Ws =(((s+.3*a)/(s+3*a)));
ds = ((1-epi*s)/(1+epi*s))
%True System
Ptrue = Pw*(1+(Ws*ds))
%Finding ARE for given system
[b,a] = tfdata(Pw,'v');
[Aw,Bw,Cw,Dw] = tf2ss(b,a);
Qw = Cw'*Cw;
[Xw,L,Fw] = care(Aw,Bw,Qw)
% ARE for true system
[bd,ad] = tfdata(Ptrue,'v');
[Adw,Bdw,Cdw,Ddw] = tf2ss(bd,ad)
Qdw = Cdw'*Cdw;
[Xdw,Ld,Fdw] = care(Adw,Bdw,Qdw);
% Plotting Step response for Tw(s)
Aw1 = Aw - (Bw*Fw);
Cw1 = Cw - (Dw*Fw);
Tw = ss(Aw1,Bw,Cw1,0);
figure(1);
step(Tw);
title('Step response for Tw(s)');
grid on
% Loop Transfer Recovery
Aw2 = Aw';
Bw2 = Cw';
q2 = 100000;
Qw2 = q2*Bw*Bw'
[Yq,L2,Lq] = care(Aw2,Bw2,Qw2)
% Finding K (Observer based Controller) value
Ak = Aw-(Bw*Fw)-(Lq'*Cw)+(Lq'*Dw*Fw) ;
Bk = -Lq';
Ck = Fw;
Dk = Dw
[b,a]= ss2tf(Ak,Bk,Ck,Dk)
K = tf(b,a)
% Loop Gain at Output Feedback
KP = K*Pw
% Finding LW(s)- Loop Gain at State Feedback
Alw = Aw;
Blw = Bw;
Clw = -Fw;
Dlw = 0;
[bL,aL]= ss2tf(Alw,Blw,Clw,Dlw)
LW = tf(bL,aL)
% Plotting the magnitude responses
% of Pw, KP, LW.
figure(2)
bode(Pw);
hold on
bode(KP);
hold on
bode(LW)
grid on
legend('Pw','KP','LW');
% Realization of Closed Loop System
At = [Adw,Bdw*Fw;-(Lq'*Cdw),Ak-(Lq'*Ddw*Fw)]
Bt = [Bdw/1.745;Bw]
Ct = [Cdw Ddw*Fw]
Dt = Ddw
% Plotting the step responses for Try
% using lsim with linear and non linear
% r(t)inputs.
tmax = 60;
sz = size(At);
n = sz(1);
t= 0:0.01:tmax;
N = length(t);
Try = ss(At,Bt,Ct,Dt);
u = [2*ones(20,1);1.5*ones(50,1);1.20*ones(30,1);1.15*ones(50,1);...
1.1*ones(50,1);1.05*ones(50,1);ones(3750,1);ones(2001,1)];
y = lsim(Try,u,t,zeros(n,1));
y1 = lsim(Try, ones(1,N),t,zeros(n,1));
figure(3);
plot(t,y,t,y1)
legend('Try improved','Try actual');
title('Step response for Try(s)with linear and non linear r(t)')
grid on