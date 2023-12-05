[J,umax] = lab3robot(020712);
s = tf('s');
G = tf([38],[180 1930 800 0]);
lab3robot(G, 020712)
%%
K = 4.55;
F = tf(K,1);
lab3robot(G,K,F,0,0,0,0,0,020712)
%%
G_o = F*G;
G_c = F*G/(1+F*G);
[Gm, Pm, Wcg, Wcp] = margin(G_o);
fb = bandwidth(G_c);
disp(fb);
disp(Pm);
disp(Wcg);
%%
%The gain plot is shifted to the right indicating an increase of gain and
%an increase of phase margin since the phase plot remains the same

%Bandwidth limits it since the response time is roughly inversely
%proportional. In turn, this means that the stability of the system limits
%how much gain we can put on

%Increasing the gain too much stops the robot arm from showing up on the
%diagram.

%%
%The frequency domain property that is most closely related to the rise time
%is the bandwidth since it's roughly inversely proportional to
%the rise time. Increasing the bandwidth by a factor of 4 should therefore
%roughly decrease the rise time by a factor of 4. From lab 2 we know that
%in the open loop system,
%the bandwidth is roughly proportional to the crossover frequency which we
%then also need to increase by a factor of 4.

%The resonant peak of the frequency domain is roughly proportional to the
%overshoot. The resonant peak has a lower limit decided by the phase margin
%meaning we should try to keep it the same.

%%
%Pm increase at desired crossover frequency omega_cd = 0.788, is 47.9
%degrees accounting for the phase decrease from the lag component.
K = 4.55;
omega_cd = 0.788;
beta = 0.15;
tau_i = 14/omega_cd;
gamma = 0.03;
tau_d = 1/(omega_cd*sqrt(beta));
K_tau=10^(22.5/20);

F_lead = K_tau*(tau_d*s +1)/(beta*tau_d*s + 1);
F_lag = (tau_i*s + 1)/(tau_i*s + gamma);

faktor = F_lead*F_lag;

G_o = G*F_lead*F_lag;
margin(G_o)
lab3robot(G,K,F_lead*F_lag,0,0,0,0,0,020712)
%For the control error to be reduced to zero we would need gamma to be
%zero, but doing this increases the gain of the low frequencies too much
%for us to fulfill the requirements
%%
step(F_lead*F_lag/(1+G_o)) %Check that the above indeed does satisfy u < umax
%%
F_leadlag = F_lead*F_lag;
F_prop = tf(4.55,1);

S_leadlag = 1/(1+G*F_leadlag);
S_prop = 1/(1+G*F_prop);
bodemag(S_leadlag, S_prop);
legend;
% We see that for the proportional controllers all frequencies below
% roughly 0.2 are attenuated followed by a peak at around 0.4. Frequencies
% between 0.2 and 5 are amplified. For the lead-lag compensator, all
% frequencies below 0.9 are attenuated and there's a peak at around 1.7.
% Frequencies between 0.9 and 20 are amplified

%%
T = F_leadlag*G/(1+F_leadlag*G);
G_delta1inv = 40/(s+10); %Does satisfy robustness criterion
G_delta2inv = 4*(s+0.01)/(s+10); %Doesn't satisfy the robustness criterion

bodemag(T,G_delta1inv, G_delta2inv);
legend;
%%
n = 1/20;
L_m = 2;
R_m = 21;
b = 1;
K_t = 38;
K_m = 0.5;
A = [0 n 0; 0 -b/J K_t/J; 0 -K_m/L_m -R_m/L_m];
B = [0 0 1/L_m]';
C = [1 0 0];
lab3robot(G, K, F_leadlag, A,B,C,1,1,020712);
%%
Controllable = [B A*B A^2*B];
Observable = [C; C*A; C*A^2];

contr_det = det(Controllable)
obs_det = det(Observable)
%Both determinants are non-zero which implies that the system is both
%observable and controllable since it has only one input signal and one
%output signal
%%
%syms L1 L2 L3
%L = [L1 L2 L3]
D = 0;
sys = ss(A,B,C,D);
P = [-2.5 + 1i,-2.5-1i, -2.5];  
K2 = place(A,B,P);
Acl = A - B*K2;
Ecl = eig(Acl);

syscl = ss(Acl,B,C,D);
gain = dcgain(syscl) ; 
syscl = ss(Acl,B/gain,C,D);

L0 = 1/gain; 
step(syscl)
lab3robot(G, K, F_leadlag, A,B,C,K2,L0,020712) % We get 1.49 instead of 1.69 as rise time 

%%First of all, we want poles in the left hand plane because this gives
%%stability. We dont want too high value on the imaginary part because this
%%results in oscilations.. We also want the real part to be negative since
%%this makes the amplitfude of oscilattipons decrease. Also, by choosing
%%the real part in the complex conjuagte pairs so that the angle formed is
%%less than 45 degrees we obtain a good damping factor. 
