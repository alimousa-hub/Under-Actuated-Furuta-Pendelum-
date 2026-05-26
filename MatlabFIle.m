%% 1) Parameters (given)
mp = 0.024;        % pendulum mass (kg)
g  = 9.81;         % gravity (m/s^2)
L  = 0.129;        % pendulum total length (m)
s  = L/2;          % COM distance from pivot (m)
r  = 0.085;        % arm length (m)

Jr = 0.000228792;  % arm inertia about motor axis (kg*m^2)
Jp = 0.000033282;  % pendulum inertia about COM (kg*m^2)

br = 1e-3;         % arm viscous damping (N*m*s/rad)
bp = 5e-5;         % pendulum viscous damping (N*m*s/rad)

Kt = 0.0422;       % torque constant (N*m/A)

DEGREE=15;
ALPHA0=(DEGREE*pi)/180;
%% 2) Derived constants
c  = Jp + mp*s^2;     % pendulum inertia about pivot (kg*m^2)
a   = Jr + mp*r^2;
b=1/2*mp*r*L
d=mp*g*L/2
delta=a*c-b^2
         

%% 3) State-space matrices (linearized about alpha = 0, alpha_dot=0, theta_dot=0)
A = [ 0          1           0      0;
      a*d/delta   -a*bp/delta      0      0;
      0          0           0      1;
     -b*d/delta   b*bp/delta     0     -c*br/delta];

  B = [ 0;
      -b*Kt/delta;
       0;
      c*Kt/delta];
      

C = [1 0 0 0;
    0 0 1 0];   % output y = alpha
D = [0;0];
X0=[ALPHA0;0;0;0];
C1=[1 0 0 0;
    0 1 0 0
    0 0 1 0
    0 0 0 1]
D1=[0;0;0;0]
desired_overshoot = 15;
settling_time = 2;

OS = desired_overshoot/100;
zeta = -log(OS) / sqrt(pi^2 + log(OS)^2);

wn = 4 / (zeta * settling_time);

wd = wn * sqrt(1 - zeta^2);   

p1 = -zeta*wn + 1j*wd;
p2 = -zeta*wn - 1j*wd;
p3 = -3*wn;
p4 = -5*wn;

desired_poles = [p1 p2 p3 p4];

% Use pole placement to find the state feedback matrix K
k = place(A, B, desired_poles);
sys = ss(A,B,C,D);
Ctheta=[0 0 1 0]
N = -inv(Ctheta * ((A - B*k)\B));

%% 4) Display results
disp('A = '); disp(A);
disp('B = '); disp(B);
disp('C = '); disp(C);
disp('D = '); disp(D);

disp('Open-loop eigenvalues of A:');
disp(eig(A));

%% 5) Open-loop simulation (zero input, initial alpha disturbance)
t  = 0:0.001:5;                 % time vector (s)
u  = zeros(size(t));            % Vm = 0
x0 = [5*pi/180; 0; 0; 0];       % 5 deg initial alpha, rest zero

[y,t_out,x] = lsim(sys,u,t,x0);

figure;
plot(t_out,y,'LineWidth',1.5);
grid on;
xlabel('Time (s)');
ylabel('\alpha (rad)');
title('Open-loop response (Vm = 0), initial \alpha = 5 deg');


%% 6) (Optional) also plot theta if you want
% To output both alpha and theta, change C to:
% C2 = [1 0 0 0;   % alpha
%       0 0 1 0];  % theta
% sys2 = ss(A,B,C2,[0;0]);
% [y2,t2,x2] = lsim(sys2,u,t,x0);
% figure; plot(t2,y2); grid on;
% legend('\alpha (rad)','\theta (rad)'); xlabel('Time (s)');
