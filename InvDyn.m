close all
%Carregando data
%[FileName,FilePath] = uigetfile([ filesep '*.mat'],'Select the file: dados.mat', pwd);

%load(['C:\Adriano\alunos_Doutorado\Guido\Matlab\BaseDadosEMG\Subject1_Trial1\dados.mat']);
load(['C:\Adriano\alunos_Doutorado\Guido\Matlab\BaseDadosEMG\dados.mat']);


dt = 0.005;
time_emg_proc = [0:dt:(length(emg_proc)-1)*dt];
time_exo = [0:dt:(length(theta_exo)-1)*dt];
time_OS = [0:dt:(length(torque_OS)-1)*dt];

theta_exo = theta_exo*pi/180;

m = 5;
g = 9.81;
l = 0.25;
J = 0.05;
C = 0.1;

G = m*g*l*cos(theta_exo);

omega_exo = diff(theta_exo)/dt;
omega_exo(end+1) = omega_exo(end);
N = 4;
Wn = 0.1;
[B,A] = butter(N,Wn);
omega_exo_f = filter(B,A,omega_exo);



alfa_exo = diff(omega_exo)/dt;
alfa_exo(end+1) = alfa_exo(end);
N = 4;
Wn = 0.1;
[B,A] = butter(N,Wn);
alfa_exo_f = filter(B,A,alfa_exo);



torque_InvDyn = J*alfa_exo_f + C*omega_exo_f + G - torque_exo;

torque_InvDyn2 = G - torque_exo;

time_InvDyn = [0:dt:(length(torque_InvDyn)-1)*dt];

figure
plot(time_exo,torque_exo);
grid
hold on
plot(time_OS,torque_OS,'r');
plot(time_InvDyn,torque_InvDyn,'g');
plot(time_InvDyn,torque_InvDyn2,'m');
legend('\tau_{robot}','\tau_{OS}','\tau_{InvDyn}','\tau_{InvDyn2}')
xlabel('TIME (s)');
ylabel('TORQUE (Nm)');


figure
plot(time_exo,theta_exo);
grid
hold on
plot(time_exo,omega_exo,'r');
plot(time_exo,omega_exo_f,'k');

figure
plot(time_exo,alfa_exo,'g');
hold on
plot(time_exo,alfa_exo_f,'k');
