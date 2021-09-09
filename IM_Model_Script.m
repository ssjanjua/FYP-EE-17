%% Induction Moter Modeling in MATLAB
% 2017-FYP-14
%% Defining parameters of a induction machine.

clear all; clc;
Vs=400; %Stator voltage
Rs=0.12; %Stator resistance
Rr=0.02; %Rotor resistance
Lls=0.0001; %Stator inductance
Llr=0.0001; %Rotor inductance
Lm=0.01;
P=2; %Number of Poles
Ws=2*pi*50; %synchronous speed
s=0.02; % slip
Wr=(1-s)*Ws; %Rotor speed
time=2; %Simulation speed
%% 3-phase voltage source input of induction motor

t=0:0.0002:time;
Vamp=Vs*sqrt(2)/sqrt(3);
Vas=Vamp*sin(Ws*t);
Vbs=Vamp*sin(Ws*t-2*pi/3);
Vcs=Vamp*sin(Ws*t+2*pi/3);
%% Plotting 3phase input to stator

figure();
subplot(3,1,1);
plot(t,Vas);
title('Va of stator');
ylabel('Voltage (volt)');
subplot(3,1,2);
plot(t,Vbs);
title('Vb of stator');
ylabel('Voltage (volt)');
subplot(3,1,3);
plot(t,Vas,t,Vbs,t,Vcs);
plot(t,Vcs);
title('Vc of stator');
ylabel('Voltage (volt)');
%% abc to alpha beta transformation

Valphas=sqrt(2/3)*((Vas)-(Vbs/2)-(Vcs/2));
Vbetas=sqrt(2/3)*((sqrt(3)*Vbs/2)-((sqrt(3)*Vcs/2)));
%% Plotting Valpha and Vbeta for stator

figure();
subplot(2,1,1);
plot(t,Valphas);
grid;
title('Valpha of stator');
ylabel('Voltage (volt)');
subplot(2,1,2);
plot(t,Vbetas);
grid;
title('Vbeta of stator');
xlabel('Time (sec)');
ylabel('Voltage (volt)');
%% Calculation of rotating anlge theta

theta=angle(Valphas+1j*Vbetas);%rotating angle theta
%% alpha beta to d q transfromation
Vds=round(Valphas.*cos(theta)+Vbetas.*sin(theta),1);
Vqs=round(-Valphas.*sin(theta)+Vbetas.*cos(theta),1);
%% Plotting Vd and Vq for stator

figure();
subplot(2,1,1)
plot(t,Vds);
grid;
title('Vd of stator');
ylabel('Voltage (volt)');
subplot(2,1,2)
plot(t,Vqs);
grid;
title('Vq of stator');
xlabel('Time (sec)');
ylabel('Voltage (volt)');
%% Calculating Ids, Iqs, Idr, Iqr using ODE45 solver

tspan=0:0.0002:2;%time span
Vdsi=@(t)interp1(tspan,Vds,t);
Vqsi=@(t)interp1(tspan,Vqs,t);
IC=[0 0 0 0];%initial conditions
%using ODE45 matlab solver to solve differential equations
[tsol,ysol] = ode45(@(t,s) g(t,s,Rs,Rr,Lls,Llr,Lm,Ws,Wr,Vdsi,Vqsi),tspan,IC);
Ids=ysol(:,1)';%Id for stator
Iqs=ysol(:,2)';%Iq for stator
Idr=ysol(:,3)';%Id for rotor
Iqr=ysol(:,4)';%Iq for rotor
%% Plotting Id and Iq of stator

figure();
subplot(2,1,1);
plot(t,Ids);
grid;
title('Id of stator');
ylabel('Current (Amp)');
subplot(2,1,2);
plot(t,Iqs);
grid;
title('Iq of stator');
xlabel('Time (sec)');
ylabel('Current (Amp)');
%% Plotting Id and Iq of rotor

figure();
subplot(2,1,1);
plot(t,Idr);
grid;
title('Id of rotor');
ylabel('Current (Amp)');
subplot(2,1,2);
plot(t,Iqr);
grid;
title('Iq for rotor');
xlabel('Time (sec)');
ylabel('Current (Amp)');
%% d q to alpha beta transformation

%stator current transformation
Ialphas=Ids.*cos(theta)-Iqs.*sin(theta);
Ibetas=Ids.*sin(theta)+Iqs.*cos(theta);
%rotor current transformation
Ialphar=Idr.*cos(theta)-Iqr.*sin(theta);
Ibetar=Idr.*sin(theta)+Iqr.*cos(theta);
%% Plotting Ialpha and Ibeta of stator

figure();
subplot(2,1,1);
plot(t,Ialphas);
grid;
title('Ialpha of stator');
ylabel('Current (A)');
subplot(2,1,2);
plot(t,Ibetas);
grid;
title('Ibeta of stator');
xlabel('Time (sec)');
ylabel('Current (A)');
%% Plotting Ialpha and Ibeta of rotor

figure();
subplot(2,1,1);
plot(t,Ialphar);
grid;
title('Ialpha of rotor');
ylabel('Current (A)');
subplot(2,1,2);
plot(t,Ibetar);
grid;
title('Ibeta of rotor');
xlabel('Time (sec)');
ylabel('Current (A)');
%% alpha beta to a b c transformation

%stator current transformation
Ias=sqrt(2/3)*Ialphas;
Ibs=sqrt(2/3)*(-Ialphas/2+Ibetas*sqrt(3)/2);
Ics=sqrt(2/3)*(-Ialphas/2-Ibetas*sqrt(3)/2);
%rotor current transformation
Iar=sqrt(2/3)*Ialphar;
Ibr=sqrt(2/3)*(-Ialphar/2+Ibetar*sqrt(3)/2);
Icr=sqrt(2/3)*(-Ialphar/2-Ibetar*sqrt(3)/2);
%% Plotting Ia Ib and Ic of stator

figure();
subplot(3,1,1);
plot(t,Ias);
grid;
title('Ia of stator');
ylabel('Current (A)');
subplot(3,1,2);
plot(t,Ibs);
title('Ib of stator');
ylabel('Current (A)');
subplot(3,1,3);
plot(t,Ics);
grid;
title('Ic of stator');
xlabel('Time (sec)');
ylabel('Current (A)');
%% plotting Ia Ib and Ic of rotor

figure();
subplot(3,1,1);
plot(t,Iar);
grid;
title('Ia of rotor');
ylabel('Current (A)');
subplot(3,1,2);
plot(t,Ibr);
grid;
title('Ib of rotor');
ylabel('Current (A)');
subplot(3,1,3);
plot(t,Icr);
grid;
title('Ic of rotor');
xlabel('Time (sec)');
ylabel('Current (A)');
%% Calculating torque (Te)

Te=(3*P*Lm/4)*(Iqs.*Idr-Ids.*Iqr);
%% Plotting electromechanical torque of induction motor

figure();
plot(t,Te);
grid;
title('Torque of the motor');
xlabel('Time (sec)');
ylabel('Torque (N-m)');
%% Function for ODE45 solver

%equation written in terms of state varivables
function sdot = g(t,s,Rs,Rr,Lls,Llr,Lm,Ws,Wr,Vdsi,Vqsi)
sdot(1,1) = [(Llr+Lm)/((Lls+Llr)*Lm+Lls*Llr)]*[Vdsi(t)-Rs*s(1)+Ws*[(Lls+Lm)*s(2)+Lm*s(4)]] ...
         -[Lm/((Lls+Llr)*Lm+Lls*Llr)]*[-Rr*s(3)+(Ws-Wr)*[(Llr+Lm)*s(4)+Lm*s(2)]];
         
sdot(2,1) =  [(Llr+Lm)/((Lls+Llr)*Lm+Lls*Llr)]*[Vqsi(t)-Rs*s(2)-Ws*[(Lls+Lm)*s(1)+Lm*s(3)]] ...
         -[Lm/((Lls+Llr)*Lm+Lls*Llr)]*[-Rr*s(4)-(Ws-Wr)*[(Llr+Lm)*s(3)+Lm*s(1)]];
         
sdot(3,1) = [(Lls+Lm)/((Lls+Llr)*Lm+Lls*Llr)]*[-Rr*s(3)+(Ws-Wr)*[(Llr+Lm)*s(4)+Lm*s(2)]] ...
         -[Lm/((Lls+Llr)*Lm+Lls*Llr)]*[Vdsi(t)-Rs*s(1)+Ws*[(Lls+Lm)*s(2)+Lm*s(4)]];
         
sdot(4,1) = [(Lls+Lm)/((Lls+Llr)*Lm+Lls*Llr)]*[-Rr*s(4)-(Ws-Wr)*[(Llr+Lm)*s(3)+Lm*s(1)]] ...
         -[Lm/((Lls+Llr)*Lm+Lls*Llr)]*[Vqsi(t)-Rs*s(2)-Ws*[(Lls+Lm)*s(1)+Lm*s(3)]];

end
