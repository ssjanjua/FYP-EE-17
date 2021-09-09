%% 3 Phase Inverter With Space Vector PWM Technique
% 2017-FYP-14
clear all; close all;
%% User defined varibles
f=50;%fundamental frequency
m=0.85;%modulation index
fs=5000;%switching frequency
Vs=200;% input DC voltage
%% Va, Vb, Vc to Vd, Vq transformation
t=linspace(0,0.02,10000);
Va=sin(2*pi*f*t+pi/2);
Vb=sin(2*pi*f*t-pi/6);
Vc=sin(2*pi*f*t-5*pi/6);
figure;
plot(t,Va);
hold;
plot(t,Vb);
plot(t,Vc);
title('Reference 3 phase voltage');
xlabel('Time (second)');
ylabel('Voltage (volt)');
legend('Va','Vb','Vc');
Vd=(2*Va/3)-(Vb/3)-(Vc/3);
Vq=(1/sqrt(3))*(Vb-Vc);
figure;
plot(t,Vd);
hold;
plot(t,Vq);
title('Two phase system Vd and Vq');
xlabel('Time (second)');
ylabel('Voltage (volt)');
legend('Vd','Vq');
%% Vd, Vq to Vref, alpha transformation
Vref=abs(Vd+1j*Vq);
ang=angle(Vd+1j*Vq)*180/pi;
for n=1:10000
    if ang(n)<0
        ang(n)=ang(n)+360;
    end
end
ang(10000)=0;
%% Sector number calculation
n=floor(ang/60)+1;
%% calculating T0, T1, T2
Ts=1/fs;
T1=(m*Ts*(sin((n*pi/3)-(ang*pi/180))))/(sin(pi/3));
T2=(m*Ts*(sin((ang*pi/180)-((n-1)*pi/3))))/(sin(pi/3));
T0=Ts-T1-T2;
figure;
plot(t,T0);
hold;
plot(t,T1);
plot(t,T2);
title('T0 T1 and T2');
xlabel('Time (second)');
ylabel('Time (second)');
legend('T0','T1','T2');
%% Calculating duty cycle of high side switches corresponding to sector
D_S1=[]; D_S3=[]; D_S5=[];
for k=1:10000;
    if n(k)==1
        D_S1=[D_S1 , (T1(k)+T2(k)+T0(k)/2)/Ts];
        D_S3=[D_S3 , (T2(k)+T0(k)/2)/Ts];
        D_S5=[D_S5 , (T0(k)/2)/Ts];
    end
    if n(k)==2
        D_S1=[D_S1 , (T1(k)+T0(k)/2)/Ts];
        D_S3=[D_S3 , (T1(k)+T2(k)+T0(k)/2)/Ts];
        D_S5=[D_S5 , (T0(k)/2)/Ts];
    end
    if n(k)==3
        D_S1=[D_S1 , (T0(k)/2)/Ts];
        D_S3=[D_S3 , (T1(k)+T2(k)+T0(k)/2)/Ts];
        D_S5=[D_S5 , (T2(k)+T0(k)/2)/Ts];
    end
    if n(k)==4
        D_S1=[D_S1 , (T0(k)/2)/Ts];
        D_S3=[D_S3 , (T1(k)+T0(k)/2)/Ts];
        D_S5=[D_S5 , (T1(k)+T2(k)+T0(k)/2)/Ts];
    end
    if n(k)==5
        D_S1=[D_S1 , (T2(k)+T0(k)/2)/Ts];
        D_S3=[D_S3 , (T0(k)/2)/Ts];
        D_S5=[D_S5 , (T1(k)+T2(k)+T0(k)/2)/Ts];
    end
    if n(k)==6
        D_S1=[D_S1 , (T1(k)+T2(k)+T0(k)/2)/Ts];
        D_S3=[D_S3 , (T0(k)/2)/Ts];
        D_S5=[D_S5 , (T1(k)+T0(k)/2)/Ts];
    end
end
D_S1=(D_S1-0.5)*2;
D_S3=(D_S3-0.5)*2;
D_S5=(D_S5-0.5)*2;
figure;
plot(t,D_S1);
hold;
plot(t,D_S3);
plot(t,D_S5);
title('Duty cycle of S1 S3 S5');
xlabel('Time (second)');
ylabel('duty cycle value');
legend('D_{S1}','D_{S3}','D_{S5}');
%% Generating reference sawtooth waveform of 5kHz
ref_signal = sawtooth(2*pi*fs*t);
figure;
plot(t,ref_signal);
axis([0 0.005 -1 1]);
title('Reference sawtooth wave form of frequency 5kHz');
xlabel('Time (second)');
ylabel('Voltage (volt)');
%% Comparing duty cycles with the reference sawtooth waveform
PWM_S1=D_S1>ref_signal;
PWM_S3=D_S3>ref_signal;
PWM_S5=D_S5>ref_signal;
%% Taking not of gate signal of high side switches 
PWM_S2=~PWM_S5;
PWM_S4=~PWM_S1;
PWM_S6=~PWM_S3;
%plotting gate signals for S1-S6
figure;
subplot(6,1,1);
plot(t,PWM_S1);
ylim([-0.2 1.2]);
title('Space vector PWM signal for all switches');
ylabel('Vg 1');
subplot(6,1,2);
plot(t,PWM_S2);
ylim([-0.2 1.2]);
ylabel('Vg 2');
subplot(6,1,3);
plot(t,PWM_S3);
ylim([-0.2 1.2]);
ylabel('Vg 3');
subplot(6,1,4);
plot(t,PWM_S4);
ylim([-0.2 1.2]);
ylabel('Vg 4');
subplot(6,1,5);
plot(t,PWM_S5);
ylim([-0.2 1.2]);
ylabel('Vg 5');
subplot(6,1,6);
plot(t,PWM_S6);
ylim([-0.2 1.2]);
xlabel('Time (Second)');
ylabel('Vg 6');
%% 3 phase inverter implementation
Van=[]; Vbn=[]; Vcn=[];
Vab=[]; Vbc=[]; Vca=[];
for k=1:10000
    % H.S switching code 101
    if (PWM_S1(k)==1 && PWM_S3(k)==0 && PWM_S5(k)==1)
        Van=[Van , Vs/3];
        Vbn=[Vbn , -2*Vs/3];
        Vcn=[Vcn , Vs/3];
        Vab=[Vab , Vs];
        Vbc=[Vbc , -Vs];
        Vca=[Vca , 0];
    end
    % H.S switching code 100
    if (PWM_S1(k)==1 && PWM_S3(k)==0 && PWM_S5(k)==0)
        Van=[Van , 2*Vs/3];
        Vbn=[Vbn , -Vs/3];
        Vcn=[Vcn , -Vs/3];
        Vab=[Vab , Vs];
        Vbc=[Vbc , 0];
        Vca=[Vca , -Vs];
    end
    % H.S switching code 110
    if (PWM_S1(k)==1 && PWM_S3(k)==1 && PWM_S5(k)==0)
        Van=[Van , Vs/3];
        Vbn=[Vbn , Vs/3];
        Vcn=[Vcn , -2*Vs/3];
        Vab=[Vab , 0];
        Vbc=[Vbc , Vs];
        Vca=[Vca , -Vs];
    end
    % H.S switching code 010
    if (PWM_S1(k)==0 && PWM_S3(k)==1 && PWM_S5(k)==0)
        Van=[Van , -Vs/3];
        Vbn=[Vbn , 2*Vs/3];
        Vcn=[Vcn , -Vs/3];
        Vab=[Vab , -Vs];
        Vbc=[Vbc , Vs];
        Vca=[Vca , 0];
    end
    % H.S switching code 011
    if (PWM_S1(k)==0 && PWM_S3(k)==1 && PWM_S5(k)==1)
        Van=[Van , -2*Vs/3];
        Vbn=[Vbn , Vs/3];
        Vcn=[Vcn , Vs/3];
        Vab=[Vab , -Vs];
        Vbc=[Vbc , 0];
        Vca=[Vca , Vs];
    end
    % H.S switching code 001
    if (PWM_S1(k)==0 && PWM_S3(k)==0 && PWM_S5(k)==1)
        Van=[Van , -Vs/3];
        Vbn=[Vbn , -Vs/3];
        Vcn=[Vcn , 2*Vs/3];
        Vab=[Vab , 0];
        Vbc=[Vbc , -Vs];
        Vca=[Vca , Vs];
    end
    % H.S switching code 000
    if (PWM_S1(k)==0 && PWM_S3(k)==0 && PWM_S5(k)==0)
        Van=[Van , 0];
        Vbn=[Vbn , 0];
        Vcn=[Vcn , 0];
        Vab=[Vab , 0];
        Vbc=[Vbc , 0];
        Vca=[Vca , 0];
    end
    % H.S switching code 111
    if (PWM_S1(k)==1 && PWM_S3(k)==1 && PWM_S5(k)==1)
        Van=[Van , 0];
        Vbn=[Vbn , 0];
        Vcn=[Vcn , 0];
        Vab=[Vab , 0];
        Vbc=[Vbc , 0];
        Vca=[Vca , 0];
    end
end
%plotting phase voltages
figure;
subplot(3,1,1);
plot(Van);
title('Output phase voltages');
ylabel('Van');
subplot(3,1,2);
plot(Vbn);
ylabel('Vbn');
subplot(3,1,3);
plot(Vcn);
xlabel('Sampling points');
ylabel('Vcn');
%plotting line voltages
figure;
subplot(3,1,1);
plot(Vab);
ylim([-Vs-20 Vs+20]);
title('Output line voltages');
ylabel('Vab');
subplot(3,1,2);
plot(Vbc);
ylim([-Vs-20 Vs+20]);
ylabel('Vbc');
subplot(3,1,3);
plot(Vca);
ylim([-Vs-20 Vs+20]);
xlabel('Sampling points');
ylabel('Vca');
