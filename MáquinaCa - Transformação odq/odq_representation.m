clear;close all;clc;

% Parâmetros
Rs = 0;
R1s = Rs;
R2s = Rs;
R3s = Rs;

Rr = 0;
R1r = Rr;
R2r = Rr;
R3r = Rr;

deltag = 0;
thetar = 0;

i1s = 0;
i2s = 0;
i3s = 0;

i1r = 0;
i2r = 0;
i3r = 0;

I123s = [i1s;i2s;i3s];
I123r = [i1r;i2r;i3r];

Ls = 2;
Ms = 1;
Msr = 0.5;

Lr = 2;
Mr = 1;
Mrs = Msr;

Lss = [Ls Ms Ms;
       Ms Ls Ms;
       Ms Ms Ls];
   
Lrr = [Lr Mr Mr;
       Mr Lr Mr;
       Mr Mr Lr];

Lsr = Msr*[cos(thetar), cos(thetar+2*pi/3), cos(thetar+4*pi/3);
           cos(thetar+4*pi/3), cos(thetar), cos(thetar+2*pi/3);
           cos(thetar+2*pi/3), cos(thetar+4*pi/3), cos(thetar)]; 
       
Lrs = Mrs*[cos(thetar), cos(thetar+4*pi/3), cos(thetar+2*pi/3);
           cos(thetar+2*pi/3), cos(thetar), cos(thetar+4*pi/3);
           cos(thetar+4*pi/3), cos(thetar+2*pi/3), cos(thetar)]; 
       
% Matrizes de transformação odq
Ps = sqrt(2/3)*[1/sqrt(2),cos(deltag),-sin(deltag);...
                1/sqrt(2),cos(deltag-2*pi/3),-sin(deltag-2*pi/3);...
                1/sqrt(2),cos(deltag-4*pi/3),-sin(deltag-4*pi/3)];

Pr = sqrt(2/3)*[1/sqrt(2),cos(deltag-thetar),-sin(deltag-thetar);...
                1/sqrt(2),cos(deltag-2*pi/3-thetar),-sin(deltag-2*pi/3-thetar);...
                1/sqrt(2),cos(deltag-4*pi/3-thetar),-sin(deltag-4*pi/3-thetar)];

            
Isodq = inv(Ps)*I123s;
Irodq = inv(Pr)*I123r;

Lssodq = inv(Ps)*Lss*Ps;  
Lsrodq = inv(Ps)*Lsr*Pr;
Lrsodq = inv(Pr)*Lrs*Ps;
Lrrodq = inv(Pr)*Lrr*Pr;

% Equações de fluxo
% Forma normal
lambda123s = Lss*I123s + Lsr*I123r;
lambda123r = Lrs*I123s + Lrr*I123r;

% Forma odq
lambdaodqs = Lssodq*Isodq + Lsrodq*Irodq;
lambdaodqr = Lrsodq*Isodq + Lrrodq*Irodq;

% Equações de tensão
%V = VR + VL + VM
% Forma normal
v123s = Rs*I123s;%+Lss*d(I123s)/dt + (Lsr*d(I123r)/dt + wr*I123r*d(Lsr)/dtheta 
v123r = Rr*I123r;%+Lrr*d(I123r)/dt + (Lrs*d(I123s)/dt + wr*I123s*d(Lrs)/dtheta

% Forma odq

% Equações de conjugado
%ce = dW/dtheta; -> Torque

