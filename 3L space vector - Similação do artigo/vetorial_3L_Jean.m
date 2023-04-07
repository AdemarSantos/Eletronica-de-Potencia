%-----------------------------------------------------------------------------------------------------------
%-----------------------------------------------------------------------------------------------------------
%                                       Parametros de Simulação
%-----------------------------------------------------------------------------------------------------------
%-----------------------------------------------------------------------------------------------------------

clear all;
close all;
clc;
          
t = 0;                          % tempo inicial de simulacao

t_s = 0;                        % tempo inicial do calculo do valor médio                       
f_s = 10e3;                     % frequência de calculo do valor médio
h_s = 1/f_s;                    % período de calculo do valor médio

h = 1e-6;                       % passo de calculo 

t_f = 4/60;                     % tempo total de simulaçao
t_armazena = 0;

h_save0 = 0.00; 					% tempo inicial de gravaçao de vetores de dados de saida 				
h_save = h_save0;
npt = (t_f-h_save0)/(h);  		% numero de pontos dos vetores de saida
hsave = h;                      % periodo de amostragem dos vetores de dados de saida 

if (hsave < h)
    hsave = h;                  % saturacao do periodo de amostragem de saida se <= passo de calcvo
end

%--------------------------------------------------------------------------    
%-------------------------------Parametros---------------------------------
%--------------------------------------------------------------------------
f = 60;
f = 60;
w = 2*pi*f;

Eg_rms = 110.0*0.7;                 % Valor eficaz da tensão da rede
Vl_ref_rms = 110.0;             % Valor eficaz da referencia da tensão de saída

fg = 60;                        % Frequência da rede
fl = fg;                        % Frequência da tensão de saída

Pl = 1000;                       % Potência Ativa da Carga
FPl = 1;                     % Fator de Potência da Carga
Sl = Pl/FPl*exp(acos(FPl)*1i);   % Potência Aparente da Carga

Zl = conj((Vl_ref_rms^2)/(Sl));	% Impedância da Carga
Rl = real(Zl)  ;                % Resistência da Carga
Xl = imag(Zl);                  % Reatância da Carga
Ll = Xl/(2*pi*fl);

Zb = (Eg_rms^2)/Pl;
Xg = 0.2*Zb;
Lg = Xg/(2*pi*fg);
Rg = 0.1*Xg;

% Cálculo do valor eficaz da corrente drenada da rede
Ig = roots([Rg -Eg_rms Pl]);

Ig = min(Ig);
faseg = (180/pi)*atan((-Ig*Xg)/(Eg_rms - Ig*Rg));
Vgref = (-Ig*Xg)/(sin(faseg*pi/180));

m = 0.7043;
Vcc = (max([sqrt(2)*Vgref sqrt(2)*Vl_ref_rms]))/m;
Vcc=221;

C1 = 2200e-6;
C2 = 2200e-6;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

qg = 1;
ql = 1;
qs = 1;
qs1 = 1;
qs2 = 1;

contg = 0;
contl = 0;
conts = 0;
conts1 = 0;
conts2 = 0;

vpg_ant = 0;
vpl_ant = 0;
vps_ant = 0;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

run('aux_3L');

x = 0;
t = 0;
t_s = 0;
t_c = 0;

ig = 0;
il = 0;


vg_soma = 0;
vl_soma = 0;
vg = 0;
vl = 0;

modo_carga_vc1 = 0;
modo_carga_vc2 = 0;

cont1 = 0;
cont2 = 0;
cont3 = 0;
cont4 = 0;
cont5 = 0;

%%

while t < t_f					     

    t = t + h;   						 % tempo de simulaçao
    
    
    eg = sqrt(2)*Eg_rms*cos(2*pi*fg*t);
    vg_ref = sqrt(2)*Vgref*cos(2*pi*fg*t + faseg*pi/180);
    vl_ref = sqrt(2)*Vl_ref_rms*cos(2*pi*fl*t);
        
    ig = sqrt(2)*Ig*cos(2*pi*fg*t);
    il = (sqrt(2)*(Vl_ref_rms/abs(Zl)))*cos(2*pi*fl*t - acos(FPl) + (faseg*pi)/180);
    
%     vg_ref_pu = vg_ref/(Vcc);
%     vl_ref_pu = vl_ref/(Vcc);

    vg_soma = vg_soma + vg*h;
    vl_soma = vl_soma + vl*h;      
  
    if(t >= t_s)
        
        if (vg_ref < 0)
            p = 1;
            vg_ref_pu = -vg_ref/Vcc;
            vl_ref_pu = -vl_ref/Vcc;
        else
            p = 0;
            vg_ref_pu = vg_ref/Vcc;
            vl_ref_pu = vl_ref/Vcc;
        end
        
        vg_med = vg_soma/(t-t_s+h_s);
        vg_soma = 0;
        vl_med = vl_soma/(t-t_s+h_s);
        vl_soma = 0;
        
        t_s = t_s + h_s;
        

    
%--------------------------------------------------------------------------
%                       Detecção da Região
%--------------------------------------------------------------------------
        
        
        if(vg_ref_pu >= vl_ref_pu)
	
            if(vl_ref_pu >= 0.)
                k = 1.;
            else
                if(vg_ref_pu >= 0.)
                    k = 6.;
                else
                    k = 5.;		
                end
            end
        else
            if(vl_ref_pu >= 0.)

                if(vg_ref_pu >= 0.) 
                    k = 2.;
                else
                    k = 3.;
                end

            else
                k = 4.;
            end
        end
	


%--------------------------------------------------------------------------
%                       SV-PWM
%--------------------------------------------------------------------------


        if(k == 1)
            v1 = v100; %V1
            v2 = v110; %V2
            v3 = v111; %V7
            q1 = q100;
            q2 = q110;
            q3 = q111;

        elseif(k == 2)
            v1 = v010; %V3
            v2 = v110; %V2
            v3 = v111; %V7
            q1 = q010;
            q2 = q110;
            q3 = q111;
        elseif(k == 3)
            v1 = v010; %V3
            v2 = v011; %V4
            v3 = v111; %V7
            q1 = q010;
            q2 = q011;
            q3 = q111;
        elseif(k == 4)
            v1 = v001; %V5
            v2 = v000; %V0
            v3 = v011; %V4
            q1 = q001;
            q2 = q000;
            q3 = q011;
        elseif(k == 5)
            v1 = v001; %V5
            v2 = v000; %V0
            v3 = v101; %V6
            q1 = q001;
            q2 = q000;
            q3 = q101;
        else
            v1 = v100; %V1
            v2 = v000; %V0
            v3 = v101; %V6
            q1 = q100;
            q2 = q000;
            q3 = q101;

        end
        


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    a = v1(1)-v3(1);
    c = v1(2)-v3(2);
    
    b = v2(1)-v3(1);
    d = v2(2)-v3(2);
    
    div = a*d-b*c; 

    t1 = ( (vg_ref_pu-v3(1))*d - (vl_ref_pu-v3(2))*b)/div*h_s;
    t2 = (-(vg_ref_pu-v3(1))*c + (vl_ref_pu-v3(2))*a)/div*h_s;
    t3 = (h_s - t1 - t2);
    
%     t_c = 0;
    end
%     t_c = t_c + h;
    if(t_c < t1/2)
        q = q1;
    elseif (t_c < t2/2 + t1/2)
        q = q2;
        cont2 = cont2+1;
    elseif (t_c < t2/2 + t1/2 + t3)
        q = q3;
        cont3 = cont3+1;
    elseif (t_c < t2 + t1/2 + t3)
        q = q2;
        cont4 = cont4+1;
    else
        q = q1;
        cont5 = cont5+1;
    end

    t_c = mod(t,h_s);
    cont1 = cont1 + 1;
    t_cc(cont1) = h_s;
    
    qg_ant = qg;
    ql_ant = ql;
    qs_ant = qs;

    if p==1
        q = not(q);
    end

    qg = q(1);
    ql = q(2);
    qs = q(3);

    vg0 = (2*qg-1)*Vcc/2;
    vs0 = (2*qs-1)*Vcc/2;
    vl0 = (2*ql-1)*Vcc/2;

    vg = vg0 - vs0;
    vl = vl0 - vs0;

   

%----------------------------------Fmed----------------------------------%
    ts = t_f;
    fsw_qg = contg/(2*ts);
    fsw_ql = contl/(2*ts);
    fsw_qs = conts/(2*ts);

%----------------------------------Perdas----------------------------------%
    vg_ant = vpg_ant;
    vs_ant = vps_ant;
    vl_ant = vpl_ant;

    is = il - ig;

%     [Pcondg, vpg_ant, Pchavg, Ptotg] = perdas(vg0, ig, vg_ant, Vcc, h);
%     [Pconds, vps_ant, Pchavs, Ptots] = perdas(vs0, is, vs_ant, Vcc, h);
%     [Pcondl, vpl_ant, Pchavl, Ptotl] = perdas(vl0, il, vl_ant, Vcc, h);

%     P_filtro = (vg-eg)*ig;
%     P_in_conv = vg*ig;
%     P_in = eg*ig;
%     P_out = vl*il;
%     P_out_conv = vl*il;


%%
    if h_save <= t                        % inicio gravaçao de vetores de dados de saida
        h_save = h_save + hsave;
        x = x + 1;
%         perdascondg(x) = Pcondg;
%         perdaschavg(x) = Pchavg;
%         perdastotg(x) = Ptotg;
%         perdasconds(x) = Pconds;
%         perdaschavs(x) = Pchavs;
%         perdastots(x) = Ptots;
%         perdascondl(x) = Pcondl;
%         perdaschavl(x) = Pchavl;
%         perdastotl(x) = Ptotl;
%         potencia_in(x) = P_in;
%         potencia_out(x) = P_out;
%         potencia_in_conv(x) = P_in_conv;
%         potencia_out_conv(x) = P_out_conv;
%         potencia_filtro(x) = P_filtro;
        T(x) = t;
        EG(x) = eg;
        IG(x) = ig;
        IL(x) = il;
        VG_REF(x) = vg_ref;
        VL_REF(x) = vl_ref;
        VG_REF_PU(x) = vg_ref_pu;
        VL_REF_PU(x) = vl_ref_pu;
        VG_MED(x) = vg_med;
        VL_MED(x) = vl_med;
        K(x) = k;
        VG(x) = vg;
        VL(x) = vl;
        QS(x) = qs;
        P(x) = p;
    end						            

end
%-------------Perdas---------------%

% Braço g
% Perdascondg = mean(perdascondg);
% Perdaschavg = mean(perdaschavg);
% Perdastotg = mean(perdastotg);
% % Braço s
% Perdasconds = mean(perdasconds);
% Perdaschavs = mean(perdaschavs);
% Perdastots = mean(perdastots);
% % Braço l
% Perdascondl = mean(perdascondl);
% Perdaschavl = mean(perdaschavl);
% Perdastotl = mean(perdastotl);
% 
% Perdascond_conv = Perdascondg + Perdasconds + Perdascondl
% Perdaschav_conv = Perdaschavg + Perdaschavs + Perdaschavl
% Perdas_conv = Perdastotg + Perdastots + Perdastotl

% ---------------Fmed---------------%
% f_med = (fsw_qg + fsw_ql + fsw_qs)/3

% --------WTHD------------%
%wthd_vg = wthdf(VG, 1/h, f)
%wthd_vl = wthdf(VL, 1/h, f)
%  
% figure(1),plot(T,IL,T,IG),grid on
% figure(2),plot(T,VG_MED),grid on

% figure(2),plot(T,VL,T,VL_REF,T,VL_MED), grid on

figure(1)
subplot(2,1,1)
plot(T,VG,T,VG_MED,T,VG_REF)
subplot(2,1,2)
plot(T,VL,T,VL_MED,T,VL_REF)


% figure
% plot(VG_REF,VL_REF)





























% figure(1)
% plot(T,VG_REF)
% f = figure(2);
% f.Units = 'centimeters';
% f.Position = [24 5 9 8];
% f.Color = [1 1 1];
% s = subplot(1,2,1);
%     p = plot(T,VG/Vcc,T,VG_REF/Vcc);
%     s.XLabel.String = '$t$ (s)';
%     s.XLabel.FontSize = 12;
%     s.XLabel.Interpreter = 'latex';
%     s.YLabel.String = '$v_{g}$ (V)';
%     s.YLabel.FontSize = 12;
%     s.YLabel.Interpreter = 'latex';
% 
%     s.TickLabelInterpreter = 'latex';
%     s.XLim = [0. 1/60];
%     s.XTick = [0 0.5/60 1/60];
%     s.YLim = [-Vcc/Vcc Vcc/Vcc];
%     
%  k = subplot(1,2,2);
%     p = plot(T,VL/Vcc,T,VL_REF/Vcc);
%     k.XLabel.String = '$t$ (s)';
%     k.XLabel.FontSize = 12;
%     k.XLabel.Interpreter = 'latex';
%     k.YLabel.String = '$v_{l}$ (V)';
%     k.YLabel.FontSize = 12;
%     k.YLabel.Interpreter = 'latex';
% 
%     k.TickLabelInterpreter = 'latex';
%     k.XLim = [0. 1/60];
%     k.XTick = [0 0.5/60 1/60];
%     k.YLim = [-Vcc/Vcc Vcc/Vcc]; 

