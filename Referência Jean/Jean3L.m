%% Código 3Ls - Versão Revista IAS
%% Jean T. Cardoso
%% Análises...

clear;
close all;
clc;

%% Inicializações 1
i = 0;
p = 0;
j = 0;

%% Parâmetros de simulação
f_s    = 10e3;                     
h_s    = 1/f_s;                    
fpwm   = f_s;
hpwm   = h_s;
h      = 1e-6;                       
delt   = h;
t_f    = 1+1/60;                  
tsave0 = 0; 					
tsave  = tsave0;
npt    = (t_f-tsave0)/(h);  		
hsave  = h;                      

if (hsave < h)
    hsave = h;                  
end

%% Constantes de entrada
f         = 60;
fg        = 60;                        
fl        = fg;                        
C         = 8.8e-3;
w         = 2*pi*f;
%%
% for SwSg = 1:0.01:1.8
SwSg      = 1;
Eg_rms    = 110*SwSg;                 
Vlref     = 110;             
Pl        = 1000;                       
atrasoPWM = 0.5*2.*pi*f/f_s;

% for FPl = 0.02:0.02:1
% for alpha = (-25*pi/180):(pi/180):(25*pi/180)

    i = i + 1;
    %% Inicializações 2
    mdc        = 0;
    x          = 0;
    b          = 0;
    
    tsave      = 0;
    tsavex     = 0;
    
    ig         = 0;
    il         = 0;
    is         = 0;

    vsh_soma   = 0;
    vl_soma    = 0;
    vse_soma   = 0;
    vcap       = 50;
    vcapx      = 50;
    vsh        = 0;
    vse        = 0;
    vl         = 0;
    t          = 0;                          
    t_s        = 0;
    t_armazena = 0;

    %% Parâmetros da carga
    FPl   = 0.7;
    Sl    = Pl/FPl*exp(acos(FPl)*1i);
    Slabs = abs(Sl); 
    Zl    = conj((Vlref^2)/(Sl));	
    Rl    = real(Zl);                  
    Xl    = imag(Zl);                  
    Ll    = Xl/(2*pi*fl);
    Il    = Vlref/abs(Zl);
    

%     Rl  = 1; 
%     Ll  = 10e-3; 
%     Xl  = Ll*(2*pi*fl);
%     Zl  = Rl + 1j*Xl;
%     Pl  = ((Vlref)/abs(Zl))^2*Rl;
%     FPl = Rl/abs(Zl);
%     Il  = Vlref/abs(Zl);
%     Sl  = abs(Pl/FPl*exp(acos(FPl)*1i));
    Ql  = sqrt(Slabs^2 - Pl^2);

    %% Fase vl  
    alpha = 0*pi/180;
    fasel = alpha; 
    
    %% Parâmetros do filtro do conversor shunt      
    Ls = 5e-3;
    Cs = 300e-6;
    Rs = 0.2;
    Xs = w*Ls;
    Xc = 1/(w*Cs);

    %% Cálculo do valor eficaz da corrente drenada pela rede (rms)
    fasegl = fasel - acos(FPl);
    Ig     = roots([Rs -(Eg_rms + 2*Rs*Il*cos(fasegl)) (Pl + Rs*Il^2)]);
    Ig     = min(Ig);
    
    %% Cálculo da fase e amplitude da tensão gedada pelo conversor shunt
    Isj = -Ig + Il*(cos(fasegl) + 1i*sin(fasegl));
    Is     = abs(Isj);
    faseIs = angle(Isj);
    Vsh    = Eg_rms + (Rs + 1i*Xs)*Is*(cos(faseIs) + 1i*sin(faseIs));
    
    Vcap   = abs(1i*Xc*Is*(cos(faseIs)+1i*sin(faseIs)))*sqrt(2);
    angVcap= angle(1i*Xc*Is*(cos(faseIs)+1i*sin(faseIs)));
    Vind   = abs((Rs + 1i*Xs)*Is*(cos(faseIs)+1i*sin(faseIs)))*sqrt(2);
    absVsh = abs(Vsh);
    angVsh = angle(Vsh);
    fasesh = angVsh;
    Vshref = absVsh;
    Vshamp = Vshref*sqrt(2);
    angVshx = angle(Vsh)*180/pi;

    

    %% Definindo a tensão de Vcc
    Vcc = 180;
    
    %% Definindo os parâmetros do controlador
    Kp       = 0.02; 
    Ki       = 1.5;
    ierro_vc = 0;
    vcref    = Vcc;
    vc       = Vcc;
    I0       = Ig*sqrt(2);
    
    while t < t_f
        
        t = t + h;   						

        %% Definição das tensões de referência  
        eg = sqrt(2)*Eg_rms*cos(2*pi*fg*t);

%         vsh_ref = sqrt(2)*Vshref*cos(2*pi*fg*t + fasesh + atrasoPWM);   
        vl_ref  = sqrt(2)*Vlref*cos(2*pi*fl*t + fasel + atrasoPWM);
        vse_ref = eg - vl_ref;
        vsh_ref  = Vshamp*cos(2*pi*fl*t + fasesh);

        vcap_ref = Vcap*cos(2*pi*fl*t + angVcap);
%         vind_ref = Vzlabs*cos(2*pi*fl*t + Vzlang);
                
        %% Definição das correntes de referências 
        ig_ref = Ig*sqrt(2)*cos(2*pi*fg*t);
        il_ref = Il*sqrt(2)*cos(2*pi*fl*t - acos(FPl) + fasel);
        is_ref = Is*sqrt(2)*cos(2*pi*fl*t + faseIs);
        
        pgx = vsh_ref*ig_ref;

        %% Definição da portadora triangular (normalizada)
        vtrig = (sawtooth(2*pi*fpwm*t, 0.5)+1)/2;
        
        %% Cálculo da média
        vsh_soma = vsh_soma + vsh*h;
        vse_soma = vse_soma + vse*h;
        vl_soma  = vl_soma  + vl*h;

        if(t >= t_s)
            
            %% Cálculo da média das tensões geradas
            vsh_med = vsh_soma/(t-t_s+h_s);
            vsh_soma = 0;
            
            vse_med = vse_soma/(t-t_s+h_s);
            vse_soma = 0;
            
            vl_med = vl_soma/(t-t_s+h_s);
            vl_soma = 0;
            
            t_s = t_s + h_s;
            %% PI
            erro_vc = vcref - vc;
            ierro_vc = ierro_vc + hpwm*Ki*erro_vc;
            Ig = Kp*erro_vc + ierro_vc + I0;
%             if(Ig>25) Ig = 25;
%             end
%             if(Ig<0)  Ig = 0; 
%             end
            
%             ig_ref = Ig*sqrt(2)*cos(2*pi*fg*t);
%             is_ref = ig_ref - il_ref;

            %% PWM 
            %% Definindo as tensões de referência

%             vcap = vcap + (is_ref-is)/Cs*hpwm;
%             vsh_ref = eg - Rs*is - Ls*(is_ref - is)/hpwm;
           
            %calculo da variavel auxiliar (v*mi) 
            V = [vsh_ref, vse_ref, 0];
            vmi_max =  vcref/2 + max(V);
            vmi_min = -vcref/2 + min(V);
            mi = 0.5;
            vmi = vmi_max*mi + vmi_min*(1-mi);
    
            %geração das tensões de polo de referencia
            vs0_ref = vmi - vsh_ref;  
            vl0_ref = vmi - vse_ref;
            vg0_ref = vmi;
        end
                       
        if((vg0_ref/vcref + 0.5) > vtrig) 
            qg = 1; 
        else 
            qg = 0;
        end
        if((vl0_ref/vcref + 0.5) > vtrig) 
            ql = 1; 
        else 
            ql = 0;
        end
        if((vs0_ref/vcref + 0.5) > vtrig) 
            qs  = 1; 
        else 
            qs  = 0;
        end		
        
        vg0 = (2*qg-1)*vcref/2;
        vl0 = (2*ql-1)*vcref/2;
        vs0 = (2*qs-1)*vcref/2;
                
        %% Tensões geradas       
        vsh = vg0 - vs0;
        vse = vg0 - vl0;
        vl  =  eg - vse;

        %% Correntes geradas  
        
        der_is = (eg - vsh - Rs*is - 0*vcapx)/Ls;
        is     = is + der_is*h;
        vcapx  = vcapx + (is/Cs)*h;

        der_il = (vl - Rl*il)/Ll;
        il     = il + der_il*h;
        
        ig     = il + is; 
        
        %% Tensão no bar cc             
        ic     = ig*qg - is*qs - il*ql;
        der_vc = ic/C;
        vc     = vc + der_vc*h;
        vc = Vcc;
        %% Potência
        Pin  = eg*ig;
        Pout = vl*il;
        
        if(t <= t_f/2 + 2/60)
            j = j + 1;
            jt(j) = t;
            jig(j) = ig;
            jil(j) = il;
            jis(j) = is;
        end

        if (tsave < t)
%             tsave < t
            tsave = tsave + hsave;
            x = x + 1;

            st(x)       = t;
            seg(x)      = eg;
            sig(x)      = ig;
            sil(x)      = il;
            sis(x)      = is;
            sig_ref(x)  = ig_ref;
            sil_ref(x)  = il_ref;
            sis_ref(x)  = is_ref;
            svsh_ref(x) = vsh_ref;
            svse_ref(x) = vse_ref;
            svl_ref(x)  = vl_ref;
            svsh_med(x) = vsh_med;
            svse_med(x) = vse_med;
            svl_med(x)  = vl_med;
            svtrig(x)   = vtrig;
            svsh(x)     = vsh;
            svse(x)     = vse;
            svl(x)      = vl;
            svc(x)      = vc;
            svcref(x)   = vcref;
            svcap(x)    = vcapx;
            svcapr(x)   = vcap;
            svcap_ref(x)= vcap_ref;
%             svind_ref(x) = vind_ref;
        end
%     end
%     sSwSg(i) = SwSg;
%       amp_is(i) =  Is;
%       amp_pl(i) = Pl;
%       amp_ql(i) = Ql;
%       fp(i) = FPl;
%       angVgx(i)  = 180*fasesh/pi;
%       angIlx(i)  = 180*fasegl/pi;
%       amp_vsh(i)  = Vshref*sqrt(2);
%       amp_eg(i)   = SwSg;%sqrt(2)*Eg_rms;
%       xImVg(i)   = Il;
%       amp_vcap(i) = Vcap;
%       amp_vind(i) = Vind;
%     salpha(i) = alpha;
%     spca(i) = mean(sca_inst);
%     spcb(i) = mean(scb_inst);
%        spgx(i) = mean(pgx);
end

figure(1)
plot(st,sig_ref,st,sig,'--')
grid on
% 
% subplot(3,1,2)
% plot(st,svind_ref)
% grid on
% 
% subplot(3,1,3)
% plot(st,svsh_ref)
% grid on

%AQUI
% figure(1)
% plot(st,svcap_ref,st,svsh_ref,st,seg+svind_ref)
% legend('vcap','vsh','egt')
% grid on




% figure(1) 
% subplot(4,1,1)
% plot(st,svsh,st,svsh_med,st,svsh_ref)
% grid on
% 
% subplot(4,1,2)
% plot(st,svse,st,svse_med,st,svse_ref)
% grid on
% 
% % subplot(4,1,3)
% % plot(st,svcap,st,svcapr)
% % grid on
% 
% subplot(4,1,4)
% plot(st,svc,st,svcref)
% grid on
% 
% %%
% figure(2)
% subplot(3,1,1)
% plot(st,sig,st,sig_ref)
% grid on
% 
% subplot(3,1,2)
% plot(st,sis,st,sis_ref)
% grid on
% 
% subplot(3,1,3)
% plot(st,sil,st,sil_ref)  
% grid on



% f = figure(1);
% 
% f.Units = 'centimeters';
% f.Position = [15 7 12 10];
% f.Color = [1 1 1];
% 
% s = subplot(1,1,1);
% p = plot(fp,amp_vsh);
% s.XLabel.String = '$PF_l$';
% s.XLabel.FontSize = 12;
% s.XLabel.Interpreter = 'latex';
% s.YLabel.String = '$E$ (p.u.)';
% s.YLabel.FontSize = 12;
% s.YLabel.Interpreter = 'latex';
% hold on
% 
% s.TickLabelInterpreter = 'latex';
% s.XLim = [0.1 1];
% s.XTick = [0.1 0.2 0.3 0.4 0.5 0.6 0.7 0.8 0.9 1];
% s.YLim = [0 155.5635]/155.5635;
% s.YTick = [0 155.5635/4 155.5635/2 155.5635/1.3333 155.5635]/155.5635;

% s.YLim = [0 311.1270*2]/311.1270;
% s.YTick = [0 311.1270/4 311.1270/2 311.1270/1.3333 311.1270]/311.1270;
% grid on
% 
% 
% 
% 
% f = figure(2);
% 
% f.Units = 'centimeters';
% f.Position = [15 7 12 10];
% f.Color = [1 1 1];
% 
% s = subplot(1,1,1);
% p = plot(fp,angVgx,fp,amp_vsh);
% s.XLabel.String = '$PF_l$';
% s.XLabel.FontSize = 12;
% s.XLabel.Interpreter = 'latex';
% s.YLabel.String = '$V_{shunt}$ (V)';
% s.YLabel.FontSize = 12;
% s.YLabel.Interpreter = 'latex';
% 
% 
% s.TickLabelInterpreter = 'latex';
% % s.XLim = [0.1 1];
% % s.XTick = [0.1 0.2 0.3 0.4 0.5 0.6 0.7 0.8 0.9 1];
% % s.YLim = [150 210];
% % s.YLim = [0 400];
% 
% % s.YTick = [-200 -100 0 100 200];
% grid on






