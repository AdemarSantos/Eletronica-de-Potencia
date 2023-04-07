%% Código 3Ls - Versão Revista IAS
%% Jean T. Cardoso
%% Análises...

clear;
% close all;
clc;

%% Inicializações 1
i = 0;
p = 0;
j = 0;

%% Parâmetros de simulação
f_s    = 10e3;                     
h_s    = 1/f_s;                    
fpwm   = f_s;
h      = 1e-6;                       
delt   = h;
t_f    = 7/60;                     
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
C         = 4.4e-3;
w         = 2*pi*f;
%%
% for SwSg = 0.5:0.1:1.5
SwSg      = 2;
Eg_rms    = 220*SwSg;                 
Vlref     = 220;             
Pl        = 1000;                       
atrasoPWM = 0.5*2.*pi*f/f_s;

% for FPl = 0.1:0.01:1
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
    
    vsh        = 0;
    vse        = 0;
    vl         = 0;
    t          = 0;                          
    t_s        = 0;
    t_armazena = 0;

    %% Parâmetros da carga
    FPl = 0.8;
    Sl  = Pl/FPl*exp(acos(FPl)*1i);   
    Zl  = conj((Vlref^2)/(Sl));	
    Rl  = real(Zl);                  
    Xl  = imag(Zl);                  
    Ll  = Xl/(2*pi*fl);
    Il  = Vlref/abs(Zl);
      
    %% Parâmetros do filtro do conversor shunt      
    Ls = 7e-3;
    Rs = 0.2;
    Xs = w*Ls;

    %% Cálculo do valor eficaz da corrente drenada pela rede (rms)
    fasel = 5*pi/180;
    fasegl = cos(fasel - acos(FPl));
    Ig     = roots([Rs -(Eg_rms+2*Rs*Il*fasegl) (Pl+Rs*Il^2)]);
    Ig     = min(Ig);

    %% Cálculo da fase e amplitude da tensão gedada pelo conversor shunt
    Is     = sqrt(Ig^2+Il^2-2*Ig*Il*fasegl);
    faseIs = acos((Ig^2+Is^2-Il^2)/(2*Ig*Is));
    Vsh    = Eg_rms - (Rs + 1i*Xs)*Is*(cos(faseIs)+1i*sin(faseIs));
    absVsh = abs(Vsh);
    angVsh = angle(Vsh);
    fasesh = angVsh;
    Vshref = absVsh;

    %% Definindo a tensão do barramento
    Vcc = 340;

    %% Limites do Conversor 3Ls
    Vpu    = 110;
    E      = Vcc/(Vpu*sqrt(2));
    Vsh    = Vshref/Vpu;
    Eg     = Eg_rms/Vpu;
    Vl     = Vlref/Vpu;
    thetaL = fasel;
    Vse    = sqrt(Vl^2 + Eg^2 - 2*Vl*Eg*cos(thetaL));
    VseA   = Vpu*sqrt(2)*Vse
    Vsek   = Eg_rms - Vlref*(cos(fasel)+1i*sin(fasel));
    absVse = abs(Vsek)*sqrt(2)
    angVse = angle(Vsek)*180/pi
    kj     = (180/pi)*acos((Vsh^2 + Vse^2 - E^2)/(2*Vsh*Vse));
    kjc    = abs(kj)
              
    %% Definindo os parâmetros do controlador
    Kp       = 0.2; 
    Ki       = 20;
    ierro_vc = 0;
    vcref    = Vcc;
    vc       = Vcc;

    while t < t_f
        
        t = t + h;   						

        %% Definição das tensões de referência  
        eg      = sqrt(2)*Eg_rms*cos(2*pi*fg*t);
        vsh_ref = sqrt(2)*Vshref*cos(2*pi*fg*t + fasesh + atrasoPWM);   
        vl_ref  = sqrt(2)*Vlref*cos(2*pi*fl*t + fasel + atrasoPWM);
        vse_ref = eg - vl_ref;


        %% Definição das correntes de referências 
        ig_ref = Ig*sqrt(2)*cos(2*pi*fg*t);
        il_ref = Il*sqrt(2)*cos(2*pi*fl*t - acos(FPl) + fasel);
        is_ref = ig_ref - il_ref;

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

	        %% PWM 
            %% Definindo as tensões de referência

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
        der_is = (-vsh + eg - Rs*is)/Ls;
        der_il = (vl - Rl*il)/Ll;
        is     = is + der_is*h;
        il     = il + der_il*h;
        ig     = il + is; 
        
        %% Tensão no bar cc             
        ic     = ig*qg + is*qs - il*ql;
        der_vc = ic/C;
        vc     = vc + der_vc*h;
        
        %% Potência
        Pin  = eg*ig;
        Pout = vl*il;
        
%         if(t >= t_f/2)
%             j = j + 1;
%             jt(j) = t;
%             jig(j) = ig;
%             jil(j) = il;
%         end

        if (tsave < t)
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

        end
%     end
%     sSwSg(i) = SwSg;
%     fp(i) = FPl;
%     salpha(i) = alpha;
%     spca(i) = mean(sca_inst);
%     spcb(i) = mean(scb_inst);
end

figure(1)
subplot(3,1,1)
plot(st,svsh,st,svsh_med,st,svsh_ref)
subplot(3,1,2)
plot(st,svse,st,svse_med,st,svse_ref)
subplot(3,1,3)
plot(st,svl,st,svl_med,st,svl_ref)

figure(2)
subplot(3,1,1)
plot(st,sig,st,sig_ref)
subplot(3,1,2)
plot(st,sis,st,sis_ref)
subplot(3,1,3)
plot(st,sil,st,sil_ref)

% figure(3)
% plot(st,sig,st,sil)

figure(4)
plot(st,svse_ref,st,svsh_ref)
grid on

