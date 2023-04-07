%% Código 5LPUC - Versão Revista IAS
%% Jean T. Cardoso
%% Análise de Potência em regime permanente - Variando FPl, ml e thetaL

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
h      = 1e-6;                       
delt   = h;
t_f    = 1/60;                     
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
C         = 4.7e-3;
w         = 2*pi*f;
%%
% for SwSg = 0.5:0.1:1.5
% SwSg      = 0.8;
SwSg      = 1;
Eg_rms    = 220*SwSg;                 
Vlref     = 220;             
Pl        = 1000;                       
atrasoPWM = 0.5*2.*pi*f/f_s;

%for FPl = 0.1:0.01:1
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
    ig1        = 0;
    il1        = 0;

    vg_soma    = 0;
    vl_soma    = 0;
    vg         = 0;
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
    
    %% Parâmetros da rede      
    Lg = 5e-3;
    Rg = 0.2;
    Xg = Lg*(2*pi*fg);

    %% Cálculo do valor eficaz da corrente drenada da rede (rms)
    Ig    = roots([Rg -Eg_rms Pl]);
    Ig    = min(Ig);
    faseg = atan((-Ig*Xg)/(Eg_rms - Ig*Rg));
    Vgref = -Ig*Xg/sin(faseg);
    
    %% Fase vl  
%     alpha = -10*pi/180;
%     alpha = -25*pi/180;
    alpha =  -0*pi/180;
    fasel = alpha; 

    %% Definindo a tensão de Ea e Eb
    Vcc     = 340;
    Ea      = Vcc/2;
    Eb      = Vcc;
    Ea_inst = Vcc/2;
    Eb_inst = Vcc;
    
    %% Definindo os parâmetros do controlador
    Kp       = 0.2; 
    Ki       = 20;
    ierro_vc = 0;
    vcref    = Ea;
    vc       = Ea;

    %% Histere ou PI
    ke = 2; %1 - Histerese

    qg1 = 1;
    qg2 = 1;
    ql  = 1;
    qs1 = 1;
    qs2 = 1;
    
    
    contg1 = 0;
    contg2 = 0;
    contl = 0;
    conts1 = 0;
    conts2 = 0;

    while t < t_f
        
        t = t + h;   						

        %% Definição das referências 
        eg      = sqrt(2)*Eg_rms*cos(2*pi*fg*t);
        vg_ref  = sqrt(2)*Vgref*cos(2*pi*fg*t + faseg + atrasoPWM);
        vg_refx = sqrt(2)*Vgref*cos(2*pi*fg*t + faseg + atrasoPWM);
        
        vl_ref = sqrt(2)*Vlref*cos(2*pi*fl*t + fasel + atrasoPWM);
        vl_refx = sqrt(2)*Vlref*cos(2*pi*fl*t + fasel + atrasoPWM);

        if(vl_ref < 0) 	
 	        p = 0.;
 	        n = -1.;
            vl_ref = -vl_ref;
            vg_ref = -vg_ref;
        else  
		    p = 1.;
		    n = 1.;	
        end
                
        %% Definição das referências
        ig = Ig*sqrt(2)*cos(2*pi*fg*t);
        il = Il*sqrt(2)*cos(2*pi*fl*t - acos(FPl) + fasel);
        is = ig - il;
        
        %% Definição da portadora triangular (normalizada)
        vtrig = (sawtooth(2*pi*fpwm*t, 0.5)+1)/2;
        
        %% Cálculo da média
        vg_soma = vg_soma + vg*h;
        vl_soma = vl_soma + vl*h;

        if(t >= t_s)
            
            %% Cálculo da média
            vg_med = vg_soma/(t-t_s+h_s);
            vg_soma = 0;
            vl_med = vl_soma/(t-t_s+h_s);
            vl_soma = 0;
            
            t_s = t_s + h_s;
 
            %% Setores do plano vetorial 
            if(vl_ref >= Eb/2.)
			    if(vg_ref >= Eb) 
				    k = 1.;
                elseif(vg_ref >= Eb/2.)  
				    if(vl_ref <= vg_ref)
					    k = 2.;
				    else
					    k = 3.;
                    end
			    else
				    k = 4.;
                end
            else
			    if(vl_ref >= vg_ref)
				    if(vg_ref <= 0.)
					    k = 5.;
				    else
					    k = 6.;
                    end
                else
				    if(vg_ref <= Eb/2.)
					    k = 7.;
				    else
					    k = 8.;
                    end
                end
            end
                        
            if(ke == 1)
                %% Histerese para controle individual de Ea
    
                if(Ea_inst >= Ea*1.05)
				    mdc = 2.; %descarregar
                end
		    
			    if(Ea_inst <= Ea*0.95)
				    mdc = 1.; %carregar
                end
    
% 			    mdc = 2.;
            else
            %% Controlador PI para controle individual de Ea
            
                erro_vc = vcref - vc;
                ierro_vc = ierro_vc + h_s*Ki*erro_vc;
                yEa = Kp*erro_vc + ierro_vc;
    
                if (yEa >= 10.)
	    		    yEa = 10.;
	        	    ierro_vc = 10.;	
                else
	    		    if (yEa <= -10.)	
	        		    yEa = -10.;
	            	    ierro_vc = -10.;		
                    end
                end
			    
			    mi = yEa;
			
			    if(mi >= 0.)
		    	    mdc = 1.; %carregar
		        else 
		    	    mdc = 2.; %descarregar
                end
                   
% 	    	    mdc = 2; 
            end
            mdc = 1;
            if((mdc == 1 && p == 1) || (mdc == 2 && p == 0))
				if(is >= 0) 
					seq = 1.;					
				else
					seq = 2.;
                end
            else 
				if(is >= 0)
					seq = 2.;
				else
					seq = 1.;
                end		
            end
		
	        %% PWM 
            %% Caso I 
	        if(seq == 2.)
			    if(k == 1. || k == 2.)
    
				    vg101 = +n*1.1*Ea/2.;
				    v0102 = +n*1.1*Ea/2.;
				    vs02  = -n*1.1*Ea;
				    vl01  =  n*vl_ref - n*1.5*Ea;
				    vg201 = -n*vg_ref + n*vl_ref + n*Ea/2.;
			    
                elseif(k == 3. || k == 4.)
    
				    vg101 = -n*1.1*Ea/2.;
				    v0102 = +n*1.1*Ea/2.;
				    vs02  = -n*1.1*Ea;	
				    vl01  =  n*vl_ref - n*1.5*Ea;
				    vg201 = -n*vg_ref - n*Ea/2.  + n*vl_ref;   
    
                elseif(k == 5. || k == 6.)
			        vg101  = -n*1.1*Ea/2.;
			        vl01   = -n*1.1*Ea/2.;
			        vs02   = -n*1.1*Ea;
			        v0102  =  n*vl_ref - n*Ea/2.;	
			        vg201  = -n*Ea/2. - n*vg_ref + n*vl_ref;   
                else
				    vg101  =  n*1.1*Ea/2;
				    vl01   = -n*1.1*Ea/2;
				    vs02   = -n*1.1*Ea;
				    v0102  =  n*vl_ref - n*Ea/2.;	
				    vg201  = -n*vg_ref + n*vl_ref + n*Ea/2.;
                end
           
		    %% Caso II
		    else
			    if(k == 1. || k == 2.)
			    
				    vg101 = +n*1.1*Ea/2.;
				    vl01  = +n*1.1*Ea/2.;
				    vs02  = -n*1.1*Ea;
				    v0102 =  n*vl_ref - n*1.5*Ea;
				    vg201 = -n*vg_ref + n*vl_ref + n*Ea/2.;
			    
                elseif(k == 3. || k == 4.)
			    
				    vg101 = -n*1.1*Ea/2.;
				    vl01  = +n*1.1*Ea/2.;
				    vs02  = -n*1.1*Ea;	
				    v0102 =  n*vl_ref - n*1.5*Ea;
				    vg201 = -n*vg_ref - n*Ea/2.  + n*vl_ref;   
		    
                elseif(k == 5. || k == 6.)
				    
				    vg101  = -n*1.1*Ea/2.;
				    v0102  = -n*1.1*Ea/2.;
				    vs02   = -n*1.1*Ea;
				    vl01   =  n*vl_ref - n*Ea/2.;	
				    vg201  = -n*Ea/2. - n*vg_ref + n*vl_ref;   
			    
			    else
				    vg101  =  n*1.1*Ea/2;
				    v0102  = -n*1.1*Ea/2;
				    vs02   = -n*1.1*Ea;
				    vl01   =  n*vl_ref - n*Ea/2.;	
				    vg201  = -n*vg_ref + n*vl_ref + n*Ea/2.;
                end
            end  

        end
        qg1_ant = qg1;
        qg2_ant = qg2;
        ql_ant  = ql;
        qs1_ant = qs1;
        qs2_ant = qs2;
        
        if(vg101/Ea + 0.5 > vtrig) 
            qg1 = 1; 
        else 
            qg1 = 0;
        end
        if(vg201/Ea + 0.5 > vtrig) 
            qg2 = 1; 
        else 
            qg2 = 0;
        end
        if(vl01/Ea  + 0.5 > vtrig) 
            ql  = 1; 
        else 
            ql  = 0;
        end		
        if(v0102/Ea + 0.5 > vtrig) 
            qs1 = 1; 
        else 
            qs1 = 0;
        end
        if(vs02/Eb  + 0.5 > vtrig) 
            qs2 = 1; 
        else 
            qs2 = 0;
        end

        vg10a = (2*qg1-1)*Ea/2;
        vg20a = (2*qg2-1)*Ea/2;
        vl0a  = (2*ql-1)*Ea/2;
        v0a0b = (2*qs1-1)*(Eb-Ea)/2;
        vs20b = (2*qs2-1)*Eb/2;
    
        v0ax = (2*qs1-1)*(-Ea)/2;
        v0bx = (2*qs1-1)*(Eb)/2;
        
        %%        
        vg = vg10a - vg20a + vl0a + v0a0b - vs20b;
        vl = vl0a + v0a0b - vs20b;
                      
        %%               
        der_ig = (eg - vg - Rg*ig1)/Lg;
        der_il = (vl - Rl*il1)/Ll;
        
        ig1 = ig1 + der_ig*h;
        il1 = il1 + der_il*h;
        %%
        ica = ig*(qg1-qg2) + is*ql - is*qs1;
        icb = (ig-il)*(qs1 - qs2);
        
        der_vca = ica/C;
        der_vcb = icb/C;
        Ea_inst = Ea_inst + der_vca*h;
        vc = Ea_inst;
        Eb_inst = Eb_inst + der_vcb*h;
        Pin = eg*ig;
        Pout = vl*il;
        
        %%
        
        if t >= t_armazena
    
            if qg1_ant ~= qg1
                contg1=contg1+1;
            end

            if qg2_ant ~= qg2
                contg2=contg2+1;
            end
    
            if ql_ant ~= ql
                contl=contl+1;
            end
    
            if qs1_ant ~= qs1
                conts1=conts1+1;
            end

            if qs2_ant ~= qs2
                conts2=conts2+1;
            end
    
        end

        ts     = t_f - t_armazena;

        fsw_qg1 = contg1/(2*ts);
        fsw_qg2 = contg2/(2*ts);
        fsw_ql  = contl/(2*ts);
        fsw_qs1 = conts1/(2*ts);  
        fsw_qs2 = conts2/(2*ts);  
        
        fsw_m = (fsw_qg1 + fsw_qg2 + fsw_ql + fsw_qs1 + fsw_qs2)/5;

        if(t >= t_f/2)
            j = j + 1;
            jt(j) = t;
            jig(j) = ig1;
            jil(j) = il1;
        end

        if (tsave < t)
            tsave = tsave + hsave;
            x = x + 1;

            st(x)       = t;
            seg(x)      = eg;
            sig(x)      = ig;
            sig1(x)     = ig1;
            sil(x)      = il;
            sil1(x)     = il1;
            sis(x)      = is;
            svg_ref(x)  = vg_ref;
            svl_ref(x)  = vl_ref;
            svg_med(x)  = vg_med;
            svl_med(x)  = vl_med;
            sk(x)       = k;
            svtrig(x)   = vtrig;
            svg10a(x)   = vg10a;
            svg201(x)   = vg20a;
            svl0a(x)    = vl0a;
            sv0a0b(x)   = v0a0b;
            svg(x)      = vg;
            svl(x)      = vl;
            sEa_inst(x) = Ea_inst;
            sca_inst(x) = ica*Ea;
            scb_inst(x) = icb*Eb;
            smdc(x)     = mdc;
            smi(x)      = mi;
            svl_refx(x) = vl_refx;
            svg_refx(x) = vg_refx;
        end
    end
%     sSwSg(i) = SwSg;
%     fp(i) = FPl;
%     salpha(i) = alpha;
%     spca(i) = mean(sca_inst);
%     spcb(i) = mean(scb_inst);
% end

% figure(2),plot(st,sEa_inst)
% figure(3)
% plot(fp,spca)
% figure(2),plot(st,svg_med)
% figure(1),plot(st,svl0a)
% figure(2),plot(st,sv0a0b)

% figure(3),plot(st,sig1,st,sig)
% figure(4),plot(st,sil1,st,sil)
% figure(5),plot(jt,jig,st,sig)
% figure(6),plot(jt,jil)

% figure(2),plot(st,smdc,st,smi)

% hold on
figure(4),plot(st,svl_med,st,svl_ref)
% 
% figure(5),plot(sSwSg,spca/Pl),grid on
% hold on

% f = figure(1);
% 
% f.Units = 'centimeters';
% f.Position = [15 7 10 5];
% 
% f.Color = [1 1 1];
% s = subplot(1,1,1);
% p = plot(sSwSg,spca/Pl);
% s.XLabel.String = '$E_g$(p.u.)';
% s.XLabel.FontSize = 12;
% s.XLabel.Interpreter = 'latex';
% s.YLabel.String = '$P_{Ca}$ (p.u.)';
% s.YLabel.FontSize = 12;
% s.YLabel.Interpreter = 'latex';
% 
% s.TickLabelInterpreter = 'latex';
% s.XLim = [.5 1.5];
% s.XTick = [0.5 0.75 1 1.25 1.5];
% s.YLim = [-1.8 0.6];
% s.YTick = [-1.8 -1.2 -0.6 0 0.6];
% grid on
% hold on

%--------WTHD------------%
% wthd_vg = wthdf(svg, 1/h, f)
% wthd_vl = wthdf(svl, 1/h, f)

% thd_ig = thdf(jig, 1/h, f)
% thd_il = thdf(jil, 1/h, f)

% figure(1),plot(jt,jig,st,sig)
% 
% f = figure(1);
% f.Units = 'centimeters';
% f.Position = [0 0 5 4];
% f.Color = [1 1 1];
% %---------------------------------
% 
% s = subplot(1,1,1);
% p = plot(st,svg,st,svg_refx);
% s.XLabel.String = '$t$ (s)';
% s.XLabel.FontSize = 12;
% s.XLabel.Interpreter = 'latex';
% s.YLabel.String = '$v_g$(V)';
% s.YLabel.FontSize = 12;
% s.YLabel.Interpreter = 'latex';
% 
% s.TickLabelInterpreter = 'latex';
% s.XLim = [2/60 4/60];
% s.XTick = [2/60 3/60 4/60];
% s.YLim = [-550 550];
% s.YTick = [-500 -275 0 275 500];
% grid on
% 
% s.YLim = [-550 550];
% s.YTick = [-500 -275 0 275 500];
% 
% s.YLim = [-370 370];
% s.YTick = [-370 -185 0 185 370];
