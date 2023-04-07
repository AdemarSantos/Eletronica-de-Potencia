close all;
clear all;
clc;


t = 0;                          %tempo incial             
tf = 0.02;                       %tempo final
h = 1E-7;                       %passo de cáculo

%--------------------------------------------------------------------------

tsave0 = 0.;             
tsave = tsave0;          
npt = 50000;            
hsave = (tf-tsave0)/npt; 

if hsave < h             
    hsave = h;           
    npt = (tf-tsave0)/hsave; 
end


%--------------------------------------------------------------------------

rg = 0.25;
lg = 3.5e-3;
rl = 15;
ll = 15e-3;

ig = 0;
il = 0;


Ea = 60;
Eb = 60;
E = (Ea + Eb)/2;
                      %tensão cc de barramento
vcrm = 98;                     %amplitude inicial   
w = 2*pi*60;                %frequência ângular

pi23 = 2*pi/3;


    

tt = 0;                     %tempo inical da portadora      
ft = 10e3;                  %frequência inical da portadora
ht = 1/ft;                  %meio período da portadora
vt = E/2;                   %tensão inicial da portadora 
dt = E/ht;                  %derivada da portadora 
sign = -1; 

vgInt = 0;                 %iniciando a integral de vg0
vlInt = 0;                 %iniciando a integral de vl0
vaInt = 0;                 %iniciando a integral de va0



u = 0.5;                    %valor de mi
j = 0;                      %indíce dos vetores de gravação

%--------------------------------------------------------------------------

while t<tf
    t = t + h;
        
    if t >= tt
        tt = tt + ht;
        if vt <= 0
            vt = -E/2;
            sign = 1;
        else
            vt = E/2;
            sign = -1;
        end
        
        eg = vcrm * cos(w*t);
        
        vgMed = vgInt / ht;               %valor médio de vg0
        vlMed = vlInt / ht;               %valor médio de vl0
        vaMed = vaInt / ht;               %valor médio de va0
        
        vgInt = 0;
        vlInt = 0;
        vaInt = 0;
       
        
        igref = 5.5 * cos(w *t);             %ig de referência
        vgRef = eg - rg * ig - lg*(igref - ig)/ht;      %vg de referência
        vlRef = vcrm * cos(w*t);              %vl de referência  

        %cálculo do fator de repartição para as tensões vgab,vlab e vaab
        VSRef = [vgRef,vlRef,0];
        vuRefMax = E - max(VSRef);                               
        vuRefMin = -E - min(VSRef);                
        vuRef = (vuRefMax + vuRefMin)/2  ;          
        
        vgabRef = vgRef + vuRef;             %vgab de referência
        vlabRef = vlRef + vuRef;             %vlab de referência
        vaabRef = vuRef;                     %vaab de referência
        
        
        %cálculo do fator de repartição para as tensões de polo do braço g
        Vyg = [vgabRef/2, -vgabRef/2];
        vygRefMax = E/2 - max(Vyg);
        vygRefMin = E/2 - min(Vyg);
        vygRefMed = (vygRefMax + vygRefMin) / 2;
        
        vga0aRef = vgabRef/2 + vygRefMed;
        vgbobRef = -vgabRef/2 + vygRefMed;
        
        %cálculo do fator de repartição para as tensões de polo do braço l
        Vyl = [vlabRef/2, -vlabRef/2];
        vylRefMax = E/2 - max(Vyl);
        vylRefMin = E/2 - min(Vyl);
        vylRefMed = (vylRefMax + vylRefMin) / 2;
        
        vla0aRef = vlabRef/2 + vylRefMed;
        vlbobRef = -vlabRef/2 + vylRefMed;
        
        %cálculo do fator de repartição para as tensões de polo do braço l
        Vya = [vaabRef/2, -vaabRef/2];
        vyaRefMax = E/2 - max(Vya);
        vyaRefMin = E/2 - min(Vya);
        vyaRefMed = (vyaRefMax + vyaRefMin) / 2;
        
        vaa0aRef = vaabRef/2 + vyaRefMed;
        vabobRef = -vaabRef/2 + vyaRefMed;
        
        
        
    end
    
    vt = vt + sign * dt * h;
    
    %inicinado as ondas triângulares
    vtpos = vt + E/2;
    vtneg = vt - E/2;
    
    %cálculo do estado das chaves
    
    if (vgabRef > 0)
     
        if (vgabRef >= vtpos)
            q1a=1; 
            q1b=0;
        else
            q1a=1; 
            q1b=1;
        end
     
    else
    
        if (vgabRef >= vtneg)
            q1a=0; 
            q1b=0;
        else
            q1a=0; 
            q1b=1;
        end
    
    end
    
    if (vlabRef > 0)
     
        if (vlabRef >= vtpos)
            q2a=1; 
            q2b=0;
        else
            q2a=1; 
            q2b=1;
        end
     
    else
    
        if (vlabRef >= vtneg)
            q2a=0; 
            q2b=0;
        else
            q2a=0; 
            q2b=1;
        end
    end
    
    
    if (vaabRef > 0)
     
        if (vaabRef >= vtpos)
            q3a=1; 
            q3b=0;
        else
            q3a=1; 
            q3b=1;
        end
     
    else
    
        if (vaabRef >= vtneg)
            q3a=0; 
            q3b=0;
        else
            q3a=0; 
            q3b =1;
        end
    end
         
    %cálculo das tensões de polo atuais 
    
    vga0a = (2 * q1a - 1) * (Ea/2);
    vgb0b = (2 * q1b - 1) * (Eb/2);
    
    
    vla0a = (2 * q2a - 1) * (Ea/2);
    vlb0b = (2 * q2b - 1) * (Eb/2);

    
    vaa0a = (2 * q3a - 1) * (Ea/2);
    vab0b = (2 * q3b - 1) * (Eb/2);
    
    
    %cálculo das tensões de polo entre as duas partes do conversor
    
    vgab = vga0a - vgb0b;
    vlab = vla0a - vlb0b;
    vaab = vaa0a - vab0b;
    
    %cálculo das tensões de saída do conversor
    vg = vgab - vaab;
    vl = vlab - vaab;


    vgInt = vgInt + vg * h;
    vlInt = vlInt + vl * h;
    
    %cálculo das corrente do conversor
    
    dev_ig = (eg - vg - rg*ig)/lg;
    dev_il = (vl - rl*il)/ll;
    
    ig = ig + dev_ig*h;
    il = il + dev_il*h;

    %armazenando variáveis de saída
    
    if tsave <= t
        tsave = tsave + hsave;
        j = j + 1;
        T(j) = tsave;
        svg(j) = vg;
        svgMed(j) = vgMed;
        svgRef(j) = vgRef;
        svl(j) = vl;
        svlMed(j) = vlMed;
        svlRef(j) = vlRef;
        sig(j) = ig;
        sil(j) = il;
        
        
    end
end

%--------------------------------------------------------------------------

figure(1)
plot(T,svg,T,svgMed,T,svgRef);
grid()
title('vg - vgMed - vgRef')
legend('vg' , 'vgMed' , 'vgRef')

figure(2)
plot(T,svl,T,svlMed,T,svlRef);
grid()
title('vl')
legend('vl' , 'vlMed' , 'vlRef')

figure(3)
plot(T,sig,T,sil)
grid()
title('correntes ig e il')
legend('ig','il')
figure(4)
plot(T,svg,T,svgMed,T,svl,T,svlMed)
