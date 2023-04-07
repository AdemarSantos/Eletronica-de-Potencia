q000 = [0 0 0];
q001 = [0 0 1];
q010 = [0 1 0];
q011 = [0 1 1];
q100 = [1 0 0];
q101 = [1 0 1];
q110 = [1 1 0];
q111 = [1 1 1];

v000 = [0 0];           % Vg = 0, Vl = 0
v001 = [0 E];           % Vg = 0, Vl = E
v010 = [E 0];           % Vg = E, Vl = 0
v011 = [E E];           % Vg = E, Vl = E
v100 = [-E -E];         % Vg =-E, Vl =-E
v101 = [-E 0];          % Vg =-E, Vl = 0
v110 = [0 -E];          % Vg = 0, Vl =-E
v111 = [0 0];           % Vg = 0, Vl = 0


if     vg_ref > vl_ref && vg_ref > 0 && vl_ref >= 0
    k = 1;
elseif vg_ref <= vl_ref && vg_ref > 0 && vl_ref > 0   
    k = 2;
elseif                    vg_ref <= 0 && vl_ref > 0
    k = 3;
elseif vg_ref < vl_ref && vg_ref < 0 && vl_ref <= 0   
    k = 4;
elseif vg_ref >= vl_ref && vg_ref < 0 && vl_ref < 0   
    k = 5;
elseif                    vg_ref >= 0 && vl_ref < 0   
    k = 6;
end


if k == 1
    t2 = (vg_ref - vl_ref)*(htriangle/E);      % t1
    t3 = vl_ref * (htriangle/E);               % t2
    to = htriangle - (t2 + t3);
    t1 = u * to;
    t4 = (1 - u)*to;
    q1 = q111;
    q2 = q100;
    q3 = q110;
    q4 = q000;
    
elseif k == 2
    t2 = vg_ref * (htriangle/E);               % t2
    t3 = (vl_ref - vg_ref)*(htriangle/E);      % t3
    to = htriangle - (t2 + t3);
    t1 = u * to;
    t4 = (1 - u)*to;
    q1 = q111;
    q2 = q110;
    q3 = q010;
    q4 = q000;
    
elseif k == 3
    t2 = vl_ref*(htriangle/E);                 % t3
    t3 = -vg_ref*(htriangle/E);                % t4
    to = htriangle - (t2 + t3);
    t1 = u * to;
    t4 = (1 - u)*to;
    q1 = q111;
    q2 = q010;
    q3 = q011;
    q4 = q000;
    
elseif k == 4
    t2 = (vl_ref-vg_ref)*(htriangle/E);        % t4
    t3 = -vl_ref*(htriangle/E);                % t5
    to = htriangle - (t2 + t3);
    t1 = u * to;
    t4 = (1 - u)*to;
    q1 = q111;
    q2 = q011;
    q3 = q001;
    q4 = q000;
    
elseif k == 5
    t2 = -vg_ref*(htriangle/E);                % t5
    t3 = (vg_ref-vl_ref)*(htriangle/E);        % t6
    to = htriangle - (t2 + t3);
    t1 = u * to;
    t4 = (1 - u)*to;
    
    q1 = q111;
    q2 = q001;
    q3 = q101;
    q4 = q000;
    
elseif k == 6
    t2 = -vl_ref*(htriangle/E);                % t6
    t3 = vg_ref*(htriangle/E);                 % t1
    to = htriangle - (t2 + t3);
    t1 = u * to;
    t4 = (1 - u)*to;
    
    q1 = q111;
    q2 = q101;
    q3 = q100;
    q4 = q000;
    
end

