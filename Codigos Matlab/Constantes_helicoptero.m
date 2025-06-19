clc
clear all;

g = 9.81;

% Datos para cambiar
LongitudTotalVarilla = 0.6;
LongitudTotalVarilla1 = 0.283;
Lc = 0.16; % Distancia entre el eje y centro de masa del cable

Mcp = 0.041; % Masa del contrapeso
Mc = 0.003808; % Masa  del cable
Mm = 0.024; % Masa del motor
Mj = 0.044; % Masa de la jaula con su soporte

% Cálculos

Lcp = 0.271; % Distancia entre el eje y centro de masa del contrapeso
Lv1 = LongitudTotalVarilla1/2;
Lv2 = (LongitudTotalVarilla-LongitudTotalVarilla1)/2;
Lm = 2 * Lv2 + 0.014; % Distancia entre el eje y centro de masa del motor, el motor está a 
Lj = Lm - 3.13/1000;% El centro de masa de jaula está a 3.13mm antes que el del motor

Mv1 = 0.3867 * (2*Lv1); % Masa  de la varilla 1
Mv2 = 0.3867 * (2*Lv2); % Masa de la varilla 2
% Pesos
Pcp = Mcp * g;     
Pv1 = Mv1 * g;     
Pv2 = Mv2 * g;    
Pc = Mc * g;       
Pm = Mm * g;       
Pj = Mj * g;       



Rv1 = 0.004;     Rv2 = 0.004;     Rc = 0.002; % Radios

Icp = Mcp * Lcp^2;
Im = Mm * Lm^2;
Ij = Mj * Lj^2;
Iv1 = (1/4) * Mv1 * Rv1^2 + (1/3) * Mv1 * (2 * Lv1)^2;
Iv2 = (1/4) * Mv2 * Rv2^2 + (1/3) * Mv2 * (2 * Lv2)^2;
Ic = (1/4) * Mc * Rc^2 + (1/3) * Mc * (Lc)^2;

I = Icp + Im + Ij + Iv1 + Iv2 + Ic;
I

C = Pcp * Lcp + Pv1 * Lv1 - Pc * Lc - Pv2 * Lv2 - Pj * Lj - Pm * Lm;

C
