%% ==============================================================
%  COMPARACIÓN   C(s)  VS.  CONTROLADOR DISCRETO  (ZOH)
%  Transformación  P(z) = (1-z^-1)·Z{C(s)/s}
% ==============================================================
clear; clc;

%% --- Datos del PID continuo ------------------------------------------
Kp = 106.62644;
Ki =  95.63817;
Kd = 102.99198;
N  = 12.0;

T       = 0.1;          % periodo de muestreo (s)
t_total = 3.0;          % simular 3 s
dt      = 1e-3;         % paso fino para el modelo continuo

% Polinomio del PIDf continuo: C(s) = (a s² + b s + c)/(s (s+d))
a = Kp + Kd*N;
b = Kp*N + Ki;
c = Ki*N;
d = N;
C_s = tf([a b c],[1 d 0]);          % num/den en s

%% --- Ecuación en diferencias (tu resultado) ---------------------------
K1 = (b*d - c)/(d^2);
K2 =  c/d;
K3 = (a*d^2 - b*d + c)/(d^2);
eA = exp(-d*T);

a0 =  K1 + K3;
a1 = -K1 - K1*eA + K2*T - 2*K3;
a2 =  K1*eA - K2*T*eA + K3;
a3 =  1;
a4 = -eA - 1;
a5 =  eA;

pidf_discrete = @(e, em1, em2, um1, um2) ...
    (a0*e + a1*em1 + a2*em2 - a4*um1 - a5*um2)/a3;

%% --- Entradas de prueba -----------------------------------------------
u_in   = { @(t) ones(size(t));      % Escalón
           @(t) t;                  % Rampa
           @(t) t.^2 };             % Parabólica
tit    = { 'Escalón', 'Rampa', 'Parabólica' };

%% --- Simulación --------------------------------------------------------
t_cont = (0:dt:t_total)';           % malla fina continua
col    = lines(3);                  % paleta

figure;
for n = 1:3
    %% 1) Controlador continuo (lsim) -----------------------------------
    e_c  = u_in{n}(t_cont);
    u_ct = lsim(C_s, e_c, t_cont);

    %% 2) Controlador discreto + retenedor ZOH --------------------------
    t_k   = (0:T:t_total)';         % instantes de cálculo
    e_k   = u_in{n}(t_k);

    % ecuación en diferencias
    u_k = zeros(size(t_k));
    em1=0; em2=0;  um1=0; um2=0;
    for k = 1:length(t_k)
        u_k(k) = pidf_discrete(e_k(k),em1,em2,um1,um2);
        em2=em1; em1=e_k(k);
        um2=um1; um1=u_k(k);
    end

    % reconstrucción ZOH en malla continua
    u_zoh = zeros(size(t_cont));
    for j = 1:length(t_cont)
        idx = find(t_k <= t_cont(j),1,'last');
        u_zoh(j) = u_k(idx);
    end

    %% 3) Gráfica --------------------------------------------------------
    subplot(3,1,n);  hold on;
    plot(t_cont, e_c ,':', 'Color',[.5 .5 .5],'LineWidth',1.2);
    plot(t_cont, u_ct,'-', 'Color',col(2,:),'LineWidth',2.0);      % continuo
    plot(t_cont, u_zoh,'-.','Color',col(3,:),'LineWidth',1.8);     % ZOH
    plot(t_k,    u_k ,'ko','MarkerFaceColor','k','MarkerSize',5);  % muestras

    title(['Respuesta a ', tit{n}]);
    xlabel('Tiempo [s]');  ylabel('u(t)');
    legend('Entrada', 'C(s) continuo', 'Discreto + ZOH', 'u[k]',...
           'Location','northwest');
    grid on;
end
sgtitle('Controlador continuo vs. controlador discreto obtenido con (1-z^{-1})·Z{C(s)/s}');
