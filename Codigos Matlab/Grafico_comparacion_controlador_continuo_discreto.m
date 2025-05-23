clear; clc; close all;

%% === Parámetros del controlador PIDf (autotuner simulink) ===
Kp = 106.62644;
Ki = 95.63817;
Kd = 102.99198;
N  = 12.00000;

%% === Modelo continuo del controlador PID ===
s = tf('s');
controlador_continuo = tf([(Kp + Kd*N), (Kp*N + Ki), Ki*N], [1, N, 0]);

%% === Parámetros del modelo del sistema ===
% Constantes del modelo físico
I  = 0.0167;  % Momento de inercia
C  = -0.1326; % Constante relacionada al torque
Lm = 0.3310;  % Longitud desde el eje hasta el motor     

angulo_equilibrio =  0 * (pi/180);  % Ángulo de equilibrio en radianes para linealización
condicion_inicial_angulo_modelo_no_lineal = angulo_equilibrio;

A = Lm / I;
B = C * sin(angulo_equilibrio) / I;


% Modelo del motor brushless Fh = m*PWM + r
m = 0.001586;
r = -1.692631;

fuerza_equilibrio  =  -(C * cos(angulo_equilibrio)) / Lm;
PWM_equilibrio = (fuerza_equilibrio-r)/m;
       
ref = 0;  % En radianes

%% === Planta continua ===
planta_mecanica_continua = tf([A], [1 0 B]);
planta_electrica_continua = m;

%% === Parámetros de simulación ===
% Tiempo de muestreo [s]
if B == 0
    T = 0.01;  % Tiempo de muestreo fijo si B es cero
else
    T = abs(1 / (10 * sqrt(B)));  % Tiempo de muestreo 10 veces menor a la frecuencia natural
end

Tsim = 20;    % Duración total de la simulación [s]
t = 0:T:Tsim; % Vector de tiempo para calcular cantidad de muestras
cantidad_muestras = length(t); % Cantidad de muestras basadas en el vector  de tiempo

%% === Entrada de referencia constante ===
entrada = ref * ones(1, cantidad_muestras);

%% === Controlador discreto manual (ecuaciones en diferencias) ===
a = Kp + Kd*N;
b = Kp*N + Ki;
c = Ki*N;
d = N;

K1 = (b*d - c)/(d^2);
K2 = c/d;
K3 = (a*d^2 - b*d + c)/(d^2);

a0 = K1 + K3;
a1 = -K1 - K1*exp(-d*T) + K2*T - 2*K3;
a2 = K1*exp(-d*T) - K2*T*exp(-d*T) + K3;
a3 =  1;
a4 = -exp(-d*T) - 1;
a5 = exp(-d*T);

%% === Discretización de la planta con ZOH ===
planta_mecanica_discreta = c2d(planta_mecanica_continua, T, 'zoh');
[num_p, den_p] = tfdata(planta_mecanica_discreta, 'v');
a1p = num_p(2);
a2p = num_p(3); 
b1p = den_p(1);
b2p = den_p(2); 
b3p = den_p(3);

%% === Inicialización de señales ===
salida_modelo_electrico = zeros(1, cantidad_muestras);  
salida_modelo_mecanico = zeros(1, cantidad_muestras);  
salida_planta = zeros(1, cantidad_muestras);  
PWM_aplicar = zeros(1, cantidad_muestras);  

u_manual       = zeros(1, cantidad_muestras);  % Señal de control
error          = zeros(1, cantidad_muestras);  % Error entre referencia y salida
perturbacion          = zeros(1, cantidad_muestras);  % Perturbación

%perturbacion(t<1)=0.1;

% Variables para valores anteriores (memoria del sistema)
errorm1 = 0; errorm2 = 0;
ukm1 = 0; ukm2 = 0; % salida controlador
ykm1 = 0; ykm2 = 0; % salida planta

smem1 = 0; smem2 = 0; % salida modelo eléctrico
smmm1 = 0; smmm2 = 0; % salida modelo mecánico


%% === Simulación con controlador discreto manual ===
for k = 1:cantidad_muestras
    % Cálculo del error
    error(k) = ref - ykm1;
    
    % Ecuación en diferencias del controlador
    u_manual(k) = (1/a3)*(a0*error(k) + a1*errorm1 + a2*errorm2 ...
                   - a4*ukm1 - a5*ukm2);
    
    % Saturación PWM
    %if u_manual(k) > 2000
    % u_manual(k) = 2000;
    %elseif u_manual(k) < 1000
    % u_manual(k) = 1000;
    %end
    
    % Simulacion Planta: Eléctrica + Mecánica en variable absoluta

    %salida_modelo_electrico(k) = (u_manual(k)-PWM_equilibrio)*(m) + fuerza_equilibrio;
    

    %tf_modelo_mecanico = (1/b1p)*(a1p * (smem1 - fuerza_equilibrio) + a2p * (smem2 - fuerza_equilibrio) ...
    %                      - b2p * smmm1 - b3p * smmm2);

    %salida_modelo_mecanico(k) = tf_modelo_mecanico + angulo_equilibrio;
    %salida_planta(k) = salida_modelo_mecanico(k);
    
    % Simulacion Planta: Eléctrica + Mecánica en variable incremental
    
    % === Saturación del PWM real ===
    PWM_aplicar(k) = u_manual(k) + PWM_equilibrio;
    
    if PWM_aplicar(k) > 2000
        PWM_aplicar(k) = 2000;
    elseif PWM_aplicar(k) < 1000
        PWM_aplicar(k) = 1000;
    end
    
    u_manual(k) = PWM_aplicar(k) - PWM_equilibrio;  % acción de control saturada


    salida_modelo_electrico(k) = m*u_manual(k);
    tf_modelo_mecanico = (1/b1p)*(a1p * (smem1) + a2p * (smem2) ...
                          - b2p * smmm1 - b3p * smmm2);
    
    salida_modelo_mecanico(k) = tf_modelo_mecanico;
    salida_planta(k) = salida_modelo_mecanico(k)+ perturbacion(k);
    
    % Actualizar memorias del controlador
    errorm2 = errorm1; errorm1 = error(k);
    ukm2 = ukm1;       ukm1 = u_manual(k);

    % Actualizar memorias de la planta
    smem2 = smem1;      smem1 = salida_modelo_electrico(k); 
    smmm2 = smmm1;      smmm1 = salida_modelo_mecanico(k);
    ykm2 = ykm1;       ykm1 = salida_planta(k);
end

%% === Datos simulados desde Simulink para comparación ===
tiempo_simulink = [];

salida_simulink = [];

%% === Gráfica comparativa de respuestas ===
figure('Name', 'Comparación PID manual vs Simulink', 'NumberTitle', 'off');

% Respuesta del sistema + referencia
subplot(3,1,1);
stairs(t, salida_planta, 'b', 'LineWidth', 1.8); hold on;
plot(tiempo_simulink, salida_simulink, 'm--', 'LineWidth', 1.5);
plot(t, entrada, ':', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.5); % entrada en gris punteado
title('Respuesta de la planta con PID: Discretización manual vs Simulink');
xlabel('Tiempo [s]');
ylabel('Salida del sistema');
legend('Respuesta PID Discreto Manual', 'Respuesta Simulink', 'Entrada de referencia');
grid on;

% Error en el tiempo
subplot(3,1,2);
plot(t, error, 'r', 'LineWidth', 1.5);
title('Evolución del error');
xlabel('Tiempo [s]');
ylabel('Error = referencia - salida');
grid on;

% Acción de control(multiplicada por m ya que esta es la encargada de la normalización)
%subplot(3,1,3);
%stairs(t, u_manual, 'c', 'LineWidth', 1.5); hold on;
%title('Acción de control (u[k])');
%xlabel('Tiempo [s]');
%ylabel('Señal de control');
%grid on;

% Acción de control(multiplicada por m ya que esta es la encargada de la normalización)
subplot(3,1,3);
stairs(t, PWM_aplicar, 'c', 'LineWidth', 1.5); hold on;
title('PWM aplicar');
xlabel('Tiempo [s]');
ylabel('PWM aplicar');
grid on;

%% ------------------------------------------ EXTRA -------------------------------

%% === Discretización ZOH del controlador PIDf y comparación ===
% (usa la misma variable 'T' y 'controlador_continuo' ya definidas)

controlador_discreto_zoh = c2d(controlador_continuo, T, 'zoh');
[num_zoh, den_zoh]       = tfdata(controlador_discreto_zoh, 'v');

% --- Coeficientes ZOH ---
b0_z = num_zoh(1);   b1_z = num_zoh(2);   b2_z = num_zoh(3);
a1_z = den_zoh(2);   a2_z = den_zoh(3);   % (a0_z = 1)

% --- Comparación con tus coeficientes manuales ---
fprintf('\n==============================================================\n');
fprintf('Comparación de coeficientes del controlador discreto PIDf\n');
fprintf('            Manual\t\tZOH\t\tΔ (ZOH-Manual)\n');
fprintf('b0 ≡ a0  %12.6f\t%12.6f\t% .3e\n', a0, b0_z, b0_z - a0);
fprintf('b1 ≡ a1  %12.6f\t%12.6f\t% .3e\n', a1, b1_z, b1_z - a1);
fprintf('b2 ≡ a2  %12.6f\t%12.6f\t% .3e\n', a2, b2_z, b2_z - a2);
fprintf('a1 ≡ a4  %12.6f\t%12.6f\t% .3e\n', a4, a1_z, a1_z - a4);
fprintf('a2 ≡ a5  %12.6f\t%12.6f\t% .3e\n', a5, a2_z, a2_z - a5);
fprintf('==============================================================\n');