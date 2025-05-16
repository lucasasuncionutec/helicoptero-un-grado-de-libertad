clc
clear all

%% === Parámetros del controlador PIDf (autotuner simulink) ===
Kp = 106.62644;
Ki = 95.63817;
Kd = 102.99198;
N  = 12.00000;

%% === Parámetros del modelo del sistema ===
% Constantes del modelo físico
I  = 0.0167;  % Momento de inercia
C  = -0.1326; % Constante relacionada al torque
Lm = 0.3310;  % Longitud desde el eje hasta el motor   

% Modelo del motor brushless Fh = m*PWM + r
m = 0.001586;
r = -1.597768;

% === Crear figura general ===
figure('Name', 'Comparación de respuestas para distintos ángulos de equilibrio');
hold on;
grid on;

% === Simulaciones para diferentes ángulos de equilibrio ===
for i = 0:1
    angulo_equilibrio = i * (pi/180);  % Ángulo de equilibrio en radianes
    condicion_inicial_angulo_modelo_no_lineal = angulo_equilibrio;
    
    A = Lm / I;
    B = C * sin(angulo_equilibrio) / I;
    
    fuerza_equilibrio  = -(C * cos(angulo_equilibrio)) / Lm;
    PWM_equilibrio = (fuerza_equilibrio - r) / m;
           
    ref = angulo_equilibrio;  
    
    % === Simulación ===
    modelo = sim("ProyectoIntegrador2025.slx");
    
    angulo_lineal    = modelo.logsout.getElement("angulo_lineal").Values;
    angulo_no_lineal = modelo.logsout.getElement("angulo_no_lineal").Values;
    
    % === Agregar a la misma figura ===
    plot(angulo_lineal.Time, angulo_lineal.Data*180/pi, 'LineWidth', 1.5, ...
         'DisplayName', sprintf('Lineal %d°', i));
    plot(angulo_no_lineal.Time, angulo_no_lineal.Data*180/pi, '--', 'LineWidth', 1.5, ...
         'DisplayName', sprintf('No lineal %d°', i));
end

xlabel('Tiempo [s]');
ylabel('Ángulo [°]');
title('Comparación de modelo lineal y no lineal para diferentes ángulos');
legend('show', 'Location', 'bestoutside'); % 'bestoutside' para que quede afuera si hay muchos
grid on;
hold off;
