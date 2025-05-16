clc
clear all

% Configurar MATLAB para mostrar más decimales
format long;

% Datos
pwm = [1000, 1100, 1200, 1300, 1400, 1500, 1600, 1700, 1800, 1900, 2000];
fuerza = [0, 0.092, 0.203, 0.294, 0.351, 0.391, 0.375, 0.369, 0.366, 0.373, 0.373];

% Seleccionar los primeros 6 datos
pwm_subset = pwm(1:6)';
fuerza_subset = fuerza(1:6)';

% Ajustar PWM para que pase por (1000, 0)
pwm_adjusted = pwm_subset - 1000;

% Realizar la regresión lineal forzada
pendiente = (pwm_adjusted' * fuerza_subset) / (pwm_adjusted' * pwm_adjusted);
intercepto = -pendiente * 1000;

% Mostrar la ecuación de la recta
fprintf('La ecuación de la recta es: fuerza = %.10f * pwm + %.10f\n', pendiente, intercepto);

% Crear la gráfica
figure;
plot(pwm, fuerza, '-o', 'LineWidth', 1.5, 'MarkerSize', 6); % Datos originales
hold on;
plot(pwm_subset, pendiente * pwm_subset + intercepto, '-r', 'LineWidth', 1.5); % Línea de ajuste
grid on;
title('Relación entre PWM y Fuerza del Motor (Forzada a pasar por 1000,0)', 'FontSize', 14);
xlabel('PWM (\mus)', 'FontSize', 12);
ylabel('Fuerza (N)', 'FontSize', 12);
legend('Fuerza vs PWM', 'Ajuste Lineal (Primeros 6 Datos)', 'Location', 'best');
set(gca, 'FontSize', 10); % Ajustar tamaño de la fuente
hold off;
