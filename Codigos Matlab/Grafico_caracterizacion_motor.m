clear all
clc

% Datos de la tabla
PWM = [1000, 1100, 1200, 1300, 1400, 1500, 1600, 1700, 1800, 1900, 2000];
Fuerza = [0.0, 0.147, 0.294, 0.405, 0.481, 0.463, 0.470, 0.439, 0.455, 0.456, 0.499];

% Realizar la regresión lineal
p = polyfit(PWM, Fuerza, 1);  % Ajuste lineal

% Generar valores ajustados
Fuerza_ajustada = polyval(p, PWM);

% Crear la gráfica
figure;
plot(PWM, Fuerza, 'o', 'MarkerFaceColor', 'b'); % Puntos experimentales
hold on;
plot(PWM, Fuerza_ajustada, 'r-', 'LineWidth', 2); % Línea de ajuste
grid on;

% Etiquetas y título
xlabel('PWM (microsegundos)');
ylabel('Fuerza (N)');
title('Relación entre PWM y Fuerza');

% Mostrar ecuación en consola
fprintf('Ecuación de la recta: Fuerza = %.4f * PWM + %.4f\n', p(1), p(2));
