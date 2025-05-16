clear all
clc

% Leer el archivo CSV
data = readtable('validacion_planta_recortado.csv');  % Asegúrate de que el archivo esté en el directorio de trabajo

% Extraer las columnas correspondientes a cada experimento
time1 = data.Tiempo_1 / 1000;  % Convertir a segundos
angle1 = data.Angulo_1;
time2 = data.Tiempo_2 / 1000;  % Convertir a segundos
angle2 = data.Angulo_2;
time3 = data.Tiempo_3 / 1000;  % Convertir a segundos
angle3 = data.Angulo_3;

% Suavizar los datos usando un ajuste de spline
time1_smooth = linspace(min(time1), max(time1), 500);
angle1_smooth = spline(time1, angle1, time1_smooth);

time2_smooth = linspace(min(time2), max(time2), 500);
angle2_smooth = spline(time2, angle2, time2_smooth);

time3_smooth = linspace(min(time3), max(time3), 500);
angle3_smooth = spline(time3, angle3, time3_smooth);

% Crear figura
figure;

% Configurar gráficos para cada experimento
tiledlayout(3, 1);

% Gráfico del Experimento 1
nexttile;
plot(time1_smooth, angle1_smooth, 'r', 'LineWidth', 1.5);
grid on;
title('Experimento 1, Ángulo Inicial: 0°', 'FontSize', 14);
xlabel('Tiempo (s)', 'FontSize', 12);
ylabel('Ángulo (°)', 'FontSize', 12);
set(gca, 'FontSize', 12, 'LineWidth', 1.5);

% Gráfico del Experimento 2
nexttile;
plot(time2_smooth, angle2_smooth, 'g', 'LineWidth', 1.5);
grid on;
title('Experimento 2, Ángulo Inicial: 30°', 'FontSize', 14);
xlabel('Tiempo (s)', 'FontSize', 12);
ylabel('Ángulo (°)', 'FontSize', 12);
set(gca, 'FontSize', 12, 'LineWidth', 1.5);

% Gráfico del Experimento 3
nexttile;
plot(time3_smooth, angle3_smooth, 'b', 'LineWidth', 1.5);
grid on;
title('Experimento 3, Ángulo Inicial: 45°', 'FontSize', 14);
xlabel('Tiempo (s)', 'FontSize', 12);
ylabel('Ángulo (°)', 'FontSize', 12);
set(gca, 'FontSize', 12, 'LineWidth', 1.5);
