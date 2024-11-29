clear all
clc

% Leer el archivo CSV
data = readtable('validacion_planta.csv');  % Asegúrate de que el archivo esté en el directorio de trabajo

% Extraer las columnas correspondientes a cada experimento
time1 = data.Tiempo_1 / 1000;  % Convertir a segundos
angle1 = data.Angulo_1;
time2 = data.Tiempo_2 / 1000;  % Convertir a segundos
angle2 = data.Angulo_2;
time3 = data.Tiempo_3 / 1000;  % Convertir a segundos
angle3 = data.Angulo_3;

% Crear figura
figure;

% Configurar gráficos para cada experimento
tiledlayout(3, 1);

% Gráfico del Experimento 1
nexttile;
plot(time1, angle1, 'r', 'LineWidth', 1.5);
grid on;
title('Experimento 1', 'FontSize', 14);
xlabel('Tiempo (s)', 'FontSize', 12);
ylabel('Ángulo (°)', 'FontSize', 12);
set(gca, 'FontSize', 12, 'LineWidth', 1.5);

% Gráfico del Experimento 2
nexttile;
plot(time2, angle2, 'g', 'LineWidth', 1.5);
grid on;
title('Experimento 2', 'FontSize', 14);
xlabel('Tiempo (s)', 'FontSize', 12);
ylabel('Ángulo (°)', 'FontSize', 12);
set(gca, 'FontSize', 12, 'LineWidth', 1.5);

% Gráfico del Experimento 3
nexttile;
plot(time3, angle3, 'b', 'LineWidth', 1.5);
grid on;
title('Experimento 3', 'FontSize', 14);
xlabel('Tiempo (s)', 'FontSize', 12);
ylabel('Ángulo (°)', 'FontSize', 12);
set(gca, 'FontSize', 12, 'LineWidth', 1.5);
