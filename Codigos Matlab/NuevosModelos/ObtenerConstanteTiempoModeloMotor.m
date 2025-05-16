clc;
clear;

% === Leer los datos suavizados ===
T = readtable('CurvasSuavizadas.csv');
nombres_columnas = T.Properties.VariableNames;

% === Parámetro de tiempo entre muestras ===
Ts = 0.022;  % [s]
N = height(T);
tiempos = (0:N-1) * Ts;

% === Inicializar vector para constantes de tiempo ===
constantes_tiempo = NaN(1, length(nombres_columnas));
etiquetas = strings(1, length(nombres_columnas));

% === Calcular constante de tiempo por curva ===
for i = 1:length(nombres_columnas)
    curva = T.(nombres_columnas{i});
    curva = curva(~isnan(curva));
    etiquetas(i) = strrep(nombres_columnas{i}, 'PWM_', '');

    if isempty(curva)
        continue;
    end

    tiempo_local = (0:length(curva)-1) * Ts;
    valor_final = curva(end);
    objetivo = 0.632 * valor_final;

    idx = find(curva >= objetivo, 1, 'first');
    if isempty(idx), continue; end

    constantes_tiempo(i) = tiempo_local(idx);
end

% === Calcular promedio (ignorando NaN) ===
tau_promedio = mean(constantes_tiempo, 'omitnan');

% === Gráfico de barras ===
figure;
bar(constantes_tiempo, 'FaceColor', [0.2 0.6 0.8]);
hold on;
yline(tau_promedio, '--r', sprintf('\\tau_{prom} = %.3f s', tau_promedio), ...
    'LineWidth', 2, 'LabelHorizontalAlignment', 'left', 'FontSize', 11);
hold off;

title('Constantes de tiempo por PWM (\tau)', 'FontSize', 14);
xlabel('PWM [\mus]', 'FontSize', 12);
ylabel('Constante de tiempo \tau [s]', 'FontSize', 12);
xticks(1:length(etiquetas));
xticklabels(etiquetas);
xtickangle(45);
grid on;
set(gca, 'FontSize', 11);
