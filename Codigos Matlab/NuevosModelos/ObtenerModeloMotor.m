clc;
clear;

% === Parámetros del modelo ===
tau = 0.593;        % Constante de tiempo
Ts = 0.1;           % Tiempo entre muestras
ventana_estacionario = 10;  % Últimos 10 puntos para promedio

% === Leer curvas suavizadas ===
T = readtable('CurvasSuavizadas.csv');
PWM_labels = T.Properties.VariableNames;

% === Eliminar la curva de PWM 1000 al final del experimento =========
% Solo queremos PWM_1000AlInicio (no Final)
labels_filtradas = {};
for i = 1:length(PWM_labels)
    nombre = PWM_labels{i};
    if contains(nombre, 'PWM_1000') && contains(nombre, 'Final')
        continue;  % saltar curva final
    end
    labels_filtradas{end+1} = nombre;  % mantener
end

% Actualizar variables
PWM_labels = labels_filtradas;
T = T(:, PWM_labels);  % reducir tabla solo a columnas deseadas


nCurvas = numel(PWM_labels);

PWM_values = NaN(1, nCurvas);
K_optimos  = NaN(1, nCurvas);

% === Extraer valores PWM desde nombres ===
for i = 1:nCurvas
    tokens = regexp(PWM_labels{i}, 'PWM_(\d+)', 'tokens');
    if ~isempty(tokens)
        PWM_values(i) = str2double(tokens{1}{1});
    end
end

% === PWM con recorte de 1.6 s ===
pwm_corte_1_6s = [1250, 1300];
muestras_1_6s = round(1.6 / Ts);
muestras_2s   = round(2.0 / Ts);

% === Crear figura para curvas y modelos ===
figure('Name', 'Modelos con K óptimos');
hold on; grid on;
xlabel('Tiempo [s]'); ylabel('Fuerza [N]');
title(sprintf('Modelos con \\tau = %.3f y K óptimo por valor estacionario', tau));
set(gca, 'FontSize', 12);
colores = lines(nCurvas);

% === Calcular K óptimo para cada curva ===
for i = 1:nCurvas
    curva = T.(PWM_labels{i});
    curva = curva(~isnan(curva));
    if isempty(curva), continue; end

    PWM_i = PWM_values(i);

    if ismember(PWM_i, pwm_corte_1_6s)
        curva = curva(muestras_1_6s+1:end);
    else
        curva = curva(muestras_2s+1:end);
    end

    if numel(curva) < 2, continue; end
    t_local = (0:length(curva)-1) * Ts;

    if length(curva) >= ventana_estacionario
        K_i = mean(curva(end - ventana_estacionario + 1:end));
    else
        K_i = curva(end);
    end
    K_optimos(i) = K_i;

    G_i = tf(K_i, [tau 1]);
    y_modelo = step(G_i, t_local);

    plot(t_local, curva, '-', 'Color', colores(i,:), 'LineWidth', 1.3);
    plot(t_local, y_modelo, '--', 'Color', colores(i,:), 'LineWidth', 1.3);
end

legend('off');

% === Promedio de K para cada PWM único ===
validos = ~isnan(K_optimos) & ~isnan(PWM_values);
PWM_valid = PWM_values(validos).';
K_valid   = K_optimos(validos).';

[PWM_unicos, ~, idx_grp] = unique(PWM_valid);
K_promedio = accumarray(idx_grp, K_valid, [], @mean);

% === Regresión forzada con origen en PWM = 1000 ===
PWM_base = 1000;
x_shift = PWM_unicos - PWM_base;
y = K_promedio;

m_forzado = (x_shift' * y) / (x_shift' * x_shift);

% === Mostrar resultados ===
fprintf('\n--- Regresión forzada con origen en PWM = %d ---\n', PWM_base);
for i = 1:numel(PWM_unicos)
    fprintf('PWM = %4d, K = %.4f\n', PWM_unicos(i), K_promedio(i));
end
fprintf('---------------------------------------------\n');
fprintf('Modelo: K = m * (PWM - %d)\n', PWM_base);
fprintf('m = %.6f\n', m_forzado);

% === Graficar dispersión y regresión ========================
figure('Name', 'Regresión: K = m·(PWM - 1000)');
hold on; grid on;
plot(PWM_unicos, K_promedio, 'o', 'MarkerSize', 6, ...
     'MarkerFaceColor', [0.2 0.6 0.8], 'DisplayName', 'K promedio');
PWM_rango = linspace(min(PWM_unicos), max(PWM_unicos), 100);
K_regresion = m_forzado * (PWM_rango - PWM_base);
plot(PWM_rango, K_regresion, '--r', 'LineWidth', 2, ...
     'DisplayName', sprintf('K = %.6f·(PWM - %d)', m_forzado, PWM_base));
xlabel('PWM [μs]');
ylabel('K óptimo promedio');
title('Regresión forzada: K = m·(PWM - 1000)', 'FontSize', 14);
legend('Location', 'northwest');
set(gca, 'FontSize', 11);

% === Comparación final: modelo con K de la regresión =================
figure('Name', 'Comparación: modelo con K ajustado por regresión');
hold on; grid on;
xlabel('Tiempo [s]');
ylabel('Fuerza [N]');
title('Modelos con K = m·(PWM - 1000)', 'FontSize', 14);
set(gca, 'FontSize', 12);

colores = lines(nCurvas);

for i = 1:nCurvas
    curva = T.(PWM_labels{i});
    curva = curva(~isnan(curva));
    if isempty(curva), continue; end

    PWM_i = PWM_values(i);
    if isnan(PWM_i), continue; end

    % Recorte según PWM
    if ismember(PWM_i, pwm_corte_1_6s)
        curva = curva(muestras_1_6s+1:end);
    else
        curva = curva(muestras_2s+1:end);
    end
    if numel(curva) < 2, continue; end

    t_local = (0:length(curva)-1) * Ts;

    % === Calcular K basado en la regresión ===
    K_reg = m_forzado * (PWM_i - PWM_base);
    G_reg = tf(K_reg, [tau 1]);
    y_modelo = step(G_reg, t_local);

    % === Graficar ===
    plot(t_local, curva, '-', 'Color', colores(i,:), ...
         'LineWidth', 1.3, 'DisplayName', sprintf('PWM %d - Exp', PWM_i));
    plot(t_local, y_modelo, '--', 'Color', colores(i,:), ...
         'LineWidth', 1.3, 'DisplayName', sprintf('PWM %d - Modelo', PWM_i));
end

legend('Location', 'eastoutside', 'FontSize', 9);

fprintf('\n--- Modelo en el dominio de Laplace ---\n');
fprintf('\n--- Modelo final del motor ---\n');
fprintf('Fuerza(s) = (PWM - 1000) · [ %.6f / (%.4f·s + 1) ]\n', m_forzado, tau);
fprintf('→ Entrada: PWM ∈ [1000, 2000] (µs)\n');
fprintf('→ Salida: Fuerza estimada [N], con F = 0 cuando PWM = 1000\n');

fprintf('\n--- Solución en el dominio del tiempo ---\n');
fprintf('Fuerza(t) = (PWM - 1000) · %.6f · [1 - exp(-t / %.4f)]\n', m_forzado, tau);
fprintf('→ Entrada: PWM constante (µs)\n');
fprintf('→ Salida: Fuerza [N] en función del tiempo\n');
fprintf('→ F(t → ∞) = (PWM - 1000) · %.6f\n', m_forzado);

