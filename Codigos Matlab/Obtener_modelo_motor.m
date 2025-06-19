clc;
clear;
close all;

% === Leer el archivo CSV corregido ===
opts = detectImportOptions('DatosExperimentalesModeloMotor_comas.csv', 'Delimiter', ',');
data = readtable('DatosExperimentalesModeloMotor_comas.csv', opts);

% === Definir PWM usados ===
PWM = [1050, 1050, 1100, 1100, 1150, 1150, 1200, 1200, 1250, 1250, ...
       1300, 1300, 1350, 1350, 1400, 1400, 1450, 1450, 1500, 1500, ...
       1550, 1550, 1600, 1600, 1650, 1650, 1700, 1700, 1750, 1750, ...
       1800, 1800, 1850, 1850, 1900, 1900, 1950, 1950, 2000, 2000];

% === Encontrar columnas de fuerza ===
columnas = data.Properties.VariableNames;
indices_fuerza = [];

for i = 1:length(columnas)
    if contains(columnas{i}, 'Fuerza')
        indices_fuerza = [indices_fuerza, i];
    end
end

% === Calcular fuerza promedio en estado estacionario ===
fuerza_promedio = zeros(length(indices_fuerza)-2,1);

for i = 3:length(indices_fuerza)-2
    fuerza_actual_texto = data{:, indices_fuerza(i)};
    fuerza_actual = -str2double(strrep(fuerza_actual_texto, ',', '.'));
    fuerza_actual = fuerza_actual(~isnan(fuerza_actual));
    
    if length(fuerza_actual) >= 10
        fuerza_estable = fuerza_actual(end-9:end);
    else
        fuerza_estable = fuerza_actual;
    end
    
    fuerza_promedio(i-2) = mean(fuerza_estable);
end

% === Eliminar outlier PWM = 2000 ===
PWM_filtrado = PWM(PWM ~= 2000);
fuerza_filtrada = fuerza_promedio(PWM ~= 2000);

% === Ajustar nueva recta PWM vs Fuerza ===
coeficientes = polyfit(PWM_filtrado', fuerza_filtrada, 1);
m = coeficientes(1);
r = coeficientes(2);

% === Ajuste automático de offset según PWM de equilibrio real ===
PWM_eq_real = 1320;
angulo_equilibrio = -20 * (pi/180);
C = -0.1326;
Lm = 0.3310;

fuerza_eq_teorica = -(C * cos(angulo_equilibrio)) / Lm;
fuerza_estimada_recta = m * PWM_eq_real + r;
offset = fuerza_eq_teorica - fuerza_estimada_recta;
r_con_offset = r + offset;

% === Calcular R^2 ===
fuerza_ajustada = polyval([m, r], PWM_filtrado);
SS_res = sum((fuerza_filtrada - fuerza_ajustada).^2);
SS_tot = sum((fuerza_filtrada - mean(fuerza_filtrada)).^2);
R2 = 1 - SS_res/SS_tot;

% === Mostrar resultados ===
fprintf('\n--- Resultado final ---\n');
fprintf('Ecuación original:\n');
fprintf('F_h = %.6f * PWM + (%.6f)\n', m, r);
fprintf('Ecuación con offset de %.2f:\n', offset);
fprintf('F_h = %.6f * PWM + (%.6f)\n', m, r_con_offset);
fprintf('R^2 del ajuste original: %.4f\n', R2);

% === Graficar ajuste ===
figure;
hold on;
grid on;
grid minor;
plot(PWM_filtrado, fuerza_filtrada, 'o', 'MarkerFaceColor', [0 0.4 0.8], ...
    'MarkerEdgeColor', [0 0.4 0.8], 'MarkerSize', 6);


PWM_largo = linspace(min(PWM_filtrado), max(PWM_filtrado), 100);
plot(PWM_largo, polyval([m, r], PWM_largo), '-', 'Color', [0.8 0 0], 'LineWidth', 2);
plot(PWM_largo, polyval([m, r_con_offset], PWM_largo), '--', 'Color', [0.2 0.6 0], 'LineWidth', 2);
xlabel('PWM [\mus]', 'FontSize', 12);
ylabel('Fuerza corregida [N]', 'FontSize', 12);
title('Modelo final del motor - Fuerza en función de PWM aplicado', 'FontSize', 14);
legend('Datos experimentales', 'Ajuste lineal', ...
       sprintf('Ajuste con offset = %.2f N', offset), ...
       'Location', 'northwest', 'FontSize', 10);
set(gca, 'FontSize', 11);
hold off;

% === Preparar datos para graficar en dos ventanas ===
pwms_detectados = [];
for i = 1:length(indices_fuerza)
    nombre = columnas{indices_fuerza(i)};
    tokens = regexp(nombre, '_(\d+)', 'tokens');
    if ~isempty(tokens)
        pwm_valor = str2double(tokens{end}{1});
        pwms_detectados = [pwms_detectados, pwm_valor];
    elseif contains(nombre, 'Experimento1000')
        pwms_detectados = [pwms_detectados, 1000];
    end
end
PWM_unicos = unique(pwms_detectados);
colores_base = lines(length(PWM_unicos));
PWM_a_etiquetar = [1000, 1300, 1350, 1500, 1700, 2000];

% === Ventana 1: Todas las curvas experimentales ===
figure('Name', 'Curvas experimentales por PWM', 'Units', 'normalized', 'Position', [0.1, 0.5, 0.8, 0.4]);
hold on;
grid on;
grid minor;
title('Datos experimentales medición fuerza motor en función de PWM', 'FontSize', 14);
xlabel('Tiempo [s]', 'FontSize', 12);
ylabel('Fuerza [N]', 'FontSize', 12);

datos_pwm_1400 = {};

for i = 1:length(indices_fuerza)
    fuerza_actual_texto = data{:, indices_fuerza(i)};
    fuerza_actual = -str2double(strrep(fuerza_actual_texto, ',', '.'));
    fuerza_actual = fuerza_actual(~isnan(fuerza_actual));
    if length(fuerza_actual) < 5, continue; end

    t_local = (0:length(fuerza_actual)-1) * 0.1;

    nombre_crudo = columnas{indices_fuerza(i)};
    if contains(nombre_crudo, 'Experimento1000AlInicio')
        pwm = 1000; experimento = 'Inicio'; marcador = 'o';
    elseif contains(nombre_crudo, 'Experimento1000AlFinal')
        pwm = 1000; experimento = 'Final'; marcador = '*';
    else
        tokens = regexp(nombre_crudo, 'EXP(\d+)_(\d+)', 'tokens');
        if ~isempty(tokens)
            experimento = tokens{1}{1};
            pwm = str2double(tokens{1}{2});
            marcador = 'none';
        else
            pwm = NaN; experimento = '?'; marcador = 'none';
        end
    end

    idx_pwm = find(PWM_unicos == pwm);
    color_base = colores_base(idx_pwm, :);
    if strcmp(experimento, '1') || strcmp(experimento, 'Inicio')
        color = min(color_base + 0.3, 1);
    elseif strcmp(experimento, '2') || strcmp(experimento, 'Final')
        color = color_base * 0.8;
    else
        color = color_base;
    end

    plot(t_local, fuerza_actual, '-', 'Color', color, 'LineWidth', 1.2, ...
        'Marker', marcador, 'MarkerSize', 6);

    % === Lógica de etiquetado ===
    etiquetar = false;
    if pwm == 1000 || pwm == 2000
        etiquetar = true;  % Mostrar ambos para 1000 y 2000
    elseif strcmp(experimento, '1')
        etiquetar = true;  % Solo el primero para los demás
    end

    if etiquetar
        x_etiqueta = t_local(end);
        y_etiqueta = fuerza_actual(end);
        texto_etiqueta = sprintf('PWM %d – EXP %s', pwm, experimento);
        text(x_etiqueta + 0.1, y_etiqueta, texto_etiqueta, ...
             'FontSize', 10, 'Color', color, 'VerticalAlignment', 'bottom');
    end

    % === Guardar para figura de PWM 1400 ===
    if pwm == 1400
        datos_pwm_1400{end+1} = struct('data', fuerza_actual, ...
            'nombre', sprintf('EXP %s – PWM %d µs', experimento, pwm), ...
            'color', color, 'tiempo', t_local);
    end
end

set(gca, 'FontSize', 11);


% === Ventana 2: Solo PWM = 1400 ===
figure('Name', 'Curvas PWM 1400', 'Units', 'normalized', 'Position', [0.2, 0.2, 0.7, 0.4]);
hold on;
grid on;
grid minor;
title('Respuesta completa obtenida de fuerza por un PWM de 1400 µs', 'FontSize', 14);
xlabel('Tiempo [s]', 'FontSize', 12);
ylabel('Fuerza [N]', 'FontSize', 12);

promedios_estacionarios = zeros(1, length(datos_pwm_1400));
for k = 1:length(datos_pwm_1400)
    curva = datos_pwm_1400{k};
    datos = curva.data;
    t_local = curva.tiempo;

    plot(t_local, datos, '-', 'Color', curva.color, ...
         'LineWidth', 2, 'DisplayName', curva.nombre);

    if length(datos) >= 10
        promedios_estacionarios(k) = mean(datos(end-9:end));
    else
        promedios_estacionarios(k) = mean(datos);
    end
end

promedio_global = mean(promedios_estacionarios);
xlim_actual = xlim;
plot(xlim_actual, [promedio_global, promedio_global], '--k', 'LineWidth', 1, ...
     'DisplayName', sprintf('Valor en estado estacionario: %.3f N', promedio_global));
legend('Location', 'best', 'FontSize', 10);
set(gca, 'FontSize', 11);
