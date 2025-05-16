clc;
clear;
close all;

% === Leer archivo CSV ===
opts = detectImportOptions('DatosExperimentalesModeloMotor_comas.csv', 'Delimiter', ',');
data = readtable('DatosExperimentalesModeloMotor_comas.csv', opts);

% === Identificar columnas de fuerza ===
columnas = data.Properties.VariableNames;
indices_fuerza = [];
nombres_columnas = {};

for i = 1:length(columnas)
    if contains(columnas{i}, 'Fuerza')
        indices_fuerza = [indices_fuerza, i];
        nombres_columnas{end+1} = columnas{i};
    end
end

% === Agrupar curvas por valor de PWM ===
grupos = containers.Map('KeyType', 'double', 'ValueType', 'any');

for i = 1:length(indices_fuerza)
    nombre = nombres_columnas{i};
    tokens = regexp(nombre, '_(\d+)', 'tokens');
    
    if isempty(tokens) && contains(nombre, '1000')
        pwm_valor = 1000;
    elseif ~isempty(tokens)
        pwm_valor = str2double(tokens{1}{1});
    else
        continue;
    end

    % Leer y convertir los datos a número, invirtiendo el signo
    datos = -str2double(strrep(data{:, indices_fuerza(i)}, ',', '.'));
    datos = datos(~isnan(datos));

    % Agregar al grupo correspondiente (forma compatible con MATLAB)
    if isKey(grupos, pwm_valor)
        temp = grupos(pwm_valor);
        temp{end+1} = datos;
        grupos(pwm_valor) = temp;
    else
        grupos(pwm_valor) = {datos};
    end
end

% === Graficar curvas suavizadas promedio ===
figure;
hold on;
grid on;
grid minor;
title('Curvas suavizadas promedio por PWM', 'FontSize', 14);
xlabel('Tiempo [s]', 'FontSize', 12);  % ← actualizado
ylabel('Fuerza promedio [N]', 'FontSize', 12);
colormap(lines(length(grupos)));

% === Recorrer cada grupo (PWM) ===
claves_pwm = sort(cell2mat(keys(grupos)));
leyendas = {};

for k = 1:length(claves_pwm)
    pwm = claves_pwm(k);
    curvas = grupos(pwm);

    if length(curvas) < 2
        continue;  % Se requiere al menos 2 curvas para promediar
    end

    curva1 = curvas{1};
    curva2 = curvas{2};
    N = min(length(curva1), length(curva2));  % Igualar tamaños

    promedio = (curva1(1:N) + curva2(1:N)) / 2;

    t = (0:N-1) * 0.1;  % ← tiempo en segundos, 100 muestras en 10 s
    plot(t, promedio, 'LineWidth', 1.8);  % ← graficar contra tiempo

    leyendas{end+1} = sprintf('PWM %d µs', pwm);
end

legend(leyendas, 'Location', 'best', 'FontSize', 10);
set(gca, 'FontSize', 11);

% === Guardar resultados en archivo CSV ===

% Crear una estructura donde cada columna representa un PWM
max_len = 0;
curvas_promedio_map = containers.Map();

for k = 1:length(claves_pwm)
    pwm = claves_pwm(k);
    curvas = grupos(pwm);
    
    if length(curvas) < 2
        continue;
    end
    
    curva1 = curvas{1};
    curva2 = curvas{2};
    N = min(length(curva1), length(curva2));
    
    promedio = (curva1(1:N) + curva2(1:N)) / 2;
    curvas_promedio_map(sprintf('PWM_%d', pwm)) = promedio;
    
    if N > max_len
        max_len = N;
    end
end

% Armar tabla con relleno de NaN donde falten datos
nombres_columnas = keys(curvas_promedio_map);
T = table();
for i = 1:length(nombres_columnas)
    nombre = nombres_columnas{i};
    datos = curvas_promedio_map(nombre);
    datos(end+1:max_len) = NaN;  % Rellenar con NaN si es más corta
    T.(nombre) = datos(:);
end

% Guardar como CSV
writetable(T, 'CurvasSuavizadas.csv');
fprintf('Archivo "CurvasSuavizadas.csv" guardado correctamente.\n');
