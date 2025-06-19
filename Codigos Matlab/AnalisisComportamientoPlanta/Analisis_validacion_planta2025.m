clc;
clear all;

% === PARÁMETROS DEL SISTEMA ===
I = 0.0167;
C = -0.1326;
Lm = 0.3310;
u = 0;                          % Entrada constante
tspan = [0 10];                 % Tiempo de simulación
angulo_inicial = 0;            % En grados
X0 = [angulo_inicial * pi/180, 0];  % [posición inicial en rad, velocidad inicial]

% === CONFIGURACIÓN DEL RECORTE DEL EXPERIMENTO ===
fila_inicio = 2;  % Empezar a leer desde esta fila

% === FIGURA INICIAL ===
figure;
hold on;
leyendas = strings(0);
color_modelo = [0.1 0.5 0.8];

% === SIMULACIÓN DEL MODELO SIN FRICCIÓN ===
[ts, y] = ode45(@(t,x)modelo_sistema_sin_friccion(t,x,u,I,C,Lm), tspan, X0);
plot(ts, y(:,1)*180/pi, 'Color', color_modelo, 'LineWidth', 2);
leyendas(end+1) = "Modelo sin fricción";

% === ARCHIVOS EXPERIMENTALES A CARGAR ===
archivos_exp = {
    'experimentos/experimento1.csv', 
    'experimentos/experimento2.csv', 
    'experimentos/experimento3.csv'
};
estilos_exp = {'k--', 'r--', 'b--'};  % estilos diferentes para distinguirlos

% === CARGA Y GRAFICADO DE EXPERIMENTOS ===
for i = 1:length(archivos_exp)
    archivo = archivos_exp{i};

    if isfile(archivo)
        datos_exp = readtable(archivo);
        datos_recortados = datos_exp(fila_inicio:end, :);

        % Reiniciar tiempo desde cero
        t_exp = datos_recortados.tiempo_s - datos_recortados.tiempo_s(1);
        y_exp = datos_recortados.angulo_real_deg;

        % Graficar experimento
        plot(t_exp, y_exp, estilos_exp{i}, 'LineWidth', 2);
        leyendas(end+1) = sprintf('Experimento %d', i);
    else
        warning('Archivo %s no encontrado.', archivo);
    end
end

% === AJUSTES FINALES DE LA GRÁFICA ===
title('Modelo sin fricción vs. experimentos reales');
xlabel('Tiempo [s]');
ylabel('Ángulo [°]');
legend(leyendas, 'Location', 'best');
grid on;
hold off;

% === FUNCIÓN DEL MODELO SIN FRICCIÓN ===
function sis = modelo_sistema_sin_friccion(t, x, u, I, C, Lm)
    x1 = x(1); x2 = x(2);
    Fh = u;
    dx1 = x2;
    dx2 = (C*cos(x1) + Fh*Lm) * (1/I);
    sis = [dx1; dx2];
end
