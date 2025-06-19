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

% === RANGO DE FRICCIÓN VISCOSA (mu_d) ===
mu_d_inicio = 0.0;
mu_d_final  = 0.1;
mu_d_step   = 0.01;
mu_d_vals   = mu_d_inicio:mu_d_step:mu_d_final;

% === CONFIGURACIÓN DEL RECORTE DEL EXPERIMENTO ===
fila_inicio = 2;  % Empezar a leer desde esta fila

% === FIGURA INICIAL ===
figure;
hold on;
colormap_actual = parula(length(mu_d_vals));
leyendas = strings(1, length(mu_d_vals));

% === SIMULACIÓN CON DIFERENTES mu_d ===
for k = 1:length(mu_d_vals)
    mu_d = mu_d_vals(k);
    color = colormap_actual(k, :);

    [ts, y] = ode45(@(t,x)modelo_sistema_friccion(t,x,u,I,C,Lm,mu_d), tspan, X0);
    plot(ts, y(:,1)*180/pi, 'Color', color, 'LineWidth', 1.3);
    leyendas(k) = sprintf('Modelo \\mu_d = %.2f', mu_d);
end

% === ARCHIVOS EXPERIMENTALES A CARGAR ===
archivos_exp = {
    'fricciondinamica/experimento1.csv', 
    'fricciondinamica/experimento2.csv', 
    'fricciondinamica/experimento3.csv'
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
title('Modelos con fricción dinámica: comparación con experimentos');
xlabel('Tiempo [s]');
ylabel('Ángulo [°]');
legend(leyendas, 'Location', 'best');
grid on;
hold off;

% === FUNCIÓN DEL MODELO CON FRICCIÓN VISCOSA ===
function sis = modelo_sistema_friccion(t, x, u, I, C, Lm, mu_d)
    x1 = x(1); x2 = x(2);
    Fh = u;
    dx1 = x2;
    dx2 = (C*cos(x1) + Fh*Lm - mu_d*x2) * (1/I);
    sis = [dx1; dx2];
end
