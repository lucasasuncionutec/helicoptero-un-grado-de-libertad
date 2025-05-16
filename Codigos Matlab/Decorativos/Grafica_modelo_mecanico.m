clc;
clear;

% === Parámetros del sistema ===
I = 0.0167;
C = -0.1326;
Lm = 0.3310;
u = 0;  % Entrada constante

% === Condición inicial ===
angulo_inicial_grados = 30;
X0 = [angulo_inicial_grados * pi/180, 0];  % [posición, velocidad] en radianes

% === Intervalo de simulación ===
tspan = [0, 10];

% === Función del sistema dinámico ===
modelo = @(t, x) [
    x(2);
    (C*cos(x(1)) + u*Lm) * (1/I)
];

% === Resolver con ODE45 ===
[t, y] = ode45(modelo, tspan, X0);


% === Graficar de forma profesional ===
figure('Color', 'none');  % 'none' para fondo transparente al exportar

% Graficar solo la curva con color personalizado
plot(t, y(:,1)*180/pi, 'Color', [0.329, 0.769, 0.369], 'LineWidth', 2);
axis off;
box off;
axis tight;

% Guardar como PNG con fondo transparente
exportgraphics(gcf, 'curva_transparente.png', 'BackgroundColor', 'none', 'ContentType', 'image');
