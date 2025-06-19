clc;
clear;

% Definir la función de entrada escalonada
u_t = @(t) (t < 2.7469)*0.4 + (t >= 2.7469)*0.1;

% Parámetros del sistema
I = 0.0167;
C = -0.1326;
Lm = 0.3310;

tspan = [0 6];
angulos_grados = [-20];
angulos_radianes = angulos_grados * (pi / 180);
angulo_objetivo = 20;
umbral_angulo = angulo_objetivo * (pi / 180);
colores = lines(length(angulos_radianes));

% Crear figura con dos subplots
figure;

% Subplot 1: Ángulo
subplot(2,1,1);
hold on;
for i = 1:length(angulos_radianes)
    X0 = [angulos_radianes(i), 0];
    options = odeset('Events', @(t,x)myEventsFcn(t,x,umbral_angulo));
    [ts, y, te, ye, ie] = ode45(@(t,x)modelo_sistema(t,x,u_t,I,C,Lm), tspan, X0, options);

    plot(ts, y(:,1)*180/pi, 'Color', colores(i,:), 'DisplayName', ['Posición inicial = ' num2str(angulos_grados(i)) '°']);

    if ~isempty(te)
        tiempo_objetivo = te(1);
        disp(['Ángulo alcanzado en ' num2str(tiempo_objetivo) ' s desde ' num2str(angulos_grados(i)) '°']);
        plot(tiempo_objetivo, ye(1)*180/pi, 'o', 'MarkerFaceColor', colores(i,:), 'MarkerEdgeColor', colores(i,:), ...
            'DisplayName', [num2str(angulo_objetivo) '° alcanzado (' num2str(angulos_grados(i)) '°)']);
    end
end
title('Ángulo del sistema (°)');
ylabel('Ángulo (°)');
legend show;
grid on;

% Subplot 2: Entrada de fuerza
subplot(2,1,2);
fplot(u_t, tspan, 'k', 'LineWidth', 2);
title('Fuerza aplicada F_h(t)');
xlabel('Tiempo (s)');
ylabel('Fuerza (N)');
grid on;
ylim([0.3 0.45]);

% ========= FUNCIONES INTERNAS ==========

function sis = modelo_sistema(t,x,u_fun,I,C,Lm)
    x1 = x(1); x2 = x(2);
    Fh = u_fun(t);
    dx1 = x2;
    dx2 = (C*cos(x1) + Fh*Lm)*(1/I);
    sis = [dx1; dx2];
end

function [value,isterminal,direction] = myEventsFcn(t,x,umbral_angulo)
    value = x(1) - umbral_angulo;
    isterminal = 0;
    direction = 0;
end
