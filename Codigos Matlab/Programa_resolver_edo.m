clc;
clear all;

% Definir la función del sistema dinámico
function sis = modelo_sistema(t,x,u,I,C,Lm)
    % Estados
    x1 = x(1);
    x2 = x(2);
    % Entradas
    Fh = u;
    % Ecuaciones diferenciales
    dx1 = x2;
    dx2 = (C*cos(x1)+Fh*Lm)*(1/I);

    sis = [dx1;dx2];
end


% Definir la función de eventos
function [value,isterminal,direction] = myEventsFcn(t,x,umbral_angulo)
    value = x(1) - umbral_angulo; % Detectar cuando x(1) - umbral_angulo = 0
    isterminal = 0; % No detener la integración
    direction = 0; % Detectar todos los cruces por cero
end

% Intervalo de tiempo
tspan = [0 10];

% Parámetros del sistema
I = 0.0167;
C = -0.1326;
Lm = 0.35;
u = 0; % Entrada constante

% Vector de ángulos en grados
angulos_grados = [0, 30, 45]; % Aquí puedes definir los ángulos que desees en grados

% Convertir los ángulos a radianes
angulos_radianes = angulos_grados * (pi / 180);

% Ángulo objetivo a alcanzar (puedes cambiar este valor para realizar diferentes pruebas)
angulo_objetivo = -54.5;  % Especificar el ángulo en grados
umbral_angulo = angulo_objetivo * (pi / 180);  % Convertir a radianes

% Colores para las gráficas
colores = lines(length(angulos_radianes));

% Inicializar la figura
figure;
hold on;

% Loop para diferentes condiciones iniciales (convertidos a radianes)
for i = 1:length(angulos_radianes)
    X0 = [angulos_radianes(i) 0];  % Condiciones iniciales [posición inicial en radianes, velocidad inicial]
    
    % Configurar opciones para la función de eventos
    options = odeset('Events', @(t,x)myEventsFcn(t,x,umbral_angulo));
    
    % Resolver la ecuación diferencial con detección de eventos
    [ts, y, te, ye, ie] = ode45(@(t,x)modelo_sistema(t,x,u,I,C,Lm), tspan, X0, options);
    
    % Graficar la posición angular en grados con su color correspondiente
    plot(ts, y(:,1)*57.2958, 'Color', colores(i,:), 'DisplayName', ['Posición inicial = ' num2str(angulos_grados(i)) '°']);
    
    % Verificar si se detectó el evento
    if ~isempty(te)
        tiempo_objetivo = te(1);
        disp(['El ángulo de ' num2str(angulo_objetivo) '° se alcanzó en ' num2str(tiempo_objetivo) ' segundos para la condición inicial de ' num2str(angulos_grados(i)) '°']);
        
        % Marcar en la gráfica el punto donde se alcanza el ángulo objetivo
        plot(tiempo_objetivo, ye(1)*57.2958, 'o', 'MarkerFaceColor', colores(i,:), 'MarkerEdgeColor', colores(i,:), ...
            'DisplayName', [num2str(angulo_objetivo) '° alcanzado (' num2str(angulos_grados(i)) '°)']);
    else
        disp(['El ángulo de ' num2str(angulo_objetivo) '° no se alcanzó para la condición inicial de ' num2str(angulos_grados(i)) '°']);
    end
end

% Añadir título y etiquetas
title('Dinámica del sistema (modelo)');
xlabel('Tiempo (s)');
ylabel('Posición angular (°)');

% Añadir leyenda
legend show;

hold off;


