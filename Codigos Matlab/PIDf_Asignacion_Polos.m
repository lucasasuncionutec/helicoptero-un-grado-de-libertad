clear; clc; close all;

%% === Parámetros del modelo del sistema ===
% Constantes del modelo físico
I  = 0.0167;  % Momento de inercia
C  = -0.1326; % Constante relacionada al torque
Lm = 0.3310;  % Longitud desde el eje hasta el motor     
angulo_equilibrio = 0 * (pi/180);  % Ángulo de equilibrio en radianes para linealización
A = Lm / I;
B = C * sin(angulo_equilibrio) / I;

% Modelo del motor brushless
m = 0.001586;                   

% Referencia(En este caso 1 para evitar una salida nula y poder analisar correctamente la gráfica, en 
% la práctica se utiliza el mismo valor que el angulo de equilibrio)
ref = 1;  

%% === Modelo Planta continua ===
planta_continua = tf([m*A], [1 0 B]);

%% === Cálculo parámetros del controlador PIDf por asignación de polos ===
% Se deben elegir sobrepico(Mp) en porcentaje y tiempo de
% establecimiento(Tss) en segundos
Mp = 0.2;
Tss = 8;

% Calculo del coeficiente de amortiguamiento y frecuencia natural
zeta = -log(100/Mp)/(sqrt(pi^2 + (log(100/Mp))^2));
omega_n = 4/(zeta*Tss);

% Calculo de polos insignificantes(p1 y p2) y variables auxiliares(alphas)
p1 = 10*zeta*omega_n;
p2 = 12*zeta*omega_n;

alpha1 = p1 + p2 + 2*zeta*omega_n;
alpha2 = p1*p2 + 2*zeta*omega_n*(p1 + p2) + omega_n^2;
alpha3 = 2*zeta*omega_n*p1*p2 + (omega_n^2)*(p1 + p2);
alpha4 = (omega_n^2)*p1*p2;


% Calculo parametros PIDf
Kp = (alpha3-alpha1*B-(alpha4/alpha1))/(alpha1*m*A);
Ki = (alpha4)/(alpha1*m*A);
Kd = (alpha2 - B - Kp*m*A)/(alpha1*m*A);
N = alpha1;

Kp = 5.9067;
Ki = 0.2765;
Kd = 30.9819;
N = 5.45;

%% === Parámetros de simulación ===
T    = 0.1;                      % Tiempo de muestreo [s]
Tsim = 20;                       % Duración total de la simulación [s]
cantidad_muestras = round(Tsim / T);
t = 0:T:(cantidad_muestras-1)*T;

%% === Entrada de referencia constante ===
entrada = ref * ones(1, cantidad_muestras);

%% === Controlador discreto manual (ecuaciones en diferencias) ===
a = Kp + Kd*N;
b = Kp*N + Ki;
c = Ki*N;
d = N;

K1 = (b*d - c)/(d^2);
K2 = c/d;
K3 = (a*d^2 - b*d + c)/(d^2);

a0 = K1 + K3;
a1 = -K1 - K1*exp(-d*T) + K2*T - 2*K3;
a2 = K1*exp(-d*T) - K2*T*exp(-d*T) + K3;
a3 =  1;
a4 = -exp(-d*T) - 1;
a5 = exp(-d*T);

%% === Discretización de la planta con ZOH ===
planta_discreta = c2d(planta_continua, T, 'zoh');
[num_p, den_p] = tfdata(planta_discreta, 'v');
a1p = num_p(2);
a2p = num_p(3); 
b1p = den_p(1);
b2p = den_p(2); 
b3p = den_p(3);

%% === Inicialización de señales ===
y_plant_manual = zeros(1, cantidad_muestras);  % Salida de la planta
u_manual       = zeros(1, cantidad_muestras);  % Señal de control
error          = zeros(1, cantidad_muestras);  % Error entre referencia y salida

% Perturbación (opcional, puede modificarse o eliminarse)
perturbacion = zeros(1, cantidad_muestras);
perturbacion(t <= 1) = 1;

% Variables para valores anteriores (memoria del sistema)
errorm1 = 0; errorm2 = 0;
ukm1 = 0; ukm2 = 0;
ykm1 = 0; ykm2 = 0;

%% === Simulación con controlador discreto manual ===
for k = 1:cantidad_muestras
    % Cálculo del error
    error(k) = ref - ykm1;

    % Ecuación en diferencias del controlador
    u_manual(k) = (1/a3)*(a0*error(k) + a1*errorm1 + a2*errorm2 ...
                   - a4*ukm1 - a5*ukm2);

    % Ecuación en diferencias de la planta
    y_plant_manual(k) = (1/b1p)*(a1p * ukm1 + a2p * ukm2 ...
                          - b2p * ykm1 - b3p * ykm2);

    % Actualizar memorias
    errorm2 = errorm1; errorm1 = error(k);
    ukm2 = ukm1;       ukm1 = u_manual(k);
    ykm2 = ykm1;       ykm1 = y_plant_manual(k);
end


%% === Gráfica comparativa de respuestas ===
figure('Name', 'Comparación PID manual vs Simulink', 'NumberTitle', 'off');

% Respuesta del sistema
subplot(2,1,1);
stairs(t, y_plant_manual, 'b', 'LineWidth', 1.8);
title('Respuesta de la planta con PID: Discretización manual vs Simulink');
xlabel('Tiempo [s]');
ylabel('Salida del sistema');
legend('Respuesta PID Discreto', 'Respuesta Simulink');
grid on;
ylim([0, max([y_plant_manual]) + 0.2]);

% Error en el tiempo
subplot(2,1,2);
plot(t, error, 'r', 'LineWidth', 1.5);
title('Evolución del error de seguimiento');
xlabel('Tiempo [s]');
ylabel('Error = referencia - salida');
grid on;

%% === Función de transferencia del controlador PIDf ===
s = tf('s');
PIDf = ((s^2)*(Kp+Kd*N)+ s*(Kp*N+Ki) + Ki*N)/(s^2+s*N);

%% === Sistema en malla cerrada ===
G_OL = PIDf * planta_continua;           % Lazo abierto
G_CL = feedback(G_OL, 1);                % Malla cerrada unitaria

%% === Gráfica de polos y ceros ===
K = 1;
G_deseada = (K * (omega_n / 2)) / ((s^2 + 2*zeta*omega_n*s + omega_n^2)*(s + p1)*(s + p2));

% Obtener polos y ceros
[p_cl, z_cl] = pzmap(G_CL);
[p_des, z_des] = pzmap(G_deseada);

% Crear nueva figura
figure('Name', 'Comparación Polos y Ceros Mejorada', 'NumberTitle', 'off'); hold on;

% === Graficar polos sobrepuestos con doble marcador ===
plot(real(p_cl), imag(p_cl), 'kx', 'MarkerSize', 14, 'LineWidth', 4); % fondo negro
plot(real(p_cl), imag(p_cl), 'rx', 'MarkerSize', 10, 'LineWidth', 2); % rojo encima

% === Graficar ceros de G_CL ===
plot(real(z_cl), imag(z_cl), 'ro', 'MarkerSize', 12, 'LineWidth', 2);

% === Puntos ficticios para leyenda ===
h1 = plot(NaN, NaN, 'kx', 'MarkerSize', 14, 'LineWidth', 4); % fondo negro
h2 = plot(NaN, NaN, 'rx', 'MarkerSize', 10, 'LineWidth', 2); % encima rojo
h3 = plot(NaN, NaN, 'ro', 'MarkerSize', 12, 'LineWidth', 2); % ceros

% === Leyenda explicativa ===
legend([h1 h3 h2], ...
       {'Polos por Asignación de Polos',  'Ceros del sistema original que se mantienen','Polos sistema deseado'}, ...
       'Location', 'best', 'FontSize', 12);


% Estilo del gráfico
title({ ...
    'Comparación de polos y ceros del sistema real', ...
    'luego de la asignación de polos y el sistema deseado'}, ...
    'FontSize', 14);

xlabel('Parte Real', 'FontSize', 12);
ylabel('Parte Imaginaria', 'FontSize', 12);
grid on;
axis equal;
xline(0, '--k', 'LineWidth', 1.5, 'HandleVisibility','off');
yline(0, '--k', 'LineWidth', 1.5, 'HandleVisibility','off');


