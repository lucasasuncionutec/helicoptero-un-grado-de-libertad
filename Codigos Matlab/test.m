% --- Parámetros del controlador PID ---
Kp = 10.8703;
Ki = 0.5088;
Kd = 57.0171;
N  = 5.4539;

% --- Controlador continuo con filtro derivativo ---
s = tf('s');
controlador_continuo = tf([(Kp + Kd*N), (Kp*N + Ki), Ki*N], [1, N, 0]);

% --- Parámetros de simulación ---
T  = 0.1;     % Tiempo de muestreo
Tsim = 10;    % Tiempo total
cantidad_muestras = round(Tsim / T);
t = 0:T:(cantidad_muestras-1)*T;

% --- Entrada escalón unitario desde t = 1s ---
escalon = zeros(1, cantidad_muestras);
escalon(t >= 1) = 1;

% --- Respuesta del controlador continuo ---
[respuesta_continua, ~] = lsim(controlador_continuo, escalon, t);

% --- Discretización automática (ZOH) ---
controlador_discreto = c2d(controlador_continuo, T, 'zoh');
[respuesta_discreta, ~] = lsim(controlador_discreto, escalon, t);

% --- Discretización manual con ecuación de diferencias  ---
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

% --- Inicializar vectores ---
y_manual = zeros(1, cantidad_muestras);
x = escalon;

% --- Simulación controlador discretizado manualmente ---
for k = 4:cantidad_muestras
    y_manual(k) = (1/a3) * (a0*x(k) + a1*x(k-1) + a2*x(k-2) + ...
                          - a4*y_manual(k-1) - a5*y_manual(k-2));
end

% --- Mostrar las funciones de transferencia ---
disp('--- Función de transferencia del controlador discretizado por c2d (ZOH) ---');
disp(controlador_discreto);

disp('--- Función de transferencia del controlador discretizado manualmente ---');
disp(['(' num2str(a0*a3) ')z^2 + (' num2str(a1)*a3 ')z + (' num2str(a2)*a3 ') / z^2 + (' num2str(a4) ')z + (' num2str(a5) ')']);

% --- Graficar todas las respuestas ---
figure;
plot(t, respuesta_continua, 'g', 'LineWidth', 1.5); hold on;
stairs(t, respuesta_discreta, 'b', 'LineWidth', 1.5);
stairs(t, y_manual, 'r--', 'LineWidth', 1.5);
plot(t, escalon, 'k:', 'LineWidth', 1.5);

title('Respuesta al escalón del controlador PID');
xlabel('Tiempo [s]');
ylabel('u[k]');
legend('PID continuo', 'PID discreto (ZOH)', 'PID discreto manual', 'Entrada escalón');
grid on;
