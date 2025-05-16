% === Parámetros del modelo del sistema ===
I  = 0.0167;    % Momento de inercia [kg·m²]
C  = -0.1326;   % Constante de torque [N·m]
Lm = 0.3310;    % Longitud desde el eje al motor [m]

% === Modelo del motor ===
m = 0.001586;       % Pendiente de la curva Fh = m*PWM + r
r = -1.597768;      % Intersección con eje fuerza
Kcorrex = 1.04;     % Corrección experimental

% === Rango de ángulos de equilibrio ===
angulos_deg = linspace(-20, 20, 200);       % en grados
angulos_rad = angulos_deg * (pi/180);       % en radianes

% === Cálculo de fuerza y PWM de equilibrio ===
fuerza_eq = -(C * cos(angulos_rad)) / Lm;
PWM_eq = Kcorrex * (fuerza_eq - r) / m;

% === Graficar ===
figure('Name','Relaciones de Equilibrio','Units','normalized','Position',[0.1 0.3 0.75 0.55]);

% Subplot 1: Fuerza vs PWM
subplot(2,1,1);
plot(PWM_eq, fuerza_eq, 'b-', 'LineWidth', 2);
xlabel('PWM_{equilibrio} [us]');
ylabel('Fuerza de equilibrio [N]');
title('Fuerza de Equilibrio vs PWM de Equilibrio');
grid on;

% Subplot 2: Ángulo vs PWM
subplot(2,1,2);
plot(PWM_eq, angulos_deg, 'r-', 'LineWidth', 2);
xlabel('PWM_{equilibrio} [us]');
ylabel('Ángulo de equilibrio [°]');
title('Ángulo de Equilibrio vs PWM de Equilibrio');
grid on;

