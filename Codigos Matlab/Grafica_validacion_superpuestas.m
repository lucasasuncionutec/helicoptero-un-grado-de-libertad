% Leer el archivo CSV
data = readtable('validacion_planta_recortado.csv');  % Asegúrate de que el archivo esté en el directorio de trabajo

% Extraer las columnas correspondientes a cada experimento
time1 = data.Tiempo_1 / 1000;  % Convertir a segundos
angle1 = data.Angulo_1;
time2 = data.Tiempo_2 / 1000;  % Convertir a segundos
angle2 = data.Angulo_2;
time3 = data.Tiempo_3 / 1000;  % Convertir a segundos
angle3 = data.Angulo_3;

% Suavizar los datos del experimento usando un ajuste de spline
time1_smooth = linspace(min(time1), max(time1), 500);
angle1_smooth = spline(time1, angle1, time1_smooth);

time2_smooth = linspace(min(time2), max(time2), 500);
angle2_smooth = spline(time2, angle2, time2_smooth);

time3_smooth = linspace(min(time3), max(time3), 500);
angle3_smooth = spline(time3, angle3, time3_smooth);

% Parámetros del sistema para el modelo teórico
I = 0.0167;
C = -0.1326;
Lm = 0.35;
u = 0; % Entrada constante

% Condiciones iniciales
angulos_grados = [0, 30, 45]; % Ángulos iniciales en grados
angulos_radianes = angulos_grados * (pi / 180); % Convertir a radianes

% Ángulo objetivo para detectar el cruce (en grados)
angulo_objetivo = -54.5; % Cambia este valor según sea necesario
umbral_angulo = angulo_objetivo * (pi / 180); % Convertir a radianes

tspan = [0 max([max(time1), max(time2), max(time3)])]; % Intervalo de tiempo

% Crear figura
figure;
tiledlayout(3, 1);

% Loop para graficar cada experimento
for i = 1:3
    % Condiciones iniciales para el modelo teórico
    X0 = [angulos_radianes(i), 0];

    % Resolver la ecuación diferencial del modelo teórico
    [t_modelo, y_modelo] = ode45(@(t, x) modelo_sistema(t, x, u, I, C, Lm), tspan, X0);

    % Detectar el cruce por el ángulo objetivo
    cruzar_objetivo = find(y_modelo(:, 1) <= umbral_angulo, 1);
    if ~isempty(cruzar_objetivo)
        tiempo_cruce = t_modelo(cruzar_objetivo);
        angulo_cruce = y_modelo(cruzar_objetivo, 1) * 57.2958; % Convertir a grados
    else
        tiempo_cruce = NaN;
        angulo_cruce = NaN;
    end

    % Seleccionar los datos experimentales
    if i == 1
        t_experimento = time1_smooth;
        angle_experimento = angle1_smooth;
    elseif i == 2
        t_experimento = time2_smooth;
        angle_experimento = angle2_smooth;
    else
        t_experimento = time3_smooth;
        angle_experimento = angle3_smooth;
    end

    % Crear subgráfica
    nexttile;
    hold on;

    % Graficar modelo teórico
    plot(t_modelo, y_modelo(:, 1) * 57.2958, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Modelo Teórico');

    % Marcar el punto de cruce en el modelo teórico
    if ~isnan(tiempo_cruce)
        plot(tiempo_cruce, angulo_cruce, 'ro', 'MarkerFaceColor', 'r', 'DisplayName', ['Cruce en ' num2str(angulo_objetivo) '°']);
    end

    % Graficar datos experimentales
    plot(t_experimento, angle_experimento, 'b', 'LineWidth', 1.5, 'DisplayName', 'Datos Experimentales');

    % Configuración de la gráfica
    grid on;
    title(['Experimento ' num2str(i) ', Ángulo Inicial: ' num2str(angulos_grados(i)) '°'], 'FontSize', 14);
    xlabel('Tiempo (s)', 'FontSize', 12);
    ylabel('Ángulo (°)', 'FontSize', 12);
    legend('show', 'Location', 'best');
    set(gca, 'FontSize', 12, 'LineWidth', 1.5);

    hold off;
end

% Definir la función del sistema dinámico
function sis = modelo_sistema(t, x, u, I, C, Lm)
    % Estados
    x1 = x(1);
    x2 = x(2);
    % Entradas
    Fh = u;
    % Ecuaciones diferenciales
    dx1 = x2;
    dx2 = (C * cos(x1) + Fh * Lm) * (1 / I);

    sis = [dx1; dx2];
end
