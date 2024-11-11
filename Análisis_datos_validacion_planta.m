% Leer el archivo CSV
data = readtable('validacion_planta.csv');  % Asegúrate de que el nombre del archivo sea correcto

% Extraer las columnas correspondientes a cada experimento
time1 = data.Time1 / 1000;  % Convertir a segundos
angle1 = data.Angle1;
time2 = data.Time2 / 1000;  % Convertir a segundos
angle2 = data.Angle2;
time3 = data.Time3 / 1000;  % Convertir a segundos
angle3 = data.Angle3;
time4 = data.Time4 / 1000;  % Convertir a segundos
angle4 = data.Angle4;

% Ángulos iniciales y ángulo final
initial_angles = [0, 30, 60];  % Ángulos iniciales a evaluar
final_angle = -61;   % Ángulo final

% Función para calcular el tiempo de transición
function [start_time, end_time] = calculate_transition_time(time, angle, initial_angle, final_angle)
    % Encuentra el índice del ángulo inicial más cercano
    [~, start_idx] = min(abs(angle - initial_angle));
    start_time = time(start_idx);

    % Encuentra el índice del ángulo final más cercano a -61 grados
    [~, end_idx] = min(abs(angle - final_angle));
    end_time = time(end_idx);
end

% Calcular tiempos de transición para cada ángulo inicial y experimento
for j = 1:length(initial_angles)
    initial_angle = initial_angles(j);
    
    % Experimento 1
    [start_time1, end_time1] = calculate_transition_time(time1, angle1, initial_angle, final_angle);
    duration1 = end_time1 - start_time1;
    disp(['Duración del experimento 1 desde ', num2str(initial_angle), '° a -61°: ', num2str(duration1), ' segundos']);
    
    % Experimento 2
    [start_time2, end_time2] = calculate_transition_time(time2, angle2, initial_angle, final_angle);
    duration2 = end_time2 - start_time2;
    disp(['Duración del experimento 2 desde ', num2str(initial_angle), '° a -61°: ', num2str(duration2), ' segundos']);
    
    % Experimento 3
    [start_time3, end_time3] = calculate_transition_time(time3, angle3, initial_angle, final_angle);
    duration3 = end_time3 - start_time3;
    disp(['Duración del experimento 3 desde ', num2str(initial_angle), '° a -61°: ', num2str(duration3), ' segundos']);
    
    % Experimento 4
    [start_time4, end_time4] = calculate_transition_time(time4, angle4, initial_angle, final_angle);
    duration4 = end_time4 - start_time4;
    disp(['Duración del experimento 4 desde ', num2str(initial_angle), '° a -61°: ', num2str(duration4), ' segundos']);
end
