%% === Comparación modelo no lineal vs linealizado (θ absoluto en ambos) ===
% Lucas E. A. Lemos – 30-abr-2025
clear; clc; close all;

% ---------- Parámetros físicos ----------
I  = 0.0167;         % [kg·m²]
C  = -0.1326;        % [N·m]
Lm = 0.3310;         % [m]

% ---------- Ángulos de equilibrio (°) ----------
theta_eq_deg = [0, 15];
theta_eq_rad = deg2rad(theta_eq_deg);

% ---------- Fuerzas aplicadas absolutas (N) ----------
F_aplicadas = [-0.3, 0.6];

% ---------- Simulación ----------
u_step = @(t) double(t >= 0);
t_span = [0 5];  % segundos

% ---------- Figura ----------
nTheta = numel(theta_eq_rad);
nForce = numel(F_aplicadas);
figure('Units','normalized','Position',[0.05 0.1 0.88 0.8])
tiledlayout(nTheta, nForce, 'TileSpacing','compact', 'Padding','compact')

for i = 1:nTheta
    theta_eq = theta_eq_rad(i);                       
    F_eq     = -C * cos(theta_eq) / Lm;               

    for j = 1:nForce
        F_aplicada = F_aplicadas(j);                  
        Fh         = @(t) F_aplicada * u_step(t);     
        DeltaFh    = @(t) Fh(t) - F_eq;               

        % === Modelo NO lineal ===
        fNL = @(t,x)[
            x(2);
            (C * cos(x(1)) + Lm * Fh(t)) / I ];
        x0_nl = [theta_eq; 0];
        [t_nl, x_nl] = ode45(fNL, t_span, x0_nl);
        theta_nl = x_nl(:,1);  

        % === Modelo linealizado ===
        A = -C * sin(theta_eq) / I;
        B =  Lm / I;
        fLIN = @(t,x)[
            x(2);
            A * x(1) + B * DeltaFh(t)];
        x0_lin = [0; 0];
        [t_lin, x_lin] = ode45(fLIN, t_span, x0_lin);
        theta_lin = theta_eq + x_lin(:,1);  

        % === Gráfica ===
        nexttile((i-1)*nForce + j)
        plot(t_nl,  rad2deg(theta_nl), 'b', 'LineWidth', 1.5); hold on
        plot(t_lin, rad2deg(theta_lin), '--r', 'LineWidth', 1.5);
        grid on; xlabel('t [s]')
        xlim(t_span)
        ylim([-200 60])

        % === Mostrar textos: θ_eq y F_eq ===
        text(0.1, 44+(j-1)*200, sprintf('\\theta_{eq} = %.1f°', theta_eq_deg(i)), ...
            'FontSize', 10, 'Color', [0.1 0.1 0.6], 'FontWeight', 'bold')
        text(0.1, 25+(j-1)*200, sprintf('F_{eq} = %.2f N', F_eq), ...
            'FontSize', 10, 'Color', [0.4 0.1 0.1], 'FontWeight', 'bold')

        % === Títulos y etiquetas ===
        if i == 1
            title(sprintf('F_h = %.2f N', F_aplicada))
        end
        if j == 1
            ylabel('\theta [°]')
        end
        if i == 1 && j == 1
            legend({'No lineal', 'Linealizado'}, 'Location', 'best')
        end
    end
end

sgtitle('Comparación modelo no lineal y linealizado','FontWeight','bold')
