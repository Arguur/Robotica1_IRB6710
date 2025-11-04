function graficar_trayectoria(Q_traj, R)
    % GRAFICAR_TRAYECTORIA Genera gráficas completas de la trayectoria
    % Entradas:
    %   Q_traj - Matriz de configuraciones articulares [N x 6]
    %   R - Robot (SerialLink)
    
    [N, ~] = size(Q_traj);
    Ts = 0.1;  
    t = (0:N-1)' * Ts;
    
    fprintf('\n========================================\n');
    fprintf('GENERANDO GRÁFICAS DE TRAYECTORIA\n');
    fprintf('========================================\n');
    
    %% CÁLCULO DE VELOCIDADES Y ACELERACIONES ARTICULARES
    qd = zeros(N, 6);
    qdd = zeros(N, 6);
    for c = 1:6
        qd(:,c) = gradient(Q_traj(:,c), Ts);
        qdd(:,c) = gradient(qd(:,c), Ts);
    end
    
    %% LÍMITES ARTICULARES
    qlim = [
        deg2rad([-170 170]);  % q1
        deg2rad([-65   85]);  % q2
        deg2rad([-180  70]);  % q3
        deg2rad([-300 300]);  % q4
        deg2rad([-130 130]);  % q5
        deg2rad([-360 360])   % q6
    ];
    
    %% FIGURA 1: TRAYECTORIA EN ESPACIO CARTESIANO (3D)
    fprintf('\n[1] Generando trayectoria en espacio cartesiano...\n');
    figure(1); clf;
    set(gcf, 'Position', [50 50 800 700], 'Name', 'Trayectoria Cartesiana 3D');
    
    % Calcular posiciones cartesianas
    p = zeros(N, 3);
    for k = 1:N
        T = R.fkine(Q_traj(k,:));
        if isa(T, 'SE3')
            T = T.double;
        end
        p(k,:) = T(1:3, 4)';
    end
    
    % Graficar trayectoria 3D
    plot3(p(:,1), p(:,2), p(:,3), 'b-', 'LineWidth', 2);
    hold on;
    plot3(p(1,1), p(1,2), p(1,3), 'go', 'MarkerSize', 12, 'MarkerFaceColor', 'g');
    plot3(p(end,1), p(end,2), p(end,3), 'ro', 'MarkerSize', 12, 'MarkerFaceColor', 'r');
    plot3(0, 0, 0, 'ks', 'MarkerSize', 15, 'MarkerFaceColor', 'k');
    hold off;
    
    grid on;
    xlabel('X [m]');
    ylabel('Y [m]');
    zlabel('Z [m]');
    title('Trayectoria del Efector Final en Espacio Cartesiano');
    legend('Trayectoria', 'Inicio', 'Final', 'Base', 'Location', 'best');
    axis equal;
    view(45, 30);
    
    %% FIGURA 2: QPLOT - TODAS LAS ARTICULACIONES
    fprintf('[2] Generando qplot (todas las articulaciones)...\n');
    figure(2); clf;
    set(gcf, 'Position', [100 100 1200 400], 'Name', 'Posiciones Articulares - Todas');
    
    qplot(t, Q_traj);
    grid on;
    ylabel('q [rad]');
    xlabel('t [s]');
    title('Posiciones Articulares - Todas las Articulaciones');
    legend('q1','q2','q3','q4','q5','q6','Location','best');
    
    %% FIGURAS 3-8: CADA ARTICULACIÓN POR SEPARADO
    nombres_q = {'q1', 'q2', 'q3', 'q4', 'q5', 'q6'};
    
    for i = 1:6
        fprintf('[%d] Generando gráficas para %s...\n', i+2, nombres_q{i});
        
        fig = figure(i+2); clf;
        set(fig, 'Position', [150 + (i-1)*50, 150 + (i-1)*30, 1000, 800], ...
                 'Name', sprintf('Articulación %s', nombres_q{i}));
        
        % SUBPLOT 1: POSICIÓN
        subplot(3,1,1);
        plot(t, Q_traj(:,i), 'b-', 'LineWidth', 1.5);
        hold on;
        plot([t(1), t(end)], [qlim(i,1), qlim(i,1)], 'r--', 'LineWidth', 2);
        plot([t(1), t(end)], [qlim(i,2), qlim(i,2)], 'r--', 'LineWidth', 2);
        hold off;
        grid on;
        ylabel(sprintf('%s [rad]', nombres_q{i}));
        title(sprintf('Articulación %s - Posición', nombres_q{i}));
        legend('Posición', 'Límite mínimo', 'Límite máximo', 'Location', 'best');
        
        % SUBPLOT 2: VELOCIDAD
        subplot(3,1,2);
        plot(t, qd(:,i), 'g-', 'LineWidth', 1.5);
        grid on;
        ylabel(sprintf('%s [rad/s]', nombres_q{i}));
        title(sprintf('Articulación %s - Velocidad', nombres_q{i}));
        legend('Velocidad', 'Location', 'best');
        
        % SUBPLOT 3: ACELERACIÓN
        subplot(3,1,3);
        plot(t, qdd(:,i), 'r-', 'LineWidth', 1.5);
        grid on;
        ylabel(sprintf('%s [rad/s²]', nombres_q{i}));
        xlabel('t [s]');
        title(sprintf('Articulación %s - Aceleración', nombres_q{i}));
        legend('Aceleración', 'Location', 'best');
    end
    
    %% FIGURA 9: VARIABLES CARTESIANAS (POSICIÓN, VELOCIDAD, ACELERACIÓN)
    fprintf('[9] Generando variables cartesianas (x, v, a)...\n');
    
    % Calcular velocidades y aceleraciones cartesianas
    pd = zeros(N, 3);
    pdd = zeros(N, 3);
    for c = 1:3
        pd(:,c) = gradient(p(:,c), Ts);
        pdd(:,c) = gradient(pd(:,c), Ts);
    end
    
    figure(9); clf;
    set(gcf, 'Position', [200 200 1200 800], 'Name', 'Variables Cartesianas');
    
    % POSICIÓN CARTESIANA
    subplot(3,1,1);
    plot(t, p(:,1), 'r-', 'LineWidth', 1.5); hold on;
    plot(t, p(:,2), 'g-', 'LineWidth', 1.5);
    plot(t, p(:,3), 'b-', 'LineWidth', 1.5); hold off;
    grid on;
    ylabel('Posición [m]');
    title('Posición Cartesiana del Efector Final');
    legend('X', 'Y', 'Z', 'Location', 'best');
    
    % VELOCIDAD CARTESIANA
    subplot(3,1,2);
    plot(t, pd(:,1), 'r-', 'LineWidth', 1.5); hold on;
    plot(t, pd(:,2), 'g-', 'LineWidth', 1.5);
    plot(t, pd(:,3), 'b-', 'LineWidth', 1.5); hold off;
    grid on;
    ylabel('Velocidad [m/s]');
    title('Velocidad Cartesiana del Efector Final');
    legend('Vx', 'Vy', 'Vz', 'Location', 'best');
    
    % ACELERACIÓN CARTESIANA
    subplot(3,1,3);
    plot(t, pdd(:,1), 'r-', 'LineWidth', 1.5); hold on;
    plot(t, pdd(:,2), 'g-', 'LineWidth', 1.5);
    plot(t, pdd(:,3), 'b-', 'LineWidth', 1.5); hold off;
    grid on;
    ylabel('Aceleración [m/s²]');
    xlabel('t [s]');
    title('Aceleración Cartesiana del Efector Final');
    legend('Ax', 'Ay', 'Az', 'Location', 'best');
    
    %% RESUMEN
    fprintf('\n========================================\n');
    fprintf('RESUMEN DE GRÁFICAS GENERADAS:\n');
    fprintf('========================================\n');
    fprintf('Figura 1:  Trayectoria 3D en espacio cartesiano\n');
    fprintf('Figura 2:  Posiciones de todas las articulaciones (qplot)\n');
    fprintf('Figura 3:  Articulación q1 (posición, velocidad, aceleración)\n');
    fprintf('Figura 4:  Articulación q2 (posición, velocidad, aceleración)\n');
    fprintf('Figura 5:  Articulación q3 (posición, velocidad, aceleración)\n');
    fprintf('Figura 6:  Articulación q4 (posición, velocidad, aceleración)\n');
    fprintf('Figura 7:  Articulación q5 (posición, velocidad, aceleración)\n');
    fprintf('Figura 8:  Articulación q6 (posición, velocidad, aceleración)\n');
    fprintf('Figura 9:  Variables cartesianas (posición, velocidad, aceleración)\n');
    fprintf('========================================\n\n');
    
    %% ESTADÍSTICAS
    fprintf('ESTADÍSTICAS DE LA TRAYECTORIA:\n');
    fprintf('========================================\n');
    fprintf('Duración total: %.2f s\n', t(end));
    fprintf('Número de puntos: %d\n', N);
    fprintf('Periodo de muestreo: %.3f s\n', Ts);
    fprintf('\n');
    
    fprintf('Velocidades máximas articulares [rad/s]:\n');
    for i = 1:6
        fprintf('  q%d: %.4f\n', i, max(abs(qd(:,i))));
    end
    fprintf('\n');
    
    fprintf('Aceleraciones máximas articulares [rad/s²]:\n');
    for i = 1:6
        fprintf('  q%d: %.4f\n', i, max(abs(qdd(:,i))));
    end
    fprintf('\n');
    
    fprintf('Velocidad máxima cartesiana [m/s]:\n');
    for i = 1:3
        coord = {'X', 'Y', 'Z'};
        fprintf('  %s: %.4f\n', coord{i}, max(abs(pd(:,i))));
    end
    fprintf('========================================\n');

end

