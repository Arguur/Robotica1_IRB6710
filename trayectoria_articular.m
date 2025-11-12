function Q_traj = trayectoria_articular(R, T_puntos, q0, N_interpol, q_seed_inicial)
% Entradas:robot, matrices de transformación objetivo, configuración
% articular inicial, puntos de interpolación entre cada par de
% configuraciones, semilla para la primera cinemática inversa
% Salida: trayectoria completa en espacio articular

if nargin < 5
    q_seed_inicial = [];
end

n_puntos = size(T_puntos, 3);

%% GENERAR TRAYECTORIA CON JTRAJ
Q_traj = [];
q_anterior = q0;

for i = 1:n_puntos
    if i == 1 && ~isempty(q_seed_inicial)
        q_seed = q_seed_inicial;
    else
        q_seed = q_anterior;
    end
    
    q_objetivo = cin_inv_IRB6710(R, T_puntos(:,:,i), q_seed, true);
    
    q_interp = jtraj(q_anterior, q_objetivo, N_interpol);
    
    Q_traj = [Q_traj; q_interp];
    q_anterior = q_objetivo;
end

verificar_continuidad(Q_traj, 'Articular');

end

%% FUNCIÓN AUXILIAR
function verificar_continuidad(Q_traj, metodo)
    discontinuidades = 0;
    max_salto = 0;
    
    for i = 2:size(Q_traj, 1)
        for j = 1:6
            diff_angle = Q_traj(i,j) - Q_traj(i-1,j);
            if abs(diff_angle) > pi
                discontinuidades = discontinuidades + 1;
                max_salto = max(max_salto, abs(diff_angle));
            end
        end
    end
    
    if discontinuidades == 0
        fprintf('✅ Trayectoria %s continua\n', metodo);
    else
        fprintf('⚠️ Trayectoria %s: %d discontinuidades detectadas (máx: %.2f rad)\n', ...
                metodo, discontinuidades, max_salto);
    end
end
