function Q_traj = trayectoria_articular(R, T_puntos, q0, N_interpol)
% TRAYECTORIA_ARTICULAR Genera trayectoria por interpolación en espacio articular
% Entradas: [R] robot, [T_puntos] matrices de transformación objetivo (4x4xn), [q0] conf inicial, [N_interpol] puntos de interpolación
% Salida: [Q_traj] trayectoria completa en espacio articular

n_puntos = size(T_puntos, 3);

%% CINEMÁTICA INVERSA PARA CADA PUNTO
q_puntos = zeros(n_puntos, 6);
for i = 1:n_puntos
    if i == 1
        q_seed = q0;
    else
        q_seed = q_puntos(i-1, :);
    end
    q_puntos(i, :) = cin_inv_IRB6710(R, T_puntos(:,:,i), q_seed, true);
end

%% CORRECCIÓN DE SALTOS ANGULARES

q_puntos = unwrap(q_puntos);

% fuerzo continuidad en muñeca
for i = 2:n_puntos
    for j = 4:6
        diff_angle = q_puntos(i,j) - q_puntos(i-1,j);
        if abs(diff_angle) > pi
            q_puntos(i,j) = q_puntos(i,j) - round(diff_angle/(2*pi)) * 2*pi;
        end
    end
end

%% INTERPOLACIÓN ARTICULAR (jtraj)
Q_traj = [];
for i = 1:n_puntos-1
    q_interp = jtraj(q_puntos(i,:), q_puntos(i+1,:), N_interpol);
    Q_traj = [Q_traj; q_interp];
end

% Verificación final
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