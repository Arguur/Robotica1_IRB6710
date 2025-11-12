function Q_traj = trayectoria_cartesiana(R, T_puntos, q0, N_interpol, mejor)
% TRAYECTORIA_CARTESIANA Genera trayectoria por interpolación en espacio cartesiano
% Entradas: [R] robot, [T_puntos] matrices de transformación objetivo (4x4xn), [q0] conf inicial, [N_interpol] puntos de interpolación, 
%  mejor flag para optimizar C.I.
% Salida: [Q_traj] trayectoria completa en espacio articular

n_puntos = size(T_puntos, 3);
Q_traj = [];

%% INTERPOLACIÓN CARTESIANA Y CINEMÁTICA INVERSA
for i = 1:n_puntos-1

    TT_interp = ctraj(T_puntos(:,:,i), T_puntos(:,:,i+1), N_interpol);
    q_interp = zeros(N_interpol, 6);
    
    for j = 1:N_interpol
        if i == 1 && j == 1
            q_seed = q0;
        elseif size(Q_traj, 1) > 0
            q_seed = Q_traj(end, :);
        else
            q_seed = q_interp(j-1, :);
        end
        
        if isa(TT_interp, 'SE3')
            T_actual = TT_interp(j).double;
        else
            T_actual = TT_interp(:,:,j);
        end
       
        q_interp(j, :) = cin_inv_IRB6710(R, T_actual, q_seed, mejor);
        
        if j > 1
            q_interp(j,:) = corregir_salto(q_interp(j,:), q_interp(j-1,:));
        elseif i > 1 && j == 1 && size(Q_traj, 1) > 0
            q_interp(j,:) = corregir_salto(q_interp(j,:), Q_traj(end,:));
        end
    end
    
    Q_traj = [Q_traj; q_interp];
end

%% VERIFICACIÓN Y CORRECCIÓN FINAL
[Q_traj, corregidos] = corregir_trayectoria_completa(Q_traj);

if corregidos == 0
    fprintf('✅ Trayectoria Cartesiana continua\n');
else
    fprintf('✅ Trayectoria Cartesiana: %d discontinuidades corregidas\n', corregidos);
end

end

%% FUNCIONES AUXILIARES
function q_corregido = corregir_salto(q_actual, q_ref)
    q_corregido = q_actual;
    
    for k = 1:6
        diff_angle = q_actual(k) - q_ref(k);
        
        if abs(diff_angle) > pi
            q_corregido(k) = q_actual(k) - round(diff_angle/(2*pi)) * 2*pi;
            
            diff_corregido = q_corregido(k) - q_ref(k);
            if abs(diff_corregido) > pi
                q_corregido(k) = q_corregido(k) - sign(diff_corregido) * 2*pi;
            end
        end
    end
end

function [Q_traj, total_corregidos] = corregir_trayectoria_completa(Q_traj)
    total_corregidos = 0;
    
    for i = 2:size(Q_traj, 1)
        q_original = Q_traj(i,:);
        Q_traj(i,:) = corregir_salto(Q_traj(i,:), Q_traj(i-1,:));
        
        if any(abs(q_original - Q_traj(i,:)) > 1e-6)
            total_corregidos = total_corregidos + 1;
        end
    end
end
