function Q_traj = trayectoria_mstraj(R, T_puntos, q0, Ts, velocidad_max)
% TRAYECTORIA_MSTRAJ Genera trayectoria usando mstraj de Peter Corke
% Entradas: 
%   R - Robot SerialLink
%   T_puntos - Matrices de transformación (4x4xn)
%   q0 - Configuración inicial
%   Ts - Periodo de muestreo [s]
%   velocidad_max - Velocidad máxima cartesiana [m/s] (opcional, default 0.5)

if nargin < 5
    velocidad_max = 0.9; 
end

% Extraer waypoints cartesianos
n_puntos = size(T_puntos, 3);
waypoints = zeros(n_puntos, 3);
for i = 1:n_puntos
    waypoints(i,:) = T_puntos(1:3, 4, i)';
end

% Generar trayectoria cartesiana con mstraj
tacc = 0.2;  % Tiempo de aceleración

% CORRECCIÓN: pasar primer punto como q0 y waypoints desde el 2do
% Esto evita detenerse en el primer punto (modo continuo)
if n_puntos > 1
    Q_cart = mstraj(waypoints(2:end,:), ...
                    [velocidad_max velocidad_max velocidad_max], ...
                    [], ...              % tseg vacío = calcula automático
                    waypoints(1,:), ...  % primer punto como inicial
                    Ts, tacc);
else
    Q_cart = waypoints;  % un solo punto
end

% Cinemática inversa para cada punto
N = size(Q_cart, 1);
Q_traj = zeros(N, 6);
q_seed = q0;

for i = 1:N
    T = T_puntos(:,:,1);
    T(1:3, 4) = Q_cart(i,:)';
    
    Q_traj(i,:) = cin_inv_IRB6710(R, T, q_seed, true);
    
    % Corregir saltos
    if i > 1
        for j = 1:6
            diff = Q_traj(i,j) - Q_traj(i-1,j);
            if abs(diff) > pi
                Q_traj(i,j) = Q_traj(i,j) - round(diff/(2*pi)) * 2*pi;
            end
        end
    end
    
    q_seed = Q_traj(i,:);
end

fprintf('✅ Trayectoria mstraj: %d puntos generados\n', N);
end
