function Q_traj = trayectoria_mstraj(R, T_puntos, q0, Ts, velocidad_max)

if nargin < 5
    velocidad_max = 1;
end

n_puntos = size(T_puntos, 3);
waypoints = zeros(n_puntos, 3);
for i = 1:n_puntos
    waypoints(i,:) = T_puntos(1:3, 4, i)';
end

tacc = 0.2;
vel = [velocidad_max velocidad_max velocidad_max];

if n_puntos > 1
    Q_cart = mstraj(waypoints(2:end,:), vel, [], waypoints(1,:), Ts, tacc);
else
    Q_cart = waypoints;
end

N = size(Q_cart, 1);
Q_traj = zeros(N, 6);
q_seed = q0;

for i = 1:N
    T = T_puntos(:,:,1);
    T(1:3, 4) = Q_cart(i,:)';
    Q_traj(i,:) = cin_inv_IRB6710(R, T, q_seed, true);
    
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
