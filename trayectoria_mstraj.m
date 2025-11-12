function Q_traj = trayectoria_mstraj(R, T_puntos, q0, Ts, velocidad_max, nivel_suavizado)
% Entradas: robot, matrices de transformación objetivo (4x4xn),
% configuración articular inicial, período de muestreo, velocidad máxima cartesiana (m/s), 
% nivel de suavizado (1-5): controla aceleración
% Salida: trayectoria completa en espacio articular

if nargin < 5
    velocidad_max = 1;
end
if nargin < 6
    nivel_suavizado = 3;
end

n_puntos = size(T_puntos, 3);

%% OBTENER POSICIÓN Y ORIENTACIÓN ACTUAL DEL ROBOT
T_actual = R.fkine(q0);
if isa(T_actual, 'SE3')
    T_actual = T_actual.double;
end
p_actual = T_actual(1:3, 4)';
R_actual = T_actual(1:3, 1:3);

%% EXTRAER ORIENTACIÓN Y POSICIONES DE LOS PUNTOS OBJETIVO
R_orientacion = T_puntos(1:3, 1:3, 1);

puntos = zeros(n_puntos, 3);
for i = 1:n_puntos
    puntos(i,:) = T_puntos(1:3, 4, i)';
end

waypoints = [p_actual; puntos];

%% CONFIGURACIÓN DE MSTRAJ
tacc_map = [0.05, 0.1, 0.3, 0.6, 0.7];
tacc = tacc_map(max(1, min(nivel_suavizado, 5)));

vel = [velocidad_max velocidad_max velocidad_max];

%% GENERAR TRAYECTORIA CARTESIANA CON MSTRAJ
Q_cart = mstraj(waypoints, vel, [], p_actual, Ts, tacc);
N = size(Q_cart, 1);

%% INTERPOLACIÓN DE ORIENTACIÓN
% Calcular si hay cambio de orientación significativo

R_diff = R_actual' * R_orientacion;
theta = acos((trace(R_diff) - 1) / 2);

umbral_angular = 1e-4;

if abs(theta) < umbral_angular
    interpolacion_necesaria = false;
else
    interpolacion_necesaria = true;
    T0_ori = [R_actual, [0;0;0]; 0 0 0 1];
    Tf_ori = [R_orientacion, [0;0;0]; 0 0 0 1];
    T_ori = ctraj(T0_ori, Tf_ori, N);
end

%% CINEMÁTICA INVERSA PARA CADA PUNTO
Q_traj = zeros(N, 6);
q_seed = q0;

for i = 1:N
    if interpolacion_necesaria
        if isa(T_ori, 'SE3')
            R_i = T_ori(i).R;
        else
            R_i = T_ori(1:3, 1:3, i);
        end
    else
        R_i = R_orientacion;
    end
    
    T = [R_i, Q_cart(i,:)'; 0 0 0 1];
    
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
