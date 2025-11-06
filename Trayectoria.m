clc, clear, close all;
run('robot.m');

%% CONFIGURACIÓN
q0 = [0, 0, 0, 0, 0, 0];
N_interpol = 20;
Ts = 0.06;
velocidad_cartesiana = 2.5;
Q_total = [];

%% DEFINICIÓN DE SECUENCIAS
% Estructura: {puntos, orientación, tipo_trayectoria, [nivel_suavizado(1-5)]}

secuencias = {
    {[2 -0.5 2.1; 2.15 -0.5 1.8; 2.35 -0.3 1.1], [0 0 1; 0 -1 0; 1 0 0]', 'mstraj', 2};
};

%% GENERAR TRAYECTORIA
fprintf('\nGenerando trayectoria completa...\n');

for s = 1:length(secuencias)
    puntos = secuencias{s}{1};
    R_orientacion = secuencias{s}{2};
    tipo = secuencias{s}{3};
    
    if length(secuencias{s}) >= 4
        nivel_suavizado = secuencias{s}{4};
    else
        nivel_suavizado = 3;
    end
    
    if isempty(Q_total)
        q_actual = q0;
    else
        q_actual = Q_total(end,:);
    end
    
    switch tipo
        case 'home'
            Q_seg = jtraj(q_actual, puntos, N_interpol);
            
        case 'puerta'
            q_seed = q0;
            q_seed(5) = -pi/2;
            Q_seg = procesar_jtraj(puntos, R_orientacion, q_actual, q_seed, N_interpol, R);
            
        case 'puerta_ret'
            T = [R_orientacion, puntos'; 0 0 0 1];
            q = cin_inv_IRB6710(R, T, q_actual, true);
            Q_seg = jtraj(q_actual, q, N_interpol);
            
        case 'jtraj'
            Q_seg = procesar_jtraj(puntos, R_orientacion, q_actual, [], N_interpol, R);
            
        case 'ctraj'
            Q_seg = procesar_ctraj(puntos, R_orientacion, q_actual, N_interpol, R);
            
        case 'mstraj'
            Q_seg = procesar_mstraj(puntos, R_orientacion, q_actual, Ts, velocidad_cartesiana, nivel_suavizado, R);
    end
    
    Q_total = [Q_total; Q_seg];
end

fprintf('Total configuraciones: %d\n', size(Q_total,1));

%% VISUALIZACIÓN Y ENVÍO
if input('\n¿Mostrar gráficas? (1=Sí, 2=No): ') == 1
    graficar_trayectoria(Q_total, R, Ts);
end

if input('\n¿Enviar a Unity? (1=Sí, 2=No): ') == 1
    enviar_trayectoria_unity(Q_total, 20, 55001);
end

%% FUNCIONES AUXILIARES

function Q_seg = procesar_jtraj(puntos, R_orientacion, q_actual, q_seed_inicial, N_interpol, R_robot)
    Q_seg = [];
    q_anterior = q_actual;
    
    for i = 1:size(puntos,1)
        T = [R_orientacion, puntos(i,:)'; 0 0 0 1];
        if i == 1 && ~isempty(q_seed_inicial)
            q = cin_inv_IRB6710(R_robot, T, q_seed_inicial, true);
        else
            q = cin_inv_IRB6710(R_robot, T, q_anterior, true);
        end
        Q_seg = [Q_seg; jtraj(q_anterior, q, N_interpol)];
        q_anterior = q;
    end
end

function Q_seg = procesar_ctraj(puntos, R_orientacion, q_actual, N_interpol, R_robot)
    T_primer = [R_orientacion, puntos(1,:)'; 0 0 0 1];
    q_primer = cin_inv_IRB6710(R_robot, T_primer, q_actual, true);
    Q_seg = jtraj(q_actual, q_primer, N_interpol);
    
    T_array = generar_transformaciones(puntos', R_orientacion);
    Q_ctraj = trayectoria_cartesiana(R_robot, T_array, q_primer, N_interpol, true);
    Q_seg = [Q_seg; Q_ctraj];
end

function Q_seg = procesar_mstraj(puntos, R_orientacion, q_actual, Ts, velocidad_max, nivel_suavizado, R_robot)
    if size(puntos, 1) == 1
        T = [R_orientacion, puntos(1,:)'; 0 0 0 1];
        q = cin_inv_IRB6710(R_robot, T, q_actual, true);
        Q_seg = jtraj(q_actual, q, 15);
        return;
    end
    
    tacc_map = [0.1, 0.2, 0.3, 0.4, 0.5];
    tacc = tacc_map(max(1, min(nivel_suavizado, 5)));
    
    T_actual = R_robot.fkine(q_actual);
    if isa(T_actual, 'SE3')
        T_actual = T_actual.double;
    end
    p_actual = T_actual(1:3, 4)';
    
    vel = [velocidad_max velocidad_max velocidad_max];
    Q_cart = mstraj(puntos, vel, [], p_actual, Ts, tacc);
    
    N = size(Q_cart, 1);
    Q_seg = zeros(N, 6);
    q_seed = q_actual;
    
    for i = 1:N
        T = [R_orientacion, Q_cart(i,:)'; 0 0 0 1];
        Q_seg(i,:) = cin_inv_IRB6710(R_robot, T, q_seed, true);
        
        if i > 1
            for j = 1:6
                diff = Q_seg(i,j) - Q_seg(i-1,j);
                if abs(diff) > pi
                    Q_seg(i,j) = Q_seg(i,j) - round(diff/(2*pi)) * 2*pi;
                end
            end
        end
        q_seed = Q_seg(i,:);
    end
end

function T_array = generar_transformaciones(puntos, R_tool)
    n_puntos = size(puntos, 2);
    T_array = zeros(4, 4, n_puntos);
    for i = 1:n_puntos
        T_array(:,:,i) = [R_tool, puntos(:,i); 0 0 0 1];
    end
end
