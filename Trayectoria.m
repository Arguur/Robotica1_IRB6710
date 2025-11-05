clc, clear, close all;
run('robot.m');

%% CONFIGURACIÓN
q0 = [0, 0, 0, 0, 0, 0];
N_interpol = 15;
tiempo_por_segmento = 2;
Ts = tiempo_por_segmento / N_interpol;
velocidad_cartesiana = 0.6;
Q_total = [];

%% DEFINICIÓN DE TODAS LAS SECUENCIAS
% Estructura: {puntos, orientación, tipo_trayectoria, [nivel_suavizado]}
% nivel_suavizado (solo para mstraj): 1=pasa cerca, 5=pasa muy lejos

secuencias = {
    {[2 -0.5 2.1; 2.15 -0.5 1.8; 2.35 -0.3 1.15], [0 0 1; 0 -1 0; 1 0 0]', 'mstraj', 5};
    
};

%% EJECUTAR SECUENCIAS
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
            Q_total = [Q_total; Q_seg];
            
        case 'puerta'
            q_seed = q0;
            q_seed(5) = -pi/2;
            Q_seg = procesar_jtraj(puntos, R_orientacion, q_actual, q_seed, N_interpol, R);
            Q_total = [Q_total; Q_seg];
            
        case 'puerta_ret'
            T = [R_orientacion, puntos'; 0 0 0 1];
            q = cin_inv_IRB6710(R, T, q_actual, true);
            Q_seg = jtraj(q_actual, q, N_interpol);
            Q_total = [Q_total; Q_seg];
            
        case 'jtraj'
            Q_seg = procesar_jtraj(puntos, R_orientacion, q_actual, [], N_interpol, R);
            Q_total = [Q_total; Q_seg];
            
        case 'ctraj'
            Q_seg = procesar_ctraj(puntos, R_orientacion, q_actual, N_interpol, R);
            Q_total = [Q_total; Q_seg];
            
        case 'mstraj'
            Q_seg = procesar_mstraj(puntos, R_orientacion, q_actual, Ts, velocidad_cartesiana, nivel_suavizado, R);
            Q_total = [Q_total; Q_seg];
    end
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
  
    T_primer = [R_orientacion, puntos(1,:)'; 0 0 0 1];
    q_primer = cin_inv_IRB6710(R_robot, T_primer, q_actual, true);
    Q_seg = jtraj(q_actual, q_primer, 10);
    
    if size(puntos, 1) == 1
        return;
    end
    
    % Mapear nivel_suavizado
    tacc_map = [0.1, 0.3, 0.5, 0.7, 3.0];
    if nivel_suavizado < 1 || nivel_suavizado > 5
        nivel_suavizado = 3;  
    end
    tacc = tacc_map(nivel_suavizado);
    
    fprintf('   → mstraj: nivel %d, tacc=%.2fs (radio de curva %s)\n', ...
            nivel_suavizado, tacc, ...
            ['pequeño', 'medio-bajo', 'medio', 'medio-alto', 'grande']);
    
    % Generar trayectoria suave con mstraj
    vel = [velocidad_max velocidad_max velocidad_max];
    Q_cart = mstraj(puntos, vel, [], [], Ts, tacc);
    
    % Cinemática inversa
    N = size(Q_cart, 1);
    Q_mstraj = zeros(N, 6);
    q_seed = q_primer;
    
    for i = 1:N
        T = [R_orientacion, Q_cart(i,:)'; 0 0 0 1];
        Q_mstraj(i,:) = cin_inv_IRB6710(R_robot, T, q_seed, true);
        
        % Corregir saltos
        if i > 1
            for j = 1:6
                diff = Q_mstraj(i,j) - Q_mstraj(i-1,j);
                if abs(diff) > pi
                    Q_mstraj(i,j) = Q_mstraj(i,j) - round(diff/(2*pi)) * 2*pi;
                end
            end
        end
        q_seed = Q_mstraj(i,:);
    end
    
    Q_seg = [Q_seg; Q_mstraj];
end

function T_array = generar_transformaciones(puntos, R_tool)
    n_puntos = size(puntos, 2);
    T_array = zeros(4, 4, n_puntos);
    for i = 1:n_puntos
        T_array(:,:,i) = [R_tool, puntos(:,i); 0 0 0 1];
    end
end
