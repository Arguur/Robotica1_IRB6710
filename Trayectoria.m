clc, clear, close all;
run('robot.m');

%% CONFIGURACIÓN
q0 = [0, 0, 0, 0, 0, 0];
N_interpol = 15;  
Q_total = [];

%% DEFINICIÓN DE TODAS LAS SECUENCIAS
% Estructura: {puntos, orientación, tipo_trayectoria}
% tipo: 'jtraj' o 'ctraj'

secuencias = {
    % 1. Aproximación inicial
    {[2.5 -0.4 2; 2.5 -0.8 2; 2.5 -0.8 1.3], [0 0 1; 0 -1 0; 1 0 0], 'jtraj'};
    
    % 2. Soldadura vertical
    {[2.5027 -0.5388 1.7983; 2.5045 -0.4421 1.8006; 2.5052 -0.3485 1.8005; 2.5059 -0.2551 1.8005], ...
     [0 0 -1; 0 1 0; 1 0 0]', 'ctraj'};
    
    % 3. Retorno
    {[2.5 -0.8 1.3], [0 0 1; 0 -1 0; 1 0 0], 'jtraj'};
    
    % 4. Soldadura diagonal
    {[2.4720 -0.1942 1.7185; 2.4380 -0.1895 1.6160; 2.4104 -0.1813 1.4977; ...
      2.3934 -0.1748 1.3679; 2.3895 -0.1673 1.2493; 2.3895 -0.1594 1.1358; ...
      2.3895 -0.1515 1.0121; 2.3895 -0.1408 0.8662], [0 -1 0; 0 0 -1; 1 0 0]', 'ctraj'};
    
    % 5. Retorno
    {[2.5 -0.8 1.3], [0 0 1; 0 -1 0; 1 0 0], 'jtraj'};
    
    % 6. Soldadura horizontal
    {[2.3895 -0.2656 0.7451; 2.3895 -0.3649 0.7332; 2.3895 -0.4586 0.7242; ...
      2.3895 -0.5553 0.7181], [0 0 1; 0 -1 0; 1 0 0], 'ctraj'};
    
    % 7. Retorno a home
    {[2.5 -0.8 1.3; 2.5 -0.8 2; 2.5 -0.4 2], [0 0 1; 0 -1 0; 1 0 0], 'jtraj'};
    {q0, [], 'home'};  % Caso especial para retorno a q0
    
    % 8. Paso por puerta
    {[2 0 2; 2 0.5 0.85], [0 0 1; 0 -1 0; 1 0 0], 'puerta'};
    
    % 9. Posicionamiento interior
    {[2.38 0.5 1.1], [0 1 0; 0 0 1; 1 0 0]', 'jtraj'};
    
    % 10. Soldadura interior lado 1
    {[2.3898 0.0147 1.2856; 2.3895 0.0136 1.1994; 2.3895 0.0209 1.1039; ...
      2.3895 0.0422 1.0103; 2.3895 0.0779 0.9300], [0 1 0; 0 0 1; 1 0 0]', 'ctraj'};
    
    % 11. Retorno preparación
    {[2.38 0.5 1.1], [0 1 0; 0 0 1; 1 0 0]', 'jtraj'};
    
    % 12. Soldadura interior lado 2
    {[2.3895 0.9243 1.2498; 2.3974 0.9178 1.3903; 2.3895 0.9328 1.1074; ...
      2.3895 0.9410 0.9713], [0 -1 0; 0 0 -1; 1 0 0]', 'ctraj'};
    
    % 13. Salida por puerta
    {[2.38 0.5 1.1; 2 0.5 0.85], [0 0 1; 0 -1 0; 1 0 0], 'jtraj'};
    {[2 0 2], [0 0 1; 0 -1 0; 1 0 0], 'puerta_ret'};
    {q0, [], 'home'};
    
    % 14. Soldadura final techo
    {[2.5562 -0.5528 1.9050; 2.5631 -0.3626 1.9136; 2.5646 -0.1768 1.9127; ...
      2.5602 0.0190 1.9024; 2.5505 0.2042 1.8801; 2.5174 0.4079 1.8429; ...
      2.5095 0.5614 1.7924; 2.5022 0.6005 1.7772; 2.4739 0.7767 1.6995; ...
      2.4404 0.9703 1.5900; 2.4137 1.1999 1.4657], [0 0 1; 0 -1 0; 1 0 0]', 'ctraj'};
    
    % 15. Retorno final
    {q0, [], 'home'};
};

%% EJECUTAR SECUENCIAS
fprintf('\nGenerando trayectoria completa...\n');

for s = 1:length(secuencias)
    puntos = secuencias{s}{1};
    R_orientacion = secuencias{s}{2};
    tipo = secuencias{s}{3};
    
    % Determinar configuración actual
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
    end
end

fprintf('Total configuraciones: %d\n', size(Q_total,1));

%% VISUALIZACIÓN Y ENVÍO
if input('\n¿Mostrar gráficas? (1=Sí, 2=No): ') == 1
    graficar_trayectoria(Q_total, R);
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
        elseif i == 1
            q = cin_inv_IRB6710(R_robot, T, q_anterior, true);
        else
            q = cin_inv_IRB6710(R_robot, T, q_anterior, true);
        end
        Q_seg = [Q_seg; jtraj(q_anterior, q, N_interpol)];
        q_anterior = q;
    end
end

function Q_seg = procesar_ctraj(puntos, R_orientacion, q_actual, N_interpol, R_robot)
    % Primero ir al primer punto
    T_primer = [R_orientacion, puntos(1,:)'; 0 0 0 1];
    q_primer = cin_inv_IRB6710(R_robot, T_primer, q_actual, true);
    Q_seg = jtraj(q_actual, q_primer, N_interpol);
    
    % Luego hacer trayectoria cartesiana
    T_array = generar_transformaciones(puntos', R_orientacion);
    Q_ctraj = trayectoria_cartesiana(R_robot, T_array, q_primer, N_interpol, true);
    Q_seg = [Q_seg; Q_ctraj];
end

function T_array = generar_transformaciones(puntos, R_tool)
    n_puntos = size(puntos, 2);
    T_array = zeros(4, 4, n_puntos);
    for i = 1:n_puntos
        T_array(:,:,i) = [R_tool, puntos(:,i); 0 0 0 1];
    end
end
