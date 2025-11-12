clc, clear, close all;
run('robot.m');

%% CONFIGURACIÓN
q0 = [0, 0, 0, 0, 0, 0];
N_interpol = 25;
Ts = 0.06;
velocidad_cartesiana = 1;
Q_total = [];
T_home = R.fkine(q0);
if isa(T_home, 'SE3')
    T_home = T_home.double;
end
p_home = T_home(1:3, 4)';  
R_home = T_home(1:3, 1:3);

%% DEFINICIÓN DE SECUENCIAS

secuencias = {
    %puerta trasera
    {[2 -0.5 2.1; 2.15 -0.5 1.8; 2.3531 -0.6324 1.1352], roty(pi/2)*rotz(pi), 'mstraj', 3};
    {[2.3571 -0.4962 1.1031; 2.3574 -0.3441 1.1002], roty(pi/2)*rotz(pi), 'ctraj'};
    {[2.3501 -0.1806 1.1559], roty(pi/2)*rotz(pi*15/12), 'ctraj'};
    {[2.3221 -0.1392 1.2688; 2.3215 -0.1490 1.4530; 2.3283 -0.1761 1.6401], roty(pi/2)*rotz(pi*1.5), 'ctraj'};
    {[2.3770 -0.1973 1.8364; 2.4500 -0.2131 1.9953], roty(pi/2)*rotz(pi*1.5)*rotx(-pi/6), 'ctraj'};
    {[2.5075 -0.3176 2.0853; 2.5021 -0.4534 2.0800; 2.5002 -0.5774 2.0692; 2.4949 -0.6949 2.0565;2.4741 -0.7996 2.0401], roty(pi*7.5/12)*rotz(pi*2), 'ctraj'};
    {[2.4741 -0.8596 1.9401; 2.3566 -1.15 1.7], roty(pi/2)*rotz(pi*9/12), 'mstraj', 3};
    {[2.3566 -1.1634 1.6662; 2.3419 -1.0787 1.5698; 2.3481 -0.9836 1.4647], roty(pi/2)*rotz(pi*9/12), 'ctraj'};

    %salir puerta trasera y pasar a la delantera
    {[2.3481 -0.9836 1.4647; 2.2 -0.4 1.6;2 -0.3 1.6; 1.9 -0.1 2;2 0.3 1.6], roty(pi/2)*rotz(pi), 'mstraj', 3};
    {[2.3458 0.0503 1.2728; 2.3433 -0.0076 1.4469; 2.3454 -0.0439 1.6143], roty(pi/2)*rotz(pi*8/12), 'ctraj'};
    {[2.3460 -0.0527 1.7397; 2.4055 -0.0957 1.9010; 2.4583 -0.1224 2.0002], roty(pi/2)*rotz(pi/2)*rotx(pi/6), 'ctraj'};
    {[2.3480 0.4301 1.48;2.3480 0.8 1.6580], roty(pi/2)*rotz(-pi/2), 'mstraj', 3};
    {[2.3480 0.9301 1.6580; 2.3465 0.9512 1.5403; 2.3466 0.9640 1.4115; 2.3466 0.9662 1.2703], roty(pi/2)*rotz(-pi/2), 'ctraj'};

    %salir de la puerta delantera
    {[2.3466 0.8 1.2703], roty(pi/2)*rotz(pi), 'ctraj'};
    {[2.1 0.4 2], roty(pi/2)*rotz(pi*3/4), 'jtraj'};

    %Soldar parabrisas
    {[2.5 0.56 2.16; 2.47 0.53 2.16], roty(pi/2)*rotz(pi*3/4), 'mstraj', 1};
    {[2.4719 0.5313 2; 2.4431 0.6438 1.942; 2.4166 0.7483 1.886; 2.3882 0.8646 1.8205; 2.3561 1.0087 1.7415; 2.3561 1.0087 1.8915], roty(pi/2)*rotz(pi*3/4)*rotx(pi/12), 'ctraj'};
    {q0, [], 'home'}
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

        case 'jtraj'
            T_array = generar_transformaciones(puntos', R_orientacion);
            Q_seg = trayectoria_articular(R, T_array, q_actual, N_interpol);
            
        case 'ctraj'
            T_array = generar_transformaciones(puntos', R_orientacion);
            
            T_primer = T_array(:,:,1);
            q_primer = cin_inv_IRB6710(R, T_primer, q_actual, true);
            Q_seg = jtraj(q_actual, q_primer, N_interpol);
            
            Q_ctraj = trayectoria_cartesiana(R, T_array, q_primer, N_interpol, true);
            Q_seg = [Q_seg; Q_ctraj];
            
        case 'mstraj'
            % Usar función externa modularizada
            T_array = generar_transformaciones(puntos', R_orientacion);
            Q_seg = trayectoria_mstraj(R, T_array, q_actual, Ts, velocidad_cartesiana, nivel_suavizado);
    end
    
    Q_total = [Q_total; Q_seg];
end
fprintf('Total configuraciones: %d\n', size(Q_total,1));

%% VISUALIZACIÓN Y ENVÍO
if input('\n¿Graficar trayectoria en MATLAB? (1=Sí, 2=No): ') == 1
       R.plot(Q_total, 'workspace', workspace, 'scale', 0.5, 'fps', 30, ...
               'trail', {'r', 'LineWidth', 2});
end

if input('\n¿Mostrar gráficas? (1=Sí, 2=No): ') == 1
    graficar_trayectoria(Q_total, R, Ts);
end

if input('\n¿Enviar a Unity? (1=Sí, 2=No): ') == 1
    enviar_trayectoria_unity(Q_total, 20, 55001);
end

%% FUNCIÓN AUXILIAR
function T_array = generar_transformaciones(puntos, R_tool)
    n_puntos = size(puntos, 2);
    T_array = zeros(4, 4, n_puntos);
    for i = 1:n_puntos
        T_array(:,:,i) = [R_tool, puntos(:,i); 0 0 0 1];
    end
end
