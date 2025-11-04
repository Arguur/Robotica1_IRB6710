clc, clear, close all;
run('robot.m');

%% CONFIGURACIÓN INICIAL
q0 = [0, 0, 0, 0, 0, 0];
N_interpol = 15;  
Q_total = [];

fprintf('\n=== INICIO DE TRAYECTORIA COMPLETA ===\n');

%% SECUENCIA 1: APROXIMACIÓN INICIAL (JTRAJ)

fprintf('\n[1] Aproximación inicial...\n');

puntos_aprox = [
    2.5, -0.4, 2;
    2.5, -0.8, 2;
    2.5, -0.8, 1.3
];
R_aprox = [0 0 1; 0 -1 0; 1 0 0];

q_aprox = zeros(size(puntos_aprox,1), 6);
for i = 1:size(puntos_aprox,1)
    T_temp = [R_aprox, puntos_aprox(i,:)'; 0 0 0 1];
    if i == 1
        q_aprox(i,:) = cin_inv_IRB6710(R, T_temp, q0, true);
    else
        q_aprox(i,:) = cin_inv_IRB6710(R, T_temp, q_aprox(i-1,:), true);
    end
end

for i = 1:size(puntos_aprox,1)
    if i == 1
        Q_seg = jtraj(q0, q_aprox(i,:), N_interpol);
    else
        Q_seg = jtraj(q_aprox(i-1,:), q_aprox(i,:), N_interpol);
    end
    Q_total = [Q_total; Q_seg];
end
fprintf('  Configuraciones: %d\n', size(Q_total,1));

%% SECUENCIA 2: SOLDADURA VERTICAL

fprintf('\n[2] Soldadura vertical...\n');

puntos_sold1 = [
    2.5027, -0.5388, 1.7983;
    2.5045, -0.4421, 1.8006;
    2.5052, -0.3485, 1.8005;
    2.5059, -0.2551, 1.8005
];
R_sold1 = [0 0 -1; 0 1 0; 1 0 0]';

% JTRAJ al primer punto de soldadura
T_primer = [R_sold1, puntos_sold1(1,:)'; 0 0 0 1];
q_primer = cin_inv_IRB6710(R, T_primer, Q_total(end,:), true);
Q_seg = jtraj(Q_total(end,:), q_primer, N_interpol);
Q_total = [Q_total; Q_seg];

% CTRAJ para toda la soldadura
T_sold1 = generar_transformaciones(puntos_sold1', R_sold1);
Q_seg = trayectoria_cartesiana(R, T_sold1, Q_total(end,:), N_interpol, true);
Q_total = [Q_total; Q_seg];
fprintf('  Configuraciones: %d\n', size(Q_total,1));

%% SECUENCIA 3: RETORNO

fprintf('\n[3] Retorno a punto intermedio...\n');

punto_retorno1 = [2.5, -0.8, 1.3];
R_retorno = [0 0 1; 0 -1 0; 1 0 0];
T_retorno1 = [R_retorno, punto_retorno1'; 0 0 0 1];
q_retorno1 = cin_inv_IRB6710(R, T_retorno1, Q_total(end,:), true);

Q_seg = jtraj(Q_total(end,:), q_retorno1, N_interpol);
Q_total = [Q_total; Q_seg];
fprintf('  Configuraciones: %d\n', size(Q_total,1));

%% SECUENCIA 4: SOLDADURA DIAGONAL

fprintf('\n[4] Soldadura diagonal...\n');

puntos_sold2 = [
    2.4720, -0.1942, 1.7185;
    2.4380, -0.1895, 1.6160;
    2.4104, -0.1813, 1.4977;
    2.3934, -0.1748, 1.3679;
    2.3895, -0.1673, 1.2493;
    2.3895, -0.1594, 1.1358;
    2.3895, -0.1515, 1.0121;
    2.3895, -0.1408, 0.8662
];
R_sold2 = [0 -1 0; 0 0 -1; 1 0 0]';

% JTRAJ al primer punto de soldadura
T_primer = [R_sold2, puntos_sold2(1,:)'; 0 0 0 1];
q_primer = cin_inv_IRB6710(R, T_primer, Q_total(end,:), true);
Q_seg = jtraj(Q_total(end,:), q_primer, N_interpol);
Q_total = [Q_total; Q_seg];

% CTRAJ para toda la soldadura
T_sold2 = generar_transformaciones(puntos_sold2', R_sold2);
Q_seg = trayectoria_cartesiana(R, T_sold2, Q_total(end,:), N_interpol, true);
Q_total = [Q_total; Q_seg];
fprintf('  Configuraciones: %d\n', size(Q_total,1));

%% SECUENCIA 5: RETORNO

fprintf('\n[5] Retorno a punto intermedio...\n');

punto_retorno2 = [2.5, -0.8, 1.3];
R_retorno2 = [0 0 1; 0 -1 0; 1 0 0];
T_retorno2 = [R_retorno2, punto_retorno2'; 0 0 0 1];
q_retorno2 = cin_inv_IRB6710(R, T_retorno2, Q_total(end,:), true);

Q_seg = jtraj(Q_total(end,:), q_retorno2, N_interpol);
Q_total = [Q_total; Q_seg];
fprintf('  Configuraciones: %d\n', size(Q_total,1));

%% SECUENCIA 6: SOLDADURA HORIZONTAL

fprintf('\n[6] Soldadura horizontal...\n');

puntos_sold3 = [
    2.3895, -0.2656, 0.7451;
    2.3895, -0.3649, 0.7332;
    2.3895, -0.4586, 0.7242;
    2.3895, -0.5553, 0.7181
];
R_sold3 = [0 0 1; 0 -1 0; 1 0 0];

% JTRAJ al primer punto de soldadura
T_primer = [R_sold3, puntos_sold3(1,:)'; 0 0 0 1];
q_primer = cin_inv_IRB6710(R, T_primer, Q_total(end,:), true);
Q_seg = jtraj(Q_total(end,:), q_primer, N_interpol);
Q_total = [Q_total; Q_seg];

% CTRAJ para toda la soldadura
T_sold3 = generar_transformaciones(puntos_sold3', R_sold3);
Q_seg = trayectoria_cartesiana(R, T_sold3, Q_total(end,:), N_interpol, true);
Q_total = [Q_total; Q_seg];
fprintf('  Configuraciones: %d\n', size(Q_total,1));

%% SECUENCIA 7: RETORNO A HOME

fprintf('\n[7] Retorno a home...\n');

puntos_retorno_home = [
    2.5, -0.8, 1.3;
    2.5, -0.8, 2;
    2.5, -0.4, 2
];
R_retorno_home = [0 0 1; 0 -1 0; 1 0 0];

q_retorno_home = zeros(size(puntos_retorno_home,1), 6);
for i = 1:size(puntos_retorno_home,1)
    T_temp = [R_retorno_home, puntos_retorno_home(i,:)'; 0 0 0 1];
    if i == 1
        q_retorno_home(i,:) = cin_inv_IRB6710(R, T_temp, Q_total(end,:), true);
    else
        q_retorno_home(i,:) = cin_inv_IRB6710(R, T_temp, q_retorno_home(i-1,:), true);
    end
end

for i = 1:size(puntos_retorno_home,1)
    if i == 1
        Q_seg = jtraj(Q_total(end,:), q_retorno_home(i,:), N_interpol);
    else
        Q_seg = jtraj(q_retorno_home(i-1,:), q_retorno_home(i,:), N_interpol);
    end
    Q_total = [Q_total; Q_seg];
end

Q_seg = jtraj(Q_total(end,:), q0, N_interpol);
Q_total = [Q_total; Q_seg];
fprintf('  Configuraciones: %d\n', size(Q_total,1));

%%  SECUENCIA 8:

fprintf('\n[8] Paso por puerta (q5=-90)...\n');

puntos_puerta = [
    2, 0, 2;
    2, 0.5, 0.85
];
R_puerta = [0 0 1; 0 -1 0; 1 0 0];

Q_puerta = zeros(size(puntos_puerta,1), 6);
for i = 1:size(puntos_puerta,1)
    T_temp = [R_puerta, puntos_puerta(i,:)'; 0 0 0 1];
    if i == 1
        q_seed = q0;
        q_seed(5) = -pi/2;
    else
        q_seed = Q_puerta(i-1,:);
    end
    Q_puerta(i,:) = cin_inv_IRB6710(R, T_temp, q_seed, true);
end

Q_seg = jtraj(Q_total(end,:), Q_puerta(1,:), N_interpol);
Q_total = [Q_total; Q_seg];

Q_seg = jtraj(Q_puerta(1,:), Q_puerta(2,:), N_interpol);
Q_total = [Q_total; Q_seg];
fprintf('  Configuraciones: %d\n', size(Q_total,1));

%% SECUENCIA 9: POSICIONAMIENTO INTERIOR

fprintf('\n[9] Posicionamiento interior...\n');

punto_prep1 = [2.38, 0.5, 1.1];
R_prep1 = [0 1 0; 0 0 1; 1 0 0]';
T_prep1 = [R_prep1, punto_prep1'; 0 0 0 1];

q_prep1 = cin_inv_IRB6710(R, T_prep1, Q_total(end,:), true);
Q_seg = jtraj(Q_total(end,:), q_prep1, N_interpol);
Q_total = [Q_total; Q_seg];
fprintf('  Configuraciones: %d\n', size(Q_total,1));

%% SECUENCIA 10: SOLDADURA INTERIOR LADO 1

fprintf('\n[10] Soldadura interior lado 1...\n');

puntos_sold4 = [
    2.3898, 0.0147, 1.2856;
    2.3895, 0.0136, 1.1994;
    2.3895, 0.0209, 1.1039;
    2.3895, 0.0422, 1.0103;
    2.3895, 0.0779, 0.9300
];

% JTRAJ al primer punto de soldadura
T_primer = [R_prep1, puntos_sold4(1,:)'; 0 0 0 1];
q_primer = cin_inv_IRB6710(R, T_primer, Q_total(end,:), true);
Q_seg = jtraj(Q_total(end,:), q_primer, N_interpol);
Q_total = [Q_total; Q_seg];

% CTRAJ para toda la soldadura
T_sold4 = generar_transformaciones(puntos_sold4', R_prep1);
Q_seg = trayectoria_cartesiana(R, T_sold4, Q_total(end,:), N_interpol, true);
Q_total = [Q_total; Q_seg];
fprintf('  Configuraciones: %d\n', size(Q_total,1));

%% SECUENCIA 11: RETORNO

fprintf('\n[11] Retorno a preparación...\n');

q_retorno_prep = cin_inv_IRB6710(R, T_prep1, Q_total(end,:), true);
Q_seg = jtraj(Q_total(end,:), q_retorno_prep, N_interpol);
Q_total = [Q_total; Q_seg];
fprintf('  Configuraciones: %d\n', size(Q_total,1));

%% SECUENCIA 12: SOLDADURA INTERIOR LADO 2

fprintf('\n[12] Soldadura interior lado 2...\n');

puntos_sold5 = [
    2.3895, 0.9243, 1.2498;
    2.3974, 0.9178, 1.3903;
    2.3895, 0.9328, 1.1074;
    2.3895, 0.9410, 0.9713
];
R_sold5 = [0 -1 0; 0 0 -1; 1 0 0]';

% JTRAJ al primer punto de soldadura
T_primer = [R_sold5, puntos_sold5(1,:)'; 0 0 0 1];
q_primer = cin_inv_IRB6710(R, T_primer, Q_total(end,:), true);
Q_seg = jtraj(Q_total(end,:), q_primer, N_interpol);
Q_total = [Q_total; Q_seg];

% CTRAJ para toda la soldadura
T_sold5 = generar_transformaciones(puntos_sold5', R_sold5);
Q_seg = trayectoria_cartesiana(R, T_sold5, Q_total(end,:), N_interpol, true);
Q_total = [Q_total; Q_seg];
fprintf('  Configuraciones: %d\n', size(Q_total,1));

%% SECUENCIA 13: SALIDA POR PUERTA

fprintf('\n[13] Salida por puerta...\n');

puntos_salida = [
    2.38, 0.5, 1.1;
    2, 0.5, 0.85
];
R_salida = [0 0 1; 0 -1 0; 1 0 0];

q_salida = zeros(size(puntos_salida,1), 6);
for i = 1:size(puntos_salida,1)
    T_temp = [R_salida, puntos_salida(i,:)'; 0 0 0 1];
    if i == 1
        q_salida(i,:) = cin_inv_IRB6710(R, T_temp, Q_total(end,:), true);
    else
        q_salida(i,:) = cin_inv_IRB6710(R, T_temp, q_salida(i-1,:), true);
    end
end

for i = 1:size(puntos_salida,1)
    if i == 1
        Q_seg = jtraj(Q_total(end,:), q_salida(i,:), N_interpol);
    else
        Q_seg = jtraj(q_salida(i-1,:), q_salida(i,:), N_interpol);
    end
    Q_total = [Q_total; Q_seg];
end

Q_seg = jtraj(Q_total(end,:), Q_puerta(1,:), N_interpol);
Q_total = [Q_total; Q_seg];

Q_seg = jtraj(Q_total(end,:), q0, N_interpol);
Q_total = [Q_total; Q_seg];
fprintf('  Configuraciones: %d\n', size(Q_total,1));

%% SECUENCIA 14: SOLDADURA FINAL TECHO

fprintf('\n[14] Soldadura final techo...\n');

puntos_sold_final = [
    2.5562, -0.5528, 1.9050;
    2.5631, -0.3626, 1.9136;
    2.5646, -0.1768, 1.9127;
    2.5602, 0.0190, 1.9024;
    2.5505, 0.2042, 1.8801;
    2.5174, 0.4079, 1.8429;
    2.5095, 0.5614, 1.7924;
    2.5022, 0.6005, 1.7772;
    2.4739, 0.7767, 1.6995;
    2.4404, 0.9703, 1.5900;
    2.4137, 1.1999, 1.4657
];
R_sold_final = [0 0 1; 0 -1 0; 1 0 0]';

% JTRAJ al primer punto de soldadura
T_primer = [R_sold_final, puntos_sold_final(1,:)'; 0 0 0 1];
q_primer = cin_inv_IRB6710(R, T_primer, Q_total(end,:), true);
Q_seg = jtraj(Q_total(end,:), q_primer, N_interpol);
Q_total = [Q_total; Q_seg];

% CTRAJ para toda la soldadura
T_sold_final = generar_transformaciones(puntos_sold_final', R_sold_final);
Q_seg = trayectoria_cartesiana(R, T_sold_final, Q_total(end,:), N_interpol, true);
Q_total = [Q_total; Q_seg];
fprintf('  Configuraciones: %d\n', size(Q_total,1));

%%  RETORNO FINAL A HOME

fprintf('\n[15] Retorno final a home...\n');

Q_seg = jtraj(Q_total(end,:), q0, N_interpol);
Q_total = [Q_total; Q_seg];
fprintf('  Configuraciones: %d\n', size(Q_total,1));

%% RESUMEN
fprintf('\n========================================\n');
fprintf('TRAYECTORIA COMPLETA GENERADA\n');
fprintf('Total de configuraciones: %d\n', size(Q_total,1));
fprintf('========================================\n');

%% GRÁFICOS
fprintf('\n¿Mostrar gráficos de velocidad y aceleración?\n');
fprintf('1. Sí\n');
fprintf('2. No\n');
opcion_graficas = input('Opción: ');

if opcion_graficas == 1
    fprintf('\n Generando gráficos...\n');
    graficar_trayectoria(Q_total, R);
end

%% ENVÍO A UNITY
fprintf('\n¿Enviar trayectoria a Unity?\n');
fprintf('1. Sí\n');
fprintf('2. No\n');
opcion = input('Opción: ');

if opcion == 1
    fprintf('\n Enviando trayectoria completa...\n');
    enviar_trayectoria_unity(Q_total, 20, 55001);
end

%% FUNCIONES AUXILIARES

function T_array = generar_transformaciones(puntos, R_tool)
    n_puntos = size(puntos, 2);
    T_array = zeros(4, 4, n_puntos);
    
    for i = 1:n_puntos
        T_array(:,:,i) = [R_tool, puntos(:,i); 0 0 0 1];
    end
end
