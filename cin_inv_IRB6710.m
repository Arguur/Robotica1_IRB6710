function Q = cin_inv_IRB6710(R, T, q0, mejor)

if nargin == 3
    mejor = false;
elseif nargin ~= 4
    error('Argumentos incorrectos. Uso: cin_inv_IRB6710(R, T, q0, mejor)')
end

% Eliminación de offsets
offsets = R.offset;
R.offset = zeros(6,1);

% Desacople de base y tool
if isa(T, 'SE3')
    T = T.double;
end
T = invHomog(R.base.double) * T * invHomog(R.tool.double);

% Punto (x,y,z) de la muñeca
p = T(1:3,4) - R.links(6).d * T(1:3,3);

% Cálculo de q1 (2 soluciones)
q1 = calcular_q1(p);

% Cálculo de q2 (2 soluciones para cada q1)
q21 = calcular_q2(R, q1(1), p);
q22 = calcular_q2(R, q1(2), p);

% Cálculo de q3 (1 solución por cada par q1,q2)
q311 = calcular_q3(R, q1(1), q21(1), p);
q312 = calcular_q3(R, q1(1), q21(2), p);
q321 = calcular_q3(R, q1(2), q22(1), p);
q322 = calcular_q3(R, q1(2), q22(2), p);

% Acomodo de soluciones en qq
qq(1,:) = [q1(1)  q1(1)  q1(1)  q1(1)  q1(2)  q1(2)  q1(2)  q1(2)];
qq(2,:) = [q21(1) q21(1) q21(2) q21(2) q22(1) q22(1) q22(2) q22(2)];
qq(3,:) = [q311   q311   q312   q312   q321   q321   q322   q322];

% Verificación parcial: posición de la muñeca
for i=1:8
    Taux = eye(4);
    for j=1:3
        Taux = Taux * R.links(j).A(qq(j,i)).double;
    end
    Taux = Taux * R.links(4).A(0).double;
    error_pos = norm(Taux(1:3,4) - p);
end

% Cálculo de q4, q5 y q6 (2 soluciones de q4 para cada tripla)
for i=1:2:7
    q1_i = qq(1, i);
    q2_i = qq(2, i);
    q3_i = qq(3, i);
    [q4, q5, q6] = calcular_qm(R, q1_i, q2_i, q3_i, T, q0);
    qq(4:6,i:i+1) = [q4; q5; q6];
end

% Restaurar offsets
R.offset = offsets;
qq = qq - R.offset' * ones(1,8);

% FILTRAR SOLUCIONES FUERA DE LÍMITES ARTICULARES
soluciones_validas = true(1, 8);

for i = 1:8
    for j = 1:6
        if qq(j,i) < R.qlim(j,1) || qq(j,i) > R.qlim(j,2)
            soluciones_validas(i) = false;
            break;
        end
    end
end

% Filtrar solo soluciones válidas
qq = qq(:, soluciones_validas);
n_soluciones = sum(soluciones_validas);

if n_soluciones == 0
    error('Ninguna solución respeta los límites articulares del robot');
end

% Verificación total: todas las soluciones válidas
for i = 1:n_soluciones
    Taux = R.fkine(qq(:,i));
    if isa(Taux, 'SE3')
        Taux = Taux.double;
    end
    error_pos = norm(Taux(1:3,4) - T(1:3,4));
    error_rot = norm(Taux(1:3,1:3) - T(1:3,1:3), 'fro');
end

% Devolución
if mejor
    if size(qq, 2) == 0
        error('No hay soluciones dentro de límites articulares');
    end
    % Encontrar la solución más cercana a q0
    Qaux = qq - q0' * ones(1, size(qq,2));
    normas = zeros(1, size(qq,2));
    for i = 1:size(qq,2)
        normas(i) = norm(Qaux(:,i));
    end
    [~, pos] = min(normas);
    Q = qq(:, pos);
else
    Q = qq;
end
end

%=========================================================================%
% FUNCIONES AUXILIARES
%=========================================================================%

function iT = invHomog(T)
% Inversa de matriz homogénea
    iT = eye(4);
    iT(1:3, 1:3) = T(1:3, 1:3)';
    iT(1:3, 4) = -iT(1:3, 1:3) * T(1:3, 4);
end

%=========================================================================%

function q1 = calcular_q1(p)
% Cálculo de q1 (2 soluciones)
    q1(1) = atan2(p(2), p(1));
    if q1(1) > 0
        q1(2) = q1(1) - pi;
    else
        q1(2) = q1(1) + pi;
    end
end

%=========================================================================%

function q2 = calcular_q2(R, q1, p)
% Cálculo de q2 (2 soluciones para un q1 dado)
    % Transformar p al sistema {S1}
    T1 = R.links(1).A(q1).double;
    p = invHomog(T1) * [p; 1];
    p = p(1:3);
    
    % Parámetros geométricos
    a2 = R.links(2).a;
    a3 = R.links(3).a;
    d4 = R.links(4).d;
    L = sqrt(a3^2 + d4^2);
    
    % Triángulo
    r = sqrt(p(1)^2 + p(2)^2);
    beta = atan2(p(2), p(1));
    cos_alpha = (a2^2 + r^2 - L^2) / (2 * a2 * r);
    
    % CRÍTICO: Verificar si la posición es alcanzable
    if abs(cos_alpha) > 1
        if abs(cos_alpha) > 1.01  % Tolerancia de 1%
            error('Posición FUERA DE ALCANCE. cos(alpha) = %.4f (debe estar en [-1,1])', cos_alpha);
        else
            % warning silencioso - posición en el límite de alcance
            cos_alpha = sign(cos_alpha);  % Saturar a ±1
        end
    end
    
    alpha = acos(cos_alpha);
    
    % Verificar si alpha es real
    if imag(alpha) ~= 0
        error('Ángulo alpha resultó imaginario. Posición inalcanzable.');
    end
    
    % Dos soluciones geométricas (sin offset)
    q2(1) = beta + alpha;
    q2(2) = beta - alpha;
end

%=========================================================================%

function q3 = calcular_q3(R, q1, q2, p)
% Cálculo de q3
    % Transformar p al sistema {S2}
    T1 = R.links(1).A(q1).double;
    T2 = T1 * R.links(2).A(q2).double;
    p = invHomog(T2) * [p; 1];
    p = p(1:3);
    
    % Parámetros
    a3 = R.links(3).a;
    d4 = R.links(4).d;
    
    % Ángulos
    gamma = atan2(d4, a3);
    theta = atan2(p(2), p(1));
    
    % Verificar que theta sea real
    if imag(theta) ~= 0
        error('Ángulo theta resultó imaginario en q3.');
    end
    
    % q3 geométrico
    q3 = theta - gamma;
end

%=========================================================================%

function [q4, q5, q6] = calcular_qm(R, q1, q2, q3, T, q0)
% Cálculo de q4, q5, q6 (muñeca esférica)
    % Calcular T36
    T1 = R.links(1).A(q1).double;
    T2 = R.links(2).A(q2).double;
    T3 = R.links(3).A(q3).double;
    
    T36 = invHomog(T3) * invHomog(T2) * invHomog(T1) * T;
    
    % Verificar caso degenerado
    if abs(T36(3,3) - 1) < 1e-6
        % Caso degenerado: z3 y z5 alineados - usar q4 anterior silenciosamente
        q4(1) = q0(4);
        q5(1) = 0;
        q6(1) = atan2(T36(2,1), T36(1,1)) - q4(1);
        q4(2) = q4(1);
        q5(2) = 0;
        q6(2) = q6(1);
    else
        % Caso normal: 2 soluciones de q4
        q4(1) = atan2(T36(2,3), T36(1,3));
        if q4(1) > 0
            q4(2) = q4(1) - pi;
        else
            q4(2) = q4(1) + pi;
        end
        
        q5 = zeros(1,2);
        q6 = zeros(1,2);
        
        for i=1:2
            % Calcular T46
            T4 = R.links(4).A(q4(i)).double;
            T46 = invHomog(T4) * T36;
            
            % q5
            q5(i) = atan2(T46(2,3), T46(1,3)) - pi/2;
            
            % Calcular T56
            T5 = R.links(5).A(q5(i)).double;
            T56 = invHomog(T5) * T46;
            
            % q6
            q6(i) = atan2(T56(2,1), T56(1,1));
        end
    end
end