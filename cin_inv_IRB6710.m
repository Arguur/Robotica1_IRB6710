function Q = cin_inv_IRB6710(R, T, q0, mejor)

if nargin == 3
    mejor = false;
elseif nargin ~= 4
    error('Argumentos incorrectos. Uso: cin_inv_IRB6710(R, T, q0, mejor)')
end

offsets = R.offset;
R.offset = zeros(6,1);

if isa(T, 'SE3')
    T = T.double;
end
T = invHomog(R.base.double) * T * invHomog(R.tool.double);

% Punto (x,y,z) de la muñeca
p = T(1:3,4) - R.links(6).d * T(1:3,3);

q1 = calcular_q1(p);


q21 = calcular_q2(R, q1(1), p);
q22 = calcular_q2(R, q1(2), p);


q311 = calcular_q3(R, q1(1), q21(1), p);
q312 = calcular_q3(R, q1(1), q21(2), p);
q321 = calcular_q3(R, q1(2), q22(1), p);
q322 = calcular_q3(R, q1(2), q22(2), p);

qq(1,:) = [q1(1)  q1(1)  q1(1)  q1(1)  q1(2)  q1(2)  q1(2)  q1(2)];
qq(2,:) = [q21(1) q21(1) q21(2) q21(2) q22(1) q22(1) q22(2) q22(2)];
qq(3,:) = [q311   q311   q312   q312   q321   q321   q322   q322];


for i=1:8
    Taux = eye(4);
    for j=1:3
        Taux = Taux * R.links(j).A(qq(j,i)).double;
    end
    Taux = Taux * R.links(4).A(0).double;
    error_pos = norm(Taux(1:3,4) - p);
end

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

soluciones_validas = true(1, 8);

for i = 1:8
    for j = 1:6
        if qq(j,i) < R.qlim(j,1) || qq(j,i) > R.qlim(j,2)
            soluciones_validas(i) = false;
            break;
        end
    end
end

% sol validas
qq = qq(:, soluciones_validas);
n_soluciones = sum(soluciones_validas);

if n_soluciones == 0
    error('Ninguna solución respeta los límites articulares del robot');
end


for i = 1:n_soluciones
    Taux = R.fkine(qq(:,i));
    if isa(Taux, 'SE3')
        Taux = Taux.double;
    end
    error_pos = norm(Taux(1:3,4) - T(1:3,4));
    error_rot = norm(Taux(1:3,1:3) - T(1:3,1:3), 'fro');
end


if mejor
    if size(qq, 2) == 0
        error('No hay soluciones dentro de límites articulares');
    end
    
    % *** CORRECCIÓN: usar repmat para manejar q0 como fila o columna ***
    Qaux = qq - repmat(q0(:), 1, size(qq,2));
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

%% FUNCIONES AUXILIARES

function iT = invHomog(T)

    iT = eye(4);
    iT(1:3, 1:3) = T(1:3, 1:3)';
    iT(1:3, 4) = -iT(1:3, 1:3) * T(1:3, 4);
end

%=========================================================================%

function q1 = calcular_q1(p)

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
    
    T1 = R.links(1).A(q1).double;
    p = invHomog(T1) * [p; 1];
    p = p(1:3);
    
    
    a2 = R.links(2).a;
    a3 = R.links(3).a;
    d4 = R.links(4).d;
    L = sqrt(a3^2 + d4^2);
    
    
    r = sqrt(p(1)^2 + p(2)^2);
    beta = atan2(p(2), p(1));
    cos_alpha = (a2^2 + r^2 - L^2) / (2 * a2 * r);
    
    % Verificar si la posición es alcanzable
    if abs(cos_alpha) > 1
        if abs(cos_alpha) > 1.01  % Tolerancia de 1%
            error('Posición FUERA DE ALCANCE. cos(alpha) = %.4f (debe estar en [-1,1])', cos_alpha);
        else
            %
            cos_alpha = sign(cos_alpha);  
        end
    end
    
    alpha = acos(cos_alpha);
    
    % Verificar si alpha es real
    if imag(alpha) ~= 0
        error('Ángulo alpha resultó imaginario. Posición inalcanzable.');
    end
    
    q2(1) = beta + alpha;
    q2(2) = beta - alpha;
end

%=========================================================================%

function q3 = calcular_q3(R, q1, q2, p)

    % Transformar p al sistema {S2}
    T1 = R.links(1).A(q1).double;
    T2 = T1 * R.links(2).A(q2).double;
    p = invHomog(T2) * [p; 1];
    p = p(1:3);
    
    a3 = R.links(3).a;
    d4 = R.links(4).d;
    
    gamma = atan2(d4, a3);
    theta = atan2(p(2), p(1));
    
    % Verificar que theta sea real
    if imag(theta) ~= 0
        error('Ángulo theta resultó imaginario en q3.');
    end

    q3 = theta - gamma;
end

%=========================================================================%

function q_unwrapped = unwrap_angle(q, q_ref, q_min, q_max)
    
    opciones = q + 2*pi*(-3:3);
    
    validas = opciones(opciones >= q_min & opciones <= q_max);
    
    if isempty(validas)
        q_unwrapped = q;  
        return;
    end
    
    [~, idx] = min(abs(validas - q_ref));
    q_unwrapped = validas(idx);
end

%=========================================================================%

function [q4, q5, q6] = calcular_qm(R, q1, q2, q3, T, q0)

    T1 = R.links(1).A(q1).double;
    T2 = R.links(2).A(q2).double;
    T3 = R.links(3).A(q3).double;
    
    T36 = invHomog(T3) * invHomog(T2) * invHomog(T1) * T;
    
    q0 = q0(:);
    
    if abs(T36(3,3) - 1) < 1e-6
        q4(1) = q0(4);
        q5(1) = 0;
        
        q6_temp = atan2(T36(2,1), T36(1,1)) - q4(1);
        q6(1) = unwrap_angle(q6_temp, q0(6), R.qlim(6,1), R.qlim(6,2));
        
        q4(2) = q4(1);
        q5(2) = 0;
        q6(2) = q6(1);
    else
        q4_temp = atan2(T36(2,3), T36(1,3));
        q4(1) = unwrap_angle(q4_temp, q0(4), R.qlim(4,1), R.qlim(4,2));
        
        if q4_temp > 0
            q4_temp2 = q4_temp - pi;
        else
            q4_temp2 = q4_temp + pi;
        end
        q4(2) = unwrap_angle(q4_temp2, q0(4), R.qlim(4,1), R.qlim(4,2));
        
        q5 = zeros(1,2);
        q6 = zeros(1,2);
        
        for i=1:2
            T4 = R.links(4).A(q4(i)).double;
            T46 = invHomog(T4) * T36;
            
            q5_temp = atan2(T46(2,3), T46(1,3)) - pi/2;
            q5(i) = unwrap_angle(q5_temp, q0(5), R.qlim(5,1), R.qlim(5,2));

            T5 = R.links(5).A(q5(i)).double;
            T56 = invHomog(T5) * T46;
            
            q6_temp = atan2(T56(2,1), T56(1,1));
            q6(i) = unwrap_angle(q6_temp, q0(6), R.qlim(6,1), R.qlim(6,2));
        end
    end

end
