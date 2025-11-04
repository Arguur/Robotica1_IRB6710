function enviar_trayectoria_unity(Q_traj, fps, puerto)
% Envía trayectoria a Unity con confirmaciones (ACK)

if nargin < 2, fps = 30; end
if nargin < 3, puerto = 55001; end

fprintf('🔧 Iniciando servidor TCP en puerto %d...\n', puerto);

% Crear servidor TCP
tcpServer = tcpserver("0.0.0.0", puerto, "Timeout", 60);
fprintf('⏳ Esperando conexión de Unity...\n');
fprintf('   (Ejecuta Unity y presiona C para conectar)\n\n');

% Esperar conexión activa
tic;
while tcpServer.NumBytesAvailable == 0 && toc < 60
    pause(0.1);
    if tcpServer.Connected
        break;
    end
end

if ~tcpServer.Connected
    error('❌ Unity no se conectó en 60 segundos');
end

fprintf('✅ Unity conectado!\n');
pause(0.5); % Dar tiempo a Unity para prepararse

fprintf('📤 Enviando %d puntos de trayectoria a %.0f FPS...\n', size(Q_traj, 1), fps);

% Enviar número total de puntos primero
write(tcpServer, sprintf('TOTAL:%d\n', size(Q_traj, 1)), "string");
pause(0.1);

% Enviar datos con confirmaciones
dt = 1/fps;
puntos_enviados = 0;

for i = 1:size(Q_traj, 1)
    % Enviar punto
    mensaje = sprintf('%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n', Q_traj(i,:));
    write(tcpServer, mensaje, "string");
    
    % Esperar confirmación (ACK) con timeout
    timeout = tic;
    while tcpServer.NumBytesAvailable == 0 && toc(timeout) < 2
        pause(0.001);
    end
    
    if tcpServer.NumBytesAvailable > 0
        ack = read(tcpServer, tcpServer.NumBytesAvailable, "string");
        if contains(ack, 'ACK')
            puntos_enviados = puntos_enviados + 1;
        end
    end
    
    % Mostrar progreso cada 10%
    if mod(i, max(1, round(size(Q_traj,1)/10))) == 0
        fprintf('   %.0f%% enviado (%d/%d puntos)\n', ...
            (i/size(Q_traj,1))*100, puntos_enviados, i);
    end
    
    pause(dt);  % Mantener velocidad deseada
end

% Señal de fin
write(tcpServer, "END\n", "string");
fprintf('\n✅ Trayectoria completada: %d/%d puntos recibidos por Unity\n', ...
    puntos_enviados, size(Q_traj, 1));

% Esperar antes de cerrar
pause(1);

% Cerrar conexión
clear tcpServer;
fprintf('🔌 Conexión cerrada.\n');
end