%% a) Incluir rtb al path
addpath(genpath('rvctools')); 

%% b) Matriz DH con parámetros: tita, d, a, alfa, sigma
dh = [
    0, 0.780, 0.320, -pi/2, 0;    % Parámetros DH asociados al Sistema 1
    0, 0.000, 1.185,     0, 0;    % Parámetros DH asociados al Sistema 2
    0, 0.000, 0.200, -pi/2, 0;    % Parámetros DH asociados al Sistema 3
    0, 1.142, 0.000,  pi/2, 0;    % Parámetros DH asociados al Sistema 4
    0,     0, 0.000, -pi/2, 0;    % Parámetros DH asociados al Sistema 5
    0, 0.180, 0.000,     0, 0     % Parámetros DH asociados al Sistema 6
];

%% c) Crear objeto SerialLink R
R = SerialLink(dh);

% c.i) name
R.name = 'IRB 6710-210/2.65';

% c.ii) qlim
R.qlim = [
    deg2rad([-170 170]);  % q1
    deg2rad([-65   85]);  % q2
    deg2rad([-180  70]);  % q3
    deg2rad([-300 300]);  % q4
    deg2rad([-130 130]);  % q5
    deg2rad([-360 360])   % q6
];

% c.iii) offset
R.offset = [0, -pi/2, 0, 0, 0, 0];

% c.iv) base
R.base = transl(0, 0, 0);

% c.v) tool
R.tool = transl(-0.10, 0, 0.98);


%% d) Workspace para ploteo
workspace = [-3 3 -3 3 -0.5 3]; % [-limX, +limX, -limY, +limY, -limZ, +limZ]