% Archivo: robot_control.m

clear all;
clc;
close all;

% Conexión con Arduino
ard = arduino('COM5');
servo1 = servo(ard, 'D2'); % Joint 1
servo2 = servo(ard, 'D3'); % Motor 2 (Joint 2 - Principal)
servo3 = servo(ard, 'D4'); % Motor 3 (Joint 2 - Complementario)
servo4 = servo(ard, 'D5'); % Motor 4 (Joint 3)
servo5 = servo(ard, 'D6'); % Motor 4 (Joint 3)
servo6 = servo(ard, 'D7'); % Joint 5
servo7 = servo(ard, 'D8'); % Motor 4 (Joint 3)
servo8 = servo(ard, 'D9'); % Joint 6

% Configuración de la comunicación por socket
t = tcpserver('localhost', 12345);
disp("Esperando datos...");
pause(1);

% Configuración de parámetros
camWidth = 32;  
camHeight = 22; 
pixWidth = 640; 
pixHeight = 480; 

scaleX = camWidth / pixWidth;
scaleY = camHeight / pixHeight;

zFixed = 8;

% Inicializar almacenamiento de ángulos
angulo1 = []; 
angulo2 = []; 
angulo3 = []; 
angulo4 = []; 
angulo5 = []; 
angulo6 = [];

% Límites de rotación (en radianes)
jointLimits = [
     -pi/2   ,  pi/2;  % Joint 1
      0   ,  2*pi/3;  % Joint 2
     -pi/2   ,  pi/2;  % Joint 3
     -pi   ,  0;       % Joint 4
     -pi/2   , pi/2;   % Joint 5
     -pi/2   ,  pi/2   % Joint 6
];

% Desplazamientos angulares para compensar offset físico
offsetAngles = [120, -30, 190, 0, 180, 90]; % Valores en grados (modifica según tus servos)

% Construcción del modelo del robot
scale = 100; 
d1 = 0.051 * scale; 
a2 = 0.1415 * scale;
a3 = 0.07 * scale;
d2 = 0.03785 * scale;
d3 = -0.015 * scale;
a4 = 0.07 * scale;
EF = 0.096 * scale;

dhparams = [0    0        d1     0;
            0    pi/2     0      0;
            a2   0        0      0;
            a3   0        0      0;
            0   -pi/2     d2     0;
            0    pi/2     d3     0;
            EF   0        0      0];

paperrobot = rigidBodyTree;
bodies = cell(7, 1);
joints = cell(7, 1);

for i = 1:7
    if i == 7
        bodies{i} = rigidBody("endeffector");
        joints{i} = rigidBodyJoint(['jnt' num2str(i)], "fixed");
    else
        bodies{i} = rigidBody(['body' num2str(i)]);
        joints{i} = rigidBodyJoint(['jnt' num2str(i)], "revolute");
        
        % Aplicar límites de rotación
        if i <= size(jointLimits, 1)
            joints{i}.PositionLimits = jointLimits(i, :);
        end
    end
    setFixedTransform(joints{i}, dhparams(i, :), "mdh");
    bodies{i}.Joint = joints{i};
    if i == 1
        addBody(paperrobot, bodies{i}, "base");
    else
        addBody(paperrobot, bodies{i}, bodies{i - 1}.Name);
    end
end

% Recepción de datos para la trayectoria
disp('Esperando datos de la trayectoria durante 15 segundos...');
positions = [];
lastData = '';

startTime = tic;
while toc(startTime) < 15
    if t.NumBytesAvailable > 0
        data = readline(t);
        try
            lastData = data;
        catch
            warning('Error al procesar los datos. Intentando nuevamente...');
        end
    end
end

if isempty(lastData)
    error('No se recibieron datos válidos para la trayectoria.');
end

try
    posicion = jsondecode(lastData);
    disp("Datos recibidos:");
    disp(posicion);
    for i = 1:size(posicion.Posiciones_, 1)
        Vx = (posicion.Posiciones_(i, 1) * scaleX - 16);
        Vy = (posicion.Posiciones_(i, 2) * scaleY + 18);
        newX = Vy;
        newY = Vx;
        newZ = zFixed;
        positions = [positions; newX, newY, newZ];
    end
catch
    error('Error al procesar los datos después de 15 segundos.');
end

if isempty(positions)
    error('No se generaron posiciones válidas para la trayectoria.');
end

% Crear la trayectoria
trayectoria = [];
for i = 1:(size(positions, 1) - 1)
    temp_traj = trajC(positions(i, :), positions(i + 1, :), 100); 
    trayectoria = [trayectoria; temp_traj];
end
trayectoria = trayectoria';

% Crear la trayectoria con menos puntos
paso = 5; % Seleccionar un punto cada 5 (ajusta según lo necesario)
trayectoriaReducida = trayectoria(:, 1:paso:end);

% Inicializar la figura para la simulación del movimiento
figRobot = figure('Name', 'Simulación del Robot');
ax = axes(figRobot);
hold on;
show(paperrobot, 'Parent', ax);
plot3(ax, trayectoria(1, :), trayectoria(2, :), trayectoria(3, :), 'b--', 'LineWidth', 2);
title('Simulación del Movimiento del Robot');
xlabel('X');
ylabel('Y');
zlabel('Z');
xlim([-60 60]);
ylim([-60 60]);
zlim([-5 25]);
grid on;

ik = inverseKinematics('RigidBodyTree', paperrobot);
weights = [0.01 0.01 0.01 0.01 0.01 0.01];
initialGuess = paperrobot.homeConfiguration;

rx = pi/2;
R_x = [cos(rx) 0 sin(rx); 0 1 0; -sin(rx) 0 cos(rx)];

for i = 1:size(trayectoriaReducida, 2)
    % Generar la matriz de transformación
    tform = trvec2tform([trayectoriaReducida(1, i), trayectoriaReducida(2, i), trayectoriaReducida(3, i)]);
    tform(1:3, 1:3) = R_x;

    % Resolver la cinemática inversa
    [configSoln, solnInfo] = ik("endeffector", tform, weights, initialGuess);
    initialGuess = configSoln; 

    % Actualizar la visualización del robot
    cla(ax); % Limpiar el gráfico para evitar múltiples posiciones del robot
    plot3(ax, trayectoria(1, :), trayectoria(2, :), trayectoria(3, :), 'b--', 'LineWidth', 2);
    show(paperrobot, configSoln, 'Parent', ax);
    drawnow;
    
    % Extraer ángulos de las juntas
    currentAngles = arrayfun(@(c) rad2deg(c.JointPosition), configSoln);

    % Controlar los servos
    for joint = 1:6
        angle = currentAngles(joint);

        % Joint 2: Controlado por Servo2 y Servo3 (Complementario)
        if joint == 2
            angle = 180 - angle; % Ajuste para invertir el sentido
            angle = angle + offsetAngles(joint); % Compensación física
            complementAngle = 270 - angle; % Complemento del motor 3
            complementAngle1 = 180 - angle;
        elseif joint == 3
            angle = angle + offsetAngles(joint); % Joint 3 usa Servo4
        else
            angle = angle + offsetAngles(joint); % Ajustar otros joints
        end

        % Mapear al rango del servo
        servoValue = max(0, min(angle / 270, 1)); % Normalización para servos de 270°

        % Enviar valores a los servos
        switch joint
            case 1, writePosition(servo1, servoValue);
            case 2
                writePosition(servo3, servoValue); % Servo2 (Joint 2)
                writePosition(servo2, max(0, min(complementAngle / 270, 1))); % Servo3 (Complemento)
            case 3, writePosition(servo4, servoValue); % Servo4 (Joint 3)
            case 4, writePosition(servo5, servoValue);
            case 5, writePosition(servo6, servoValue);
            case 6
                writePosition(servo7, max(0, min(complementAngle1 / 180, 1)));
                writePosition(servo8, 0);
        end

        fprintf('Joint %d: %.2f° -> Servo Value: %.2f\n', joint, angle, servoValue);
    end

    % Reducir la pausa para aumentar velocidad (ajusta este valor)
    pause(0.05); % Pausa más corta para mayor rapidez
end


%% Función para generar trayectorias
function [trayectoria] = trajC(A, B, N)
    t = linspace(0, 1, N)';
    trayectoria = (1 - t) .* A + t .* B;
end