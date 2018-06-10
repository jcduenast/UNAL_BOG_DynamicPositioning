cd('C:\Users\juanc\Documents\MATLAB\PD'); % Cambia el workspace
%% Inicializar descriptores de la simulación
clc; clear;             % Limpia el espacio de trabajo
L1=0.120;               % Descripción de la disposición geométrica de los
L2=0.180;               % motores con respecto al centro de masa.
m=13;                   % Masa del barco en Kg
Iz=0.296606;            % Momento de inercia respecto al eje Z en el centro 
                        % de masa en kg/m^2
dx=2; dy=2; dz=0.7;     % Establecimiento de factores de amortiguamiento;
                        % aún se requiere validación experimental.
F= [3 0 0.3]';          % Vector de fuerzas deseado bajo un hipotético caso
                        % de acción de control
T= [1 0 -1 0            % Matriz T del modelo #1 con dirección de motores
    0 1 0 -1            % fija; primera aproximación.
    L1 L2 L1 L2];
Ts = 0.1;               % Tiempo entre muestras
seg = 6;                % Cantidad de segundos de simulación
t = 0:Ts:seg;           % Vector de tiempo para simulación
plotArea = [-2 2 -2 2]; % Límites de ejes en el gráfico
%% Simulación
Pg=[t; t; t];           % Inicialización del vector de posiciones global
V=Pg;                   % Inicialización del vector de velocidades local
A=V;                    % Inicialización del vector de aceleraciones local

Fm = invKin1(F(1), F(2), F(3)); % Solución de la cinemática inversa de 
                        % acuerdo a la disposición #1 descrita en T
Tau=T*Fm;               % Generación del vector tau de fuerzas

Pg(:,1)=[0 0 0]';       % Inicialización en posición cero
V(:,1)=[0 0 0]';        % Inicialización en velocidad cero
Vg=V;                   % Inicialización de velocidad global
Ag=A;                   % Inicialización de aceleración global

D= [-dx 0 0             % Matriz de amortiguamiento, fricción viscosa
    0 -dy 0
    0 0 -dz];

M =[m 0 0               % Matriz de inercia
    0 m 0
    0 0 Iz];

% Primer vector de aceleracion
A(:,1)=inv(M)*(T*Fm+D*V(:,1));
Ag=A;

for i = 2:length(t)
    A(:,i)=inv(M)*(T*Fm+D*V(:,i-1));    % Aceleración local
    V(:,i)=V(:,i-1)+A(:,i-1)*Ts;        % Velocidad local
    Pg(3,i)=Pg(3,i-1)+V(3,i-1)*Ts+A(3,i-1).*A(3,i-1)*(Ts/2); % Rotación
    Vg(3,i)=V(3,i);                     % Velocidad angular
    Vg(1,i)=V(1,i)*cos(Pg(3,i-1))+V(2,i)*sin(Pg(3,i-1)); % Velocidad X global
    Vg(2,i)=V(1,i)*sin(Pg(3,i-1))+V(2,i)*cos(Pg(3,i-1)); % Velocidad Y global
    Ag(3,i)=A(3,i);                     % Aceleración angular
    Ag(1,i)=A(1,i)*cos(Pg(3,i-1));      % Aceleración X global
    Ag(2,i)=A(2,i)*sin(Pg(3,i-1));      % Aceleración X global
    Pg(:,i)=Pg(:,i-1)+Vg(:,i-1)*Ts+A(:,i-1).*A(:,i-1)*(Ts/2); % Posición
end
%% Gráfica trayectoria/orientación
j=6;
figure ('rend','painters','pos',[10 10 900 500])    % Parámetros de la figura
quiver(Pg(1,1),-Pg(2,1),cos(Pg(3,1)),-sin(Pg(3,1)),'b');
hold on
axis(plotArea)
for f = 1:length(t)/j
    quiver(Pg(1,j*f),-Pg(2,j*f),cos(Pg(3,j*f)),-sin(Pg(3,j*f)),'b');
end
plot(Pg(1,:),-Pg(2,:),'-r.')
title('Posición Absoluta')
xlabel('Eje X [m]')
ylabel('Eje Y [m]')

%% Gráficas de Aceleración Local, Velocidad Local y Posición en el tiempo
figure ('rend','painters','pos',[10 10 900 500])
subplot(3,1,1);
plot(t,A)
title('Aceleración Local')
legend('X','Y','R','Location','northeast')
subplot(3,1,2); 
plot(t,V)
title('Velocidad Local')
legend('X','Y','R','Location','northeast')
subplot(3,1,3); 
plot(t,Pg)
title('Posición Absoluta')
legend('X','Y','R','Location','northeast')