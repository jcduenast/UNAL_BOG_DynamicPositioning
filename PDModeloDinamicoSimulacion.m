cd('C:\Users\juanc\Documents\MATLAB\PD'); % Cambia el workspace
%% Inicializar descriptores de la simulaci�n
clc; clear;             % Limpia el espacio de trabajo
L1=0.120;               % Descripci�n de la disposici�n geom�trica de los
L2=0.180;               % motores con respecto al centro de masa.
m=13;                   % Masa del barco en Kg
Iz=0.296606;            % Momento de inercia respecto al eje Z en el centro 
                        % de masa en kg/m^2
dx=2; dy=2; dz=0.7;     % Establecimiento de factores de amortiguamiento;
                        % a�n se requiere validaci�n experimental.
F= [3 0 0.3]';          % Vector de fuerzas deseado bajo un hipot�tico caso
                        % de acci�n de control
T= [1 0 -1 0            % Matriz T del modelo #1 con direcci�n de motores
    0 1 0 -1            % fija; primera aproximaci�n.
    L1 L2 L1 L2];
Ts = 0.1;               % Tiempo entre muestras
seg = 6;                % Cantidad de segundos de simulaci�n
t = 0:Ts:seg;           % Vector de tiempo para simulaci�n
plotArea = [-2 2 -2 2]; % L�mites de ejes en el gr�fico
%% Simulaci�n
Pg=[t; t; t];           % Inicializaci�n del vector de posiciones global
V=Pg;                   % Inicializaci�n del vector de velocidades local
A=V;                    % Inicializaci�n del vector de aceleraciones local

Fm = invKin1(F(1), F(2), F(3)); % Soluci�n de la cinem�tica inversa de 
                        % acuerdo a la disposici�n #1 descrita en T
Tau=T*Fm;               % Generaci�n del vector tau de fuerzas

Pg(:,1)=[0 0 0]';       % Inicializaci�n en posici�n cero
V(:,1)=[0 0 0]';        % Inicializaci�n en velocidad cero
Vg=V;                   % Inicializaci�n de velocidad global
Ag=A;                   % Inicializaci�n de aceleraci�n global

D= [-dx 0 0             % Matriz de amortiguamiento, fricci�n viscosa
    0 -dy 0
    0 0 -dz];

M =[m 0 0               % Matriz de inercia
    0 m 0
    0 0 Iz];

% Primer vector de aceleracion
A(:,1)=inv(M)*(T*Fm+D*V(:,1));
Ag=A;

for i = 2:length(t)
    A(:,i)=inv(M)*(T*Fm+D*V(:,i-1));    % Aceleraci�n local
    V(:,i)=V(:,i-1)+A(:,i-1)*Ts;        % Velocidad local
    Pg(3,i)=Pg(3,i-1)+V(3,i-1)*Ts+A(3,i-1).*A(3,i-1)*(Ts/2); % Rotaci�n
    Vg(3,i)=V(3,i);                     % Velocidad angular
    Vg(1,i)=V(1,i)*cos(Pg(3,i-1))+V(2,i)*sin(Pg(3,i-1)); % Velocidad X global
    Vg(2,i)=V(1,i)*sin(Pg(3,i-1))+V(2,i)*cos(Pg(3,i-1)); % Velocidad Y global
    Ag(3,i)=A(3,i);                     % Aceleraci�n angular
    Ag(1,i)=A(1,i)*cos(Pg(3,i-1));      % Aceleraci�n X global
    Ag(2,i)=A(2,i)*sin(Pg(3,i-1));      % Aceleraci�n X global
    Pg(:,i)=Pg(:,i-1)+Vg(:,i-1)*Ts+A(:,i-1).*A(:,i-1)*(Ts/2); % Posici�n
end
%% Gr�fica trayectoria/orientaci�n
j=6;
figure ('rend','painters','pos',[10 10 900 500])    % Par�metros de la figura
quiver(Pg(1,1),-Pg(2,1),cos(Pg(3,1)),-sin(Pg(3,1)),'b');
hold on
axis(plotArea)
for f = 1:length(t)/j
    quiver(Pg(1,j*f),-Pg(2,j*f),cos(Pg(3,j*f)),-sin(Pg(3,j*f)),'b');
end
plot(Pg(1,:),-Pg(2,:),'-r.')
title('Posici�n Absoluta')
xlabel('Eje X [m]')
ylabel('Eje Y [m]')

%% Gr�ficas de Aceleraci�n Local, Velocidad Local y Posici�n en el tiempo
figure ('rend','painters','pos',[10 10 900 500])
subplot(3,1,1);
plot(t,A)
title('Aceleraci�n Local')
legend('X','Y','R','Location','northeast')
subplot(3,1,2); 
plot(t,V)
title('Velocidad Local')
legend('X','Y','R','Location','northeast')
subplot(3,1,3); 
plot(t,Pg)
title('Posici�n Absoluta')
legend('X','Y','R','Location','northeast')