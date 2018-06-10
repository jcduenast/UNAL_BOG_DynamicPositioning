function Fm = invKin1(X, Y, N)
L1=0.120;               % Descripción de la disposición geométrica de los
L2=0.180;               % motores.
T= [1 0 -1 0            % Matriz T del modelo #1
    0 1 0 -1
    L1 L2 L1 L2];

if X>0                  % Primer paso de la solución de la IK
    Fno=X;              % Plantea los valores de las fuerzas para lograr
    Fse=0;              % cumplir con los valores de tau en las posiciones
else                    % 1 y 2 que corresponden a las fuerzas en X y Y 
    Fno=0;
    Fse=X;
end
if Y>0
    Fne=Y;
    Fso=0;
else
    Fne=0;
    Fso=Y;
end
Fm=[Fno Fne Fse Fso]';  % Se genera un vector de fuerzas
Tau=T*Fm;               % Fm genera un vector tau que puede no corresponder
                        % con el deseado.
Np=Tau(3);              % Torque generado por Fm
Nr=N-Np;                % Diferencia entre el torque deseado y el obtenido
Fn=Nr/(2*(L1+L2));      % Fuerza adicional en cada motor para generar el
                        % torque requerido

if X>0                  % Ajuste del primer resultado para generar el torque
    Fno=X+Fn;           % requerido
    Fse=Fn;
else
    Fno=Fn;
    Fse=X+Fn;
end
if Y>0
    Fne=Y+Fn;
    Fso=Fn;
else
    Fne=Fn;
    Fso=Y+Fn;
end

Fm=[Fno Fne Fse Fso]';  % Fm corregido
end