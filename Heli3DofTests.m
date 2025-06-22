theta = deg2rad(0);    % roll
alpha = deg2rad(0);    % pitch
psi   = deg2rad(0);    % yaw
Ff = 2.5;               % fuerza motor frontal
Fb = 1.0;               % fuerza motor trasero
L_arm = 0.1;           % longitud del brazo helicóptero
L_base = 1;           % altura del poste
L_yaw = 1;            % longitud del balancín
% Definir masas
m_motor = 0.071;      % masa de cada motor            [kg]
m_red   = 0.180;      % masa del brazo rojo completo  [kg]
m_green = 0.220;      % masa del balancín verde       [kg]

J_roll  = 1.85e-3;    % constante: inercia vista por el motor de roll
J_pitch = 2.10e-3;    % constante: inercia vista por el motor de pitch
% fkine
pos = f_kine_Heli3Dof(theta, alpha, psi, L_arm, L_base, L_yaw)
[pos.motor_R pos.motor_L pos.cm_A pos.cm_B pos.cp pos.base]
Jz = f_inertiaYaw_Heli3Dof(pos, m_motor, m_green, m_red)
plot_heli3Dof(pos)


%% 
% $$J_x \ddot{\theta} =\frac{\left(F_R -F_L \right)}{2R}$$
% 
% $$J_y \ddot{\phi} =\frac{F_R +F_L }{L}\cos \left(\theta \right)-\left({\mathrm{m}}_{\textrm{cp}} 
% -m_m \right)\textrm{gcos}\left(\phi \right)$$
% 
% $$J_z \ddot{\psi} =\frac{F_R +F_L }{L}\sin \left(\theta \right)\cos \left(\phi 
% \right)$$
% 
% Nota: $J_z$ puede variar con $\phi$
% 
% $$\left\lbrack \begin{array}{c}x_1 \\x_2 \\x_3 \\x_4 \\x_5 \\x_6 \end{array}\right\rbrack 
% =\left\lbrack \begin{array}{c}\theta \\\dot{\theta} \\\phi \\\dot{\phi} \\\psi 
% \\\dot{\psi} \end{array}\right\rbrack$$
% 
% $$\left\lbrack \begin{array}{c}\dot{x_1 } \\\dot{x_2 } \\\dot{x_3 } \\\dot{x_4 
% } \\\dot{x_5 } \\\dot{x_6 } \end{array}\right\rbrack =\left\lbrack \begin{array}{c}x_2 
% \\\frac{u_1 -u_2 }{2J_X R}\\x_4 \\\frac{u_1 +u_2 }{J_Y L}\cos \left(x_1 \right)-\frac{\left(m_{\textrm{cp}} 
% -m_m \right)g}{J_Y }\cos \left(x_3 \right)\\x_5 \\\frac{u_1 +u_2 }{J_Z L}\sin 
% \left(x_1 \right)\cos \left(x_3 \right)\end{array}\right\rbrack$$

dt = 0.01;
t = 0:dt:3;
x = zeros(6,length(t));
u = zeros(2,length(t));
phi_d = pi/4;
psi_d = pi/2;
L_arm = 0.1;
L_base = 0.5;
L_yaw = 1;
Kp = 5;
Kd = 2.5;
for i = 2:length(t)
    u(:,i) = Kp*[phi_d-x(3,i-1); phi_d-x(3,i-1)] + Kd*[-x(4,i-1); -x(4,i-1)];
    x(:,i) = x(:,i-1) + Heli3DoF(t(i),x(:,i-1),u(:,i))*dt;
    plot_heli3DoF(x(1,i),x(2,i),x(3,i), u(1,i), u(2,i), L_arm, L_base, L_yaw)
    pause(0)
end
figure()
plot(t,x)
%%
function dx = Heli3DoF(~,x,u)
Jx = 1;
Jy = 1;
Jz = 1;
R = 0.25;
L = 0.5;
mcp = 2;
mm = 2.5;
g = 9.8;
dx(1) = x(2);
dx(2) = (u(1)-u(2))/(2*Jx*R);
dx(3) = x(4);
dx(4) = (u(1)+u(2))/(Jy*L)*cos(x(1))-(mm-mcp)*g/Jy*cos(x(3));
dx(5) = x(6);
dx(6) = (u(1)+u(2))/(Jz*L)*sin(x(1))*cos(x(3));
dx = dx';
end

function pos = f_kine_Heli3Dof(theta, alpha, psi, L_A, L_B, L_C)
% f_kine_Heli3Dof: cinemática directa del helicóptero 3DOF
% Entradas:
%   theta: roll [rad]
%   alpha: pitch [rad]
%   psi: yaw [rad]
%   L_A: longitud del brazo rojo (helicóptero)
%   L_B: longitud del brazo verde (balancín)
%   L_C: longitud del poste vertical (base)
%
% Salida:
%   pos.motor_R: posición motor derecho (3x1)
%   pos.motor_L: posición motor izquierdo (3x1)
%   pos.cm_A: centro masa brazo rojo (3x1)
%   pos.cm_B: centro masa brazo verde (3x1)
%   pos.cp: posición contrapeso (punta opuesta a los motores)
%   pos.base: punto base del eje de yaw (inicio del balancín)

% --- Cálculo de referencias ---
Rz = rotz(psi);
Ry = roty(alpha);
green_vector = Rz*Ry*[1; 0; 0];

% Posición base de rotación green (75% del poste)
pivot_green = [0; 0; 0.75 * L_C];

% Puntos extremos del balancín
green_start = pivot_green - (L_B/2) * green_vector;  % hacia atrás (contrapeso)
green_end   = pivot_green + (L_B/2) * green_vector;  % hacia adelante (helicóptero)

% Dirección perpendicular al balancín
red_axis = [0; 1; 0];

% Rotaciones del brazo rojo
Rx = rotx(theta);
arm_vector = Rz*Ry*Rx*red_axis;

% Posiciones de los motores
pos.motor_R = green_end + L_A * arm_vector;
pos.motor_L = green_end - L_A * arm_vector;

% Centro de masa del brazo rojo (centrado en yaw_end)
pos.cm_A = green_end;

% Centro de masa del brazo verde (en el centro del balancín)
pos.cm_B = pivot_green;

% Contrapeso (en el extremo opuesto al brazo rojo)
pos.cp = green_start;

% Base
pos.base = pivot_green;

end

function plot_heli3Dof(pos)
% plot_heli3Dof  Visualiza la configuración de un helicóptero 3-DOF
%
%   plot_heli3Dof(pos)
%
%   pos : estructura con los campos generados por f_kine_Heli3Dof
%         ├─ pos.motor_R (3×1)  – motor derecho
%         ├─ pos.motor_L (3×1)  – motor izquierdo
%         ├─ pos.cm_A    (3×1)  – unión balancín–helicóptero
%         ├─ pos.cm_B    (3×1)  – centro del balancín (= pivote)
%         ├─ pos.cp      (3×1)  – contrapeso
%         └─ pos.base    (3×1)  – extremo superior del poste
%
%   El gráfico representa:
%     • Poste azul (base fijo)
%     • Balancín verde (pitch)
%     • Brazo rojo del helicóptero (roll)

%----- Ventana y ejes -----------------------------------------------------
figure('Name','Helicóptero 3-DOF','Position',[100 100 600 600]);
hold on; grid on; axis equal; view(3);
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
title('Helicóptero 3-DOF');

%----- Poste azul ---------------------------------------------------------
plot3([0 0],[0 0],[0 pos.base(3)/0.75],'b-','LineWidth',4);

%----- Balancín verde -----------------------------------------------------
plot3([pos.cp(1) pos.cm_A(1)], ...
      [pos.cp(2) pos.cm_A(2)], ...
      [pos.cp(3) pos.cm_A(3)], ...
      'g-','LineWidth',3);

%----- Brazo rojo (cuerpo helicóptero) ------------------------------------
plot3([pos.motor_L(1) pos.motor_R(1)], ...
      [pos.motor_L(2) pos.motor_R(2)], ...
      [pos.motor_L(3) pos.motor_R(3)], ...
      'r-','LineWidth',3);

% Puntos destacados (opcional; puedes comentar si no los quieres)
scatter3(pos.base(1),  pos.base(2),  pos.base(3), 40,'b','filled'); % pivote poste
scatter3(pos.base(1),  pos.base(2),  pos.base(3)/0.75, 40,'b','filled'); % punta poste
scatter3(pos.cp(1),    pos.cp(2),    pos.cp(3),   40,'g','filled'); % contrapeso
scatter3(pos.cm_A(1),  pos.cm_A(2),  pos.cm_A(3), 40,'g','filled'); % unión verde-rojo
scatter3(pos.motor_L(1),pos.motor_L(2),pos.motor_L(3),40,'r','filled');
scatter3(pos.motor_R(1),pos.motor_R(2),pos.motor_R(3),40,'r','filled');

%----- Ajuste automático de límites para centrar la vista ----------------
xlim([-1 1]*norm(pos.cm_A));
ylim([-1 1]*norm(pos.cm_A));
zlim([ 0 1]*pos.base(3)/0.75);

drawnow;
end

function Jz = f_inertiaYaw_Heli3Dof(pos, m_motor, m_green, m_red)
% f_inertiaYaw_Heli3Dof  Inercia variable respecto al eje Z (yaw)
%
%   Jz = f_inertiaYaw_Heli3Dof(pos)
%
%   • pos : estructura devuelta por f_kine_Heli3Dof
%   • Jz  : momento de inercia total visto por el motor de yaw [kg·m²]
%
%   NOTA
%   ----
%   - Las masas y las inercias constantes de roll y pitch se definen
%     aquí como parámetros de usuario y, en la práctica, se colocarían
%     en un script de configuración o en un fichero de parámetros.

% Utilidad interna: inercia de una barra uniforme (ver ecuación arriba)
rodI = @(p1,p2,m) (m/3) * ( ...
      sum(p1.^2) + dot(p1,p2) + sum(p2.^2) );

% --- Todas las coordenadas relativas al eje de giro (pos.base) ----------
o = pos.base(1:2);              % origen del eje Z
% Motores
rR = pos.motor_R(1:2) - o;
rL = pos.motor_L(1:2) - o;

% Eslabones
%  - verde   : contrapeso (cp)  ↔  cm_A   (pivote brazo rojo)
%  - rojo    : motor_L          ↔  motor_R
r_cp   = pos.cp(1:2)     - o;
r_cm_A = pos.cm_A(1:2)   - o;

% --- Suma de contribuciones ---------------------------------------------
Jz  = m_motor*(rR(1)^2 + rR(2)^2) ...
    + m_motor*(rL(1)^2 + rL(2)^2) ...
    + rodI(r_cp, r_cm_A, m_green) ...
    + rodI(rL,  rR,     m_red);
end

function R = rotx(a)
R = [1 0 0; 0 cos(a) -sin(a); 0 sin(a) cos(a)];
end

function R = roty(a)
R = [cos(a) 0 sin(a); 0 1 0; -sin(a) 0 cos(a)];
end

function R = rotz(a)
R = [cos(a) -sin(a) 0; sin(a) cos(a) 0; 0 0 1];
end