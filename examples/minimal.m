clear all;
close all;

E = 210e9;                          % Young's modulus (for steel)
rho = 7850;                         % Density
rho_dh = 7900;                      % Density horizontal diagonals
nu = 0.3;                           % Poisson ratio
G = E/(2*(1+nu));                   % Shear modulus

w_v     = 0.3;      % Vertical column cross sectional width
w_h     = 0.2;      % Horizontal bar cross sectional width
h_h     = 0.12;     % Horizontal bar cross sectional height
w_dh    = 0.01;
h_dh    = 0.06;
w_dv    = 0.012;
h_dv    = 0.15;
 
t_v1    = 0.01;     % Lower column wall thickness
t_v2    = 0.008;    % Upper column wall thickness
t_h     = 0.008;    % Horizontal bar wall thickness

h_floor = 3;
w_floor = 4.75;

L_v  = h_floor;           % Length of vertical columns
L_h  = w_floor;    
L_dv = sqrt((3*h_floor)^2 + w_floor^2);
L_dh = sqrt(2*w_floor^2);

% Cross sectional area
A_v = w_v^2 - (w_v - 2*t_v1)^2;
A_h  = 2*((w_h * h_h) - (w_h - 2*t_h)*(h_h - 2*t_h));
A_dh = w_dh * h_dh;
A_dv = w_dv * h_dv;

myu_v = A_v * rho;
myu_h  = A_h * rho; 
myu_dh = A_dh * rho_dh;
myu_dv = A_dv * rho;

% 2nd moment of area
I_v = 1/12 * (w_v^4 - (w_v - 2*t_v1)^4);

% Torsion constant
It_v = w_v*(w_v-t_v1)^3;

%% Node table
n_stories = 3;
nodes = zeros(n_stories+1*4, 3);
for i = 0:n_stories
    nodes(i*4+1:(i+1)*4,:) = [0 0 i*h_floor; w_floor 0 i*h_floor; w_floor w_floor i*h_floor; 0 w_floor i*h_floor];
end
%% Elements
elems = [1 9 1; 2 10 1; 3 11 1; 4 12 1; ...    % Vertical columns
    9 13 1; 10 14 1; 11 15 1; 12 16 1; ...
    9 10 2; 10 11 2; 11 12 2; 12 9 2; ...      % Horizonzal bars
    13 14 2; 14 15 2; 15 16 2; 16 13 2; ...
    9 11 3; 10 12 3; ...                       % Horizontal diagonals
    13 15 3; 14 16 3; ... 
    1 14 4; 2 13 4; 2 15 4; 3 14 4; ...    % Vertical diagonals
    3 16 4; 4 15 4; 4 13 4; 1 16 4];      

vis_high_rise_building(nodes,elems)

%% Element types
% Euler-Bernouli beam
column_v    = PH_FEM_Beam(4, 2, 2, myu_v, E, A_v, G, It_v, I_v, I_v, L_v);
% Rod elements
bar_h       = PH_FEM_Link(2, myu_h, E, A_h, L_h); 
bar_dh      = PH_FEM_Link(2, myu_dh, E, A_dh, L_dh);
diagonal    = PH_FEM_Link(2, myu_dv, E, A_dv, L_dv); 


%% FEM mesh
mesh = PH_FEM_mesh(nodes, elems, {column_v, bar_h, bar_dh, diagonal});
%% Lock DOFs of lowermost nodes
mesh.fixNodeDOFs(nodes(1,:), [1 1 1 0 0 1]);
mesh.fixNodeDOFs(nodes(2,:), [1 1 1 0 0 1]);
mesh.fixNodeDOFs(nodes(3,:), [1 1 1 0 0 1]);
mesh.fixNodeDOFs(nodes(4,:), [1 1 1 0 0 1]);


% Add Rayleigh damping 
mesh.addRayleighDamping(0.05, 0.005);
% Add external forces acting on global DOFs
mesh.addExternalInputsAtNodes();
% Generate constraints
mesh.generateConstraints();
% Assemble constraint matrix B
mesh.assembleDAESystem();
% Eliminate algebraic constraints
mesh.eliminateAlgebraicConstraints();
% Eliminate linear dependencies between state/effort variables
mesh.eliminateLinearDependencies();
% Transform coordinate space to global DOFs
mesh.transformToGlobalDOFs();


% Get mass, stiffness and damping matrix
M_ph = mesh.Q(1:mesh.n/2, 1:mesh.n/2)^-1;
K_ph = mesh.Q(mesh.n/2+1:end, mesh.n/2+1:end);
D_ph = mesh.R(1:mesh.n/2, 1:mesh.n/2);



%% Modal Analysis
highestMode = 10;
% Get the eigenvalues
lambda = mesh.getSmallestMagnitudeEigenvalues(highestMode);
omega_Hz_ph = sqrt(diag(lambda))/2/pi;


%% Simulation

% Add Rayleigh damping
D_ph = M_ph * 0.05 + K_ph * 0.005;
J = mesh.J;
Q = mesh.Q; 
R = mesh.R;

A = (J-R)*Q;

% Initial displacement due to wind in x-direction
n_dofs = mesh.n/2;
wind_dofs_x = sort([(5:4:49).*6-5 (8:4:52).*6-5]-24+8);
E_wind = zeros(n_dofs, 1);
for i=1:3
    E_wind(wind_dofs_x(2*i-1), 1) = i/12;
    E_wind(wind_dofs_x(2*i), 1) = i/12;
end
E_wind = [E_wind; zeros(mesh.n/2, 1)];
E_wind = [E_wind(1:length(A(:,1)),1)];
x0_ph = -A\E_wind*0.2e5;
%%
time = 1:0.01:5; 
odefun_ph = @(t, x) A*x;
jacobian_ph = @(t, x) A;

y_ph = linear_gls(odefun_ph, jacobian_ph, time, x0_ph, 2);

%%

y_ph_topNode1 = y_ph(:,end);
y_ph_topNode2 = y_ph(:,end-1);
y_ph_topNode3 = y_ph(:,end-2);
y_ph_topNode4 = y_ph(:,end-3);

figure
hold all
plot(time,y_ph_topNode1)
plot(time,y_ph_topNode2)
plot(time,y_ph_topNode3)
plot(time,y_ph_topNode4)






