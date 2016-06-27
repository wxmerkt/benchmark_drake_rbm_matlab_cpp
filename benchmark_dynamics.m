%% Construct RBM
clear all; clc;
warning('off','all');

tic;
options = struct();
options.terrain = [];
options.floating = false;
options.base_offset = [0.0, 0.0, 0.0]';
options.base_rpy = [0.0, 0.0, 0.0]';

urdf = '/home/wxm/oh-distro/software/models/val_description/urdf/valkyrie_sim_drake.urdf';

r = RigidBodyManipulator(urdf, options);
r = compile(r);
toc

disp(['RigidBodySystem has ', num2str(r.getNumStates), ' states and ', num2str(r.getNumInputs), ' inputs'])

%% Run Benchmark
x0 = zeros(r.getNumStates, 1);
u = ones(r.getNumInputs, 1);

tic;
for i=1:1000
    x0(1:r.num_positions) = getRandomConfiguration(r);
    xdot = r.dynamics(0, x0, u);
end
toc