
%script ot run MC simulations for various parameter settings

clear;
close all;
%clc;

tic;

%Initialise variables
simulations = 5000;  %00; %000; %00; %50000; % number of Monte Carlo simulations
trials = 100; % number of trials (seconds) per MC simulation

% state is (x,x_dot, x_dotdot) (x-position, velocity and acceleration)
global_vars = struct();
global_vars.delta_t=0.1;



num_objects = 3;
dim = num_objects*3;

%Set number of particles for Particle Filter
num_particles= 5000; %0;


% temporal_params is struct containing mean, transition and noise
% measurement_params is struct containing mean, emission and noise
temporal_params = struct();
temporal_params.mean = 0;

%create the state transition matrix
%tmp = [1,1,0.5;0,1,1];
tmp = [1,global_vars.delta_t, 0.5*(global_vars.delta_t^2); 0,1,global_vars.delta_t];

temporal_params.transition = tmp;
temporal_params.noise_mean =0;

%specify the noise cov.matrix (noise enters through acceleration only)
%x and y cordinates are not affected by "noise" in the temporal model
temporal_params.noise_sigma = 0.09; %001; %0.1;


temporal_params.mean_rev_speed = 0.08; %0.01;
temporal_params.mean_target_vel = 5; %00; %15;
temporal_params.AR_coeff = 0.75; % 0.75;


measurement_params = struct();
measurement_params.mean = 0;

% only x co-ordinate is observed
measurement_params.emission = [1,0];
measurement_params.noise_mean =0;
measurement_params.noise_sigma = 3; %10^10; %3; %10^10;


%car_idx_occlusion =2;


global_vars.safe_dist = 8; %15; %recommended dist between vehicles
global_vars.v_max = 10; %45; %max allowalable velocity
% global_vars.mode = dist_safe; %mode of gamma r.v. (for co-ordinate simulation)
% global_vars.var = 3*global_vars.v_max;  %variance of gamma --- test this!!
%global_vars.scale_fac_measurement_noise_sigma=1;

% global_vars.mean_obj1_accn = 0; % this is mode of acceleration
% global_vars.sigma_obj1_accn = 3; % this is variance of accleration

global_vars.accn_max = 1; %200;%3;
global_vars.accn_min = -2; %global_vars.v_max; %200; %-4;

%Set occlusion probability 
%prob_occl = 1;%unifrnd(0.4, 0.8);

global_vars.num_occlusion_blocks = 0;

global_vars.occlusion_blocs_cord = zeros(global_vars.num_occlusion_blocks,2);
%global_vars.occlusion_blocs_cord(1,:) = [100,150];
%global_vars.occlusion_blocs_cord(2,:) = [300,400];

%enter Id's of cars always occluded throughout.
%should only test models in which num of cars are equal to the number of measurments and 2
%more models which are models of cars with one and 2 extra cars resp.

%For the time being assume that always can see leading and trailing car

global_vars.measurement_id_occlusion=0;



%event is a struct storing event.type and event.occlusion_zone
%     event.type=[];

%  event.type =[-1];
%  event.occlusion_zone = [1];
%  event.car_id = [2];
% 
%  event.type =[1];
%  event.occlusion_zone = [1];
%  event.car_id = [1];
% 

% event.type =[-1, 1];
% event.occlusion_zone = [1, 2];
% event.car_id = [2, 2];


models_to_consider = 3; %[2, 3];

tol = 1e-20; %0.005; %1e-2; %1e-20; %.25;


% % % % overall_car_id__pre_occl_error = zeros(8,1);
% % % % overall_car_id_est1_error = zeros(8,1);
% % % % overall_car_id_est2_error = zeros(8,1);
% % % % overall_car_event_type_error = zeros(8,1);
% % % % 
% % % % overall_dist_error = struct;
% % % % overall_dist_error.run_num_1=zeros(simulations,1);
% % % % overall_dist_error.run_num_2=zeros(simulations,1);
% % % % overall_dist_error.run_num_3=zeros(simulations,1);
% % % % overall_dist_error.run_num_4=zeros(simulations,1);
% % % % overall_dist_error.run_num_5=zeros(simulations,1);
% % % % overall_dist_error.run_num_6=zeros(simulations,1);
% % % % overall_dist_error.run_num_7=zeros(simulations,1);
% % % % overall_dist_error.run_num_8=zeros(simulations,1);


%overall_MSE_particle_filter_timepts = zeros(0,100);
%overall_MSE_mean_filter_full_state_timepts = zeros(0,trials);
%overall_MSE_mahal_filter_full_state_timepts = zeros(0, trials);
 %==================================================================
 
%  err_total=0;error_caught=0;
 
%run simulator script for various values of parameters
%BASE run
fprintf('base run in Progress ...\n');
 event.type=[];
% run_num=1;
simulator_1d_v2_mahal;
%fprintf('Running 1 particle model ...\n');
%simulator_1d_v2_one_vehicle;
%simulator_1d_v2_no_interaction;


% % % 
% % % fprintf('1 car disappearance car 1 in Progress ...\n');
% % % event.type =[-1];
% % % event.occlusion_zone = [1];
% % % event.car_id = [1];
% % % run_num=2;
% % % simulator_1d_v2;
% % % 
% % % 
% % % fprintf('1 car disappearance car 2 in Progress ...\n');
% % % event.type =[-1];
% % % event.occlusion_zone = [1];
% % % event.car_id = [2];
% % % run_num=3;
% % % simulator_1d_v2;
% % % 
% % % 
% % % fprintf('1 car disappearance car 3 in Progress ...\n');
% % % event.type =[-1];
% % % event.occlusion_zone = [1];
% % % event.car_id = [3];
% % % run_num=4;
% % % simulator_1d_v2;
% % % 
% % % 
% % % 
% % % fprintf('1 car appearance car 1 in Progress ...\n');
% % % event.type =[1];
% % % event.occlusion_zone = [1];
% % % event.car_id = [1];
% % % run_num=5;
% % % simulator_1d_v2;
% % % 
% % % 
% % % fprintf('1 car appearance car 2 in Progress ...\n');
% % % event.type =[1];
% % % event.occlusion_zone = [1];
% % % event.car_id = [2];
% % % run_num=6;
% % % simulator_1d_v2;
% % % 
% % % 
% % % fprintf('1 car appearance car 3 in Progress ...\n');
% % % event.type =[1];
% % % event.occlusion_zone = [1];
% % % event.car_id = [3];
% % % run_num=7;
% % % simulator_1d_v2;
% % % 
% % % 
% % % fprintf('1 car appearance car 4 in Progress ...\n');
% % % event.type =[1];
% % % event.occlusion_zone = [1];
% % % event.car_id = [4];
% % % run_num=8;
% % % simulator_1d_v2;
% % % 
% % % 
toc;
