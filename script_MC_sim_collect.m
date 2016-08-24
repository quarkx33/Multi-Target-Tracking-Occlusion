
%script ot run MC simulations for various parameter settings

clear;
close all;
%clc;

tic;

%Initialise variables
simulations = 10000;  %00; %000; %00; %50000; % number of Monte Carlo simulations
trials = 50; % number of trials (time points) per MC simulation

% state is (x,x_dot, x_dotdot) (x-position, velocity and acceleration)

num_objects = 3;
dim = num_objects*3;

%Set number of particles for Particle Filter
num_particles= 15000; %0;


% temporal_params is struct containing mean, transition and noise
% measurement_params is struct containing mean, emission and noise
temporal_params = struct();
temporal_params.mean = 0;

%create the state transition matrix
tmp = [1,1,0.5;0,1,1];
temporal_params.transition = tmp;
temporal_params.noise_mean =0;

%specify the noise cov.matrix (noise enters through acceleration only)
%x and y cordinates are not affected by "noise" in the temporal model
temporal_params.noise_sigma = 0.001; %0.1;


temporal_params.mean_rev_speed = 0.3; %0.01;
temporal_params.mean_target_vel = 5; %00; %15;
temporal_params.AR_coeff = 0.75; % 0.75;


measurement_params = struct();
measurement_params.mean = 0;

% only x co-ordinate is observed
measurement_params.emission = [1,0];
measurement_params.noise_mean =0;
measurement_params.noise_sigma = 3; %10^10; %3; %10^10;


%car_idx_occlusion =2;


global_vars = struct();
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

global_vars.num_occlusion_blocks = 1;

global_vars.occlusion_blocs_cord = zeros(global_vars.num_occlusion_blocks,2);
global_vars.occlusion_blocs_cord(1,:) = [100,150];
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
 event.type =[1];
 event.occlusion_zone = [1];
 event.car_id = [1];
% 

% event.type =[-1, 1];
% event.occlusion_zone = [1, 2];
% event.car_id = [2, 2];


%models_to_consider = [2, 3, 4];

tol = 1e-20; %0.005; %1e-2; %1e-20; %.25;

%overall_MSE_particle_filter_timepts = zeros(0,100);
%overall_MSE_mean_filter_full_state_timepts = zeros(0,trials);
%overall_MSE_mahal_filter_full_state_timepts = zeros(0, trials);
 %==================================================================
%run simulator script for various values of parameters
%BASE run
fprintf('BASE Run in Progress ...\n');
simulator_1d_v2;
%fprintf('Running 1 particle model ...\n');
%simulator_1d_v2_one_vehicle;
%simulator_1d_v2_no_interaction;

% % % %overall_MSE(1) = MSE_particle_filter;
% % % overall_num_failures(1) = num_failures;
% % % %overall_MSE_particle_filter_timepts(1,1:50) = MSE_particle_filter_timepts;
% % % overall_MSE_mean_filter_full_state(1) = MSE_mean_filter_full_state;
% % % overall_MSE_mahal_filter_full_state(1) = MSE_mahal_filter_full_state;
% % % 
% % % overall_MSE_mean_filter_full_state_timepts(1,1:trials) = MSE_mean_filter_full_state_timepts;
% % % overall_MSE_mahal_filter_full_state_timepts(1,1:trials) = MSE_mahal_filter_full_state_timepts;
% % % 
% % % 
% % % overall_MSE_mahal_filter_full_state_preupdate(1) = MSE_mahal_filter_full_state_preupdate;
% % % overall_MSE_mahal_filter_full_state_timepts_preupdate(1,1:trials) = MSE_mahal_filter_full_state_timepts_preupdate;
% % % 


% % % % %run #1
% % % % %num_particles= 500;
% % % % tol=0.75;
% % % % fprintf('Run #1 in Progress ...\n');
% % % % simulator_1d_v2;
% % % % 
% % % % %overall_MSE(2) = MSE_particle_filter;
% % % % overall_num_failures(2) = num_failures;
% % % % %overall_MSE_particle_filter_timepts(2,1:50) = MSE_particle_filter_timepts;
% % % % overall_MSE_mean_filter_full_state(2) = MSE_mean_filter_full_state;
% % % % overall_MSE_mahal_filter_full_state(2) = MSE_mahal_filter_full_state;
% % % % 
% % % % overall_MSE_mean_filter_full_state_timepts(2,1:trials) = MSE_mean_filter_full_state_timepts;
% % % % overall_MSE_mahal_filter_full_state_timepts(2,1:trials) = MSE_mahal_filter_full_state_timepts;
% % % % 
% % % % 
% % % % %run #2
% % % % %num_particles= 1500;
% % % % tol=1;
% % % % fprintf('Run #2 in Progress ...\n');
% % % % simulator_1d_v2;
% % % % 
% % % % %overall_MSE(3) = MSE_particle_filter;
% % % % overall_num_failures(3) = num_failures;
% % % % %overall_MSE_particle_filter_timepts(3,1:50) = MSE_particle_filter_timepts;
% % % % overall_MSE_mean_filter_full_state(3) = MSE_mean_filter_full_state;
% % % % overall_MSE_mahal_filter_full_state(3) = MSE_mahal_filter_full_state;
% % % % 
% % % % overall_MSE_mean_filter_full_state_timepts(3,1:trials) = MSE_mean_filter_full_state_timepts;
% % % % overall_MSE_mahal_filter_full_state_timepts(3,1:trials) = MSE_mahal_filter_full_state_timepts;
% % % % 
% % % % 
% % % % %run #3
% % % % %num_particles= 1000;
% % % % %trials =200;
% % % % tol=0.25;
% % % % fprintf('Run #3 in Progress ...\n');
% % % % simulator_1d_v2;
% % % % 
% % % % %overall_MSE(4) = MSE_particle_filter;
% % % % overall_num_failures(4) = num_failures;
% % % % %overall_MSE_particle_filter_timepts(4,:) = MSE_particle_filter_timepts;
% % % % overall_MSE_mean_filter_full_state(4) = MSE_mean_filter_full_state;
% % % % overall_MSE_mahal_filter_full_state(4) = MSE_mahal_filter_full_state;
% % % % 
% % % % overall_MSE_mean_filter_full_state_timepts(4,1:trials) = MSE_mean_filter_full_state_timepts;
% % % % overall_MSE_mahal_filter_full_state_timepts(4,1:trials) = MSE_mahal_filter_full_state_timepts;
% % % % 
% % 
% % %run #4
% % trials =50;
% % num_objects = 2;
% % fprintf('Run #4 in Progress ...\n');
% % simulator_1d_v2;
% % 
% % %overall_MSE(5) = MSE_particle_filter;
% % overall_num_failures(5) = num_failures;
% % %overall_MSE_particle_filter_timepts(5,1:50) = MSE_particle_filter_timepts;
% % overall_MSE_mean_filter_full_state(5) = MSE_mean_filter_full_state;
% % overall_MSE_mahal_filter_full_state(5) = MSE_mahal_filter_full_state;
% % 
% % overall_MSE_mean_filter_full_state_timepts(5,1:trials) = MSE_mean_filter_full_state_timepts;
% % overall_MSE_mahal_filter_full_state_timepts(5,1:trials) = MSE_mahal_filter_full_state_timepts;
% % 
% % 
% % %run #5
% % num_objects =4;
% % fprintf('Run #5 in Progress ...\n');
% % simulator_1d_v2;
% % 
% % %overall_MSE(6) = MSE_particle_filter;
% % overall_num_failures(6) = num_failures;
% % %overall_MSE_particle_filter_timepts(6,1:50) = MSE_particle_filter_timepts;
% % overall_MSE_mean_filter_full_state(6) = MSE_mean_filter_full_state;
% % overall_MSE_mahal_filter_full_state(6) = MSE_mahal_filter_full_state;
% % 
% % overall_MSE_mean_filter_full_state_timepts(6,1:trials) = MSE_mean_filter_full_state_timepts;
% % overall_MSE_mahal_filter_full_state_timepts(6,1:trials) = MSE_mahal_filter_full_state_timepts;
% % 
% % 
% % %run #6
% % num_objects =3;
% % prob_occl =0.25;
% % fprintf('Run #6 in Progress ...\n');
% % simulator_1d_v2;
% % 
% % %overall_MSE(7) = MSE_particle_filter;
% % overall_num_failures(7) = num_failures;
% % %overall_MSE_particle_filter_timepts(7,1:50) = MSE_particle_filter_timepts;
% % overall_MSE_mean_filter_full_state(7) = MSE_mean_filter_full_state;
% % overall_MSE_mahal_filter_full_state(7) = MSE_mahal_filter_full_state;
% % 
% % overall_MSE_mean_filter_full_state_timepts(7,1:trials) = MSE_mean_filter_full_state_timepts;
% % overall_MSE_mahal_filter_full_state_timepts(7,1:trials) = MSE_mahal_filter_full_state_timepts;
% % 
% % 
% % %run #7
% % prob_occl =0.5;
% % fprintf('Run #7 in Progress ...\n');
% % simulator_1d_v2;
% % 
% % %overall_MSE(8) = MSE_particle_filter;
% % overall_num_failures(8) = num_failures;
% % %overall_MSE_particle_filter_timepts(8,1:50) = MSE_particle_filter_timepts;
% % overall_MSE_mean_filter_full_state(8) = MSE_mean_filter_full_state;
% % overall_MSE_mahal_filter_full_state(8) = MSE_mahal_filter_full_state;
% % 
% % overall_MSE_mean_filter_full_state_timepts(8,1:trials) = MSE_mean_filter_full_state_timepts;
% % overall_MSE_mahal_filter_full_state_timepts(8,1:trials) = MSE_mahal_filter_full_state_timepts;
% % 
% % 
% % %run #8
% % prob_occl =0.75;
% % fprintf('Run #8 in Progress ...\n');
% % simulator_1d_v2;
% % 
% % %overall_MSE(9) = MSE_particle_filter;
% % overall_num_failures(9) = num_failures;
% % %overall_MSE_particle_filter_timepts(9,1:50) = MSE_particle_filter_timepts;
% % overall_MSE_mean_filter_full_state(9) = MSE_mean_filter_full_state;
% % overall_MSE_mahal_filter_full_state(9) = MSE_mahal_filter_full_state;
% % 
% % overall_MSE_mean_filter_full_state_timepts(9,1:trials) = MSE_mean_filter_full_state_timepts;
% % overall_MSE_mahal_filter_full_state_timepts(9,1:trials) = MSE_mahal_filter_full_state_timepts;
% % 
% % 
% % %run #9
% % prob_occl =0.9;
% % fprintf('Run #9 in Progress ...\n');
% % simulator_1d_v2;
% % 
% % %overall_MSE(10) = MSE_particle_filter;
% % overall_num_failures(10) = num_failures;
% % %overall_MSE_particle_filter_timepts(10,1:50) = MSE_particle_filter_timepts;
% % overall_MSE_mean_filter_full_state(10) = MSE_mean_filter_full_state;
% % overall_MSE_mahal_filter_full_state(10) = MSE_mahal_filter_full_state;
% % 
% % overall_MSE_mean_filter_full_state_timepts(10,1:trials) = MSE_mean_filter_full_state_timepts;
% % overall_MSE_mahal_filter_full_state_timepts(10,1:trials) = MSE_mahal_filter_full_state_timepts;
% % 
% % 
% % %run #10
% % prob_occl =1;
% % fprintf('Run #10 in Progress ...\n');
% % simulator_1d_v2;
% % 
% % %overall_MSE(11) = MSE_particle_filter;
% % overall_num_failures(11) = num_failures;
% % %overall_MSE_particle_filter_timepts(11,1:50) = MSE_particle_filter_timepts;
% % overall_MSE_mean_filter_full_state(11) = MSE_mean_filter_full_state;
% % overall_MSE_mahal_filter_full_state(11) = MSE_mahal_filter_full_state;
% % 
% % overall_MSE_mean_filter_full_state_timepts(11,1:trials) = MSE_mean_filter_full_state_timepts;
% % overall_MSE_mahal_filter_full_state_timepts(11,1:trials) = MSE_mahal_filter_full_state_timepts;
% % 

toc;
