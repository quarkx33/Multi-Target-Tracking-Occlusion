%Simulator code

% % % % % clear;
% % % % % close all;
% % % % % 
% % % % % %Initialise variables
% % % % % simulations = 1000; % number of Monte Carlo simulations
% % % % % trials = 50; % number of trials (time points) per MC simulation
% % % % % 
% % % % % % state is (x,x_dot, x_dotdot) (x-position, velocity and acceleration)
% % % % % 
% % % % % num_objects = 3;
% % % % % dim = num_objects*3;
% % % % % 
% % % % % %Set number of particles for Particle Filter
% % % % % num_particles= 100;
% % % % % 
% % % % % 
% % % % % % temporal_params is struct containing mean, transition and noise
% % % % % % measurement_params is struct containing mean, emission and noise
% % % % % temporal_params = struct();
% % % % % temporal_params.mean = 0;
% % % % % 
% % % % % %create the state transition matrix
% % % % % tmp = [1,1,0;0,1,1];
% % % % % temporal_params.transition = tmp;
% % % % % temporal_params.noise_mean =0;
% % % % % 
% % % % % %specify the noise cov.matrix (noise enters through acceleration only)
% % % % % %x and y cordinates are not affected by "noise" in the temporal model
% % % % % temporal_params.noise_sigma = 1;
% % % % % 
% % % % % 
% % % % % measurement_params = struct();
% % % % % measurement_params.mean = 0;
% % % % % 
% % % % % % only x co-ordinate is observed
% % % % % measurement_params.emission = [1,0];
% % % % % measurement_params.noise_mean =0;
% % % % % measurement_params.noise_sigma = 3;
% % % % % 
% % % % % 
% % % % % car_idx_occlusion =2;
% % % % % 
% % % % % 
% % % % % global_vars = struct();
% % % % % global_vars.safe_dist = 15; %recommended dist between vehicles
% % % % % global_vars.v_max = 45; %max allowalable velocity
% % % % % % global_vars.mode = dist_safe; %mode of gamma r.v. (for co-ordinate simulation)
% % % % % % global_vars.var = 3*global_vars.v_max;  %variance of gamma --- test this!!
% % % % % global_vars.scale_fac_measurement_noise_sigma=1;
% % % % % 
% % % % % % global_vars.mean_obj1_accn = 0; % this is mode of acceleration
% % % % % % global_vars.sigma_obj1_accn = 3; % this is variance of accleration
% % % % % 
% % % % % global_vars.accn_max = 3;
% % % % % global_vars.accn_min = -4;
% % % % % 
% % % % %     %Set occlusion probability 
% % % % %     prob_occl = 1;%unifrnd(0.4, 0.8);
% % % % % 
% MSE_kalman = 0;
%MSE_particle_filter = 0;
num_failures =0;

%MSE_particle_filter_timepts = zeros(1, trials);
dim = num_objects*3;
error_est_filter_full_state= zeros(dim, trials, simulations);

%MSE_mean_filter_full_state_timepts = zeros(simulations,trials);

MSE_mahal_filter_full_state = zeros(simulations,1);
MSE_mahal_filter_full_state_timepts = zeros(simulations, trials);
MSE_mahal_filter_full_state_preupdate = zeros(simulations,1);
MSE_mahal_filter_full_state_timepts_preupdate = zeros(simulations, trials);

MSE_mahal_filter_full_state_car1 = zeros(simulations,1);
MSE_mahal_filter_full_state_timepts_car1 = zeros(simulations, trials);
MSE_mahal_filter_full_state_preupdate_car1 = zeros(simulations,1);
MSE_mahal_filter_full_state_timepts_preupdate_car1 = zeros(simulations, trials);

MSE_mahal_filter_full_state_car2 = zeros(simulations,1);
MSE_mahal_filter_full_state_timepts_car2 = zeros(simulations, trials);
MSE_mahal_filter_full_state_preupdate_car2 = zeros(simulations,1);
MSE_mahal_filter_full_state_timepts_preupdate_car2 = zeros(simulations, trials);

MSE_mahal_filter_full_state_car3 = zeros(simulations,1);
MSE_mahal_filter_full_state_timepts_car3 = zeros(simulations, trials);
MSE_mahal_filter_full_state_preupdate_car3 = zeros(simulations,1);
MSE_mahal_filter_full_state_timepts_preupdate_car3 = zeros(simulations, trials);

MSE_mahal_filter_full_state_car1_x = zeros(simulations,1);
MSE_mahal_filter_full_state_timepts_car1_x = zeros(simulations, trials);
MSE_mahal_filter_full_state_preupdate_car1_x = zeros(simulations,1);
MSE_mahal_filter_full_state_timepts_preupdate_car1_x = zeros(simulations, trials);

MSE_mahal_filter_full_state_car2_x = zeros(simulations,1);
MSE_mahal_filter_full_state_timepts_car2_x = zeros(simulations, trials);
MSE_mahal_filter_full_state_preupdate_car2_x = zeros(simulations,1);
MSE_mahal_filter_full_state_timepts_preupdate_car2_x = zeros(simulations, trials);

MSE_mahal_filter_full_state_car3_x = zeros(simulations,1);
MSE_mahal_filter_full_state_timepts_car3_x = zeros(simulations, trials);
MSE_mahal_filter_full_state_preupdate_car3_x = zeros(simulations,1);
MSE_mahal_filter_full_state_timepts_preupdate_car3_x = zeros(simulations, trials);

MSE_mahal_filter_full_state_car1_v = zeros(simulations,1);
MSE_mahal_filter_full_state_timepts_car1_v = zeros(simulations, trials);
MSE_mahal_filter_full_state_preupdate_car1_v = zeros(simulations,1);
MSE_mahal_filter_full_state_timepts_preupdate_car1_v = zeros(simulations, trials);

MSE_mahal_filter_full_state_car2_v = zeros(simulations,1);
MSE_mahal_filter_full_state_timepts_car2_v = zeros(simulations, trials);
MSE_mahal_filter_full_state_preupdate_car2_v = zeros(simulations,1);
MSE_mahal_filter_full_state_timepts_preupdate_car2_v = zeros(simulations, trials);

MSE_mahal_filter_full_state_car3_v = zeros(simulations,1);
MSE_mahal_filter_full_state_timepts_car3_v = zeros(simulations, trials);
MSE_mahal_filter_full_state_preupdate_car3_v = zeros(simulations,1);
MSE_mahal_filter_full_state_timepts_preupdate_car3_v = zeros(simulations, trials);


MSE_mahal_filter_full_state_car1_a = zeros(simulations,1);
MSE_mahal_filter_full_state_timepts_car1_a = zeros(simulations, trials);
MSE_mahal_filter_full_state_preupdate_car1_a = zeros(simulations,1);
MSE_mahal_filter_full_state_timepts_preupdate_car1_a = zeros(simulations, trials);


MSE_mahal_filter_full_state_car2_a = zeros(simulations,1);
MSE_mahal_filter_full_state_timepts_car2_a = zeros(simulations, trials);
MSE_mahal_filter_full_state_preupdate_car2_a = zeros(simulations,1);
MSE_mahal_filter_full_state_timepts_preupdate_car2_a = zeros(simulations, trials);

MSE_mahal_filter_full_state_car3_a = zeros(simulations,1);
MSE_mahal_filter_full_state_timepts_car3_a = zeros(simulations, trials);
MSE_mahal_filter_full_state_preupdate_car3_a = zeros(simulations,1);
MSE_mahal_filter_full_state_timepts_preupdate_car3_a = zeros(simulations, trials);



% num_iterations = 500;
% all_collect = zeros(dim,num_iterations*simulations, trials);


%% Monte Carlo simulations
%tic;
parfor ii = 1: simulations
    ii;
    
    obj_idx_occlude = ones(num_objects, trials);
    tmp = mnrnd(1,[prob_occl, 1- prob_occl],trials);
    if num_objects>1
        obj_idx_occlude(car_idx_occlusion,:) =  tmp(:,2);  % occlusion for object car_idx_occlusion set earlier
    end
    
    %Reset simulator and generate temporal movements and subsequent measurements
    %set starting positions of objects
    separation_dist = global_vars.safe_dist*2 + randn;
    %init_x = sort( unifrnd(0,20*global_vars.safe_dist*(num_objects-1),1, num_objects), 'descend');
    init_x = fliplr(separation_dist*[0: (num_objects-1)]);
    init_vel = unifrnd(5, 10, 1, num_objects); %gamrnd(22.5,1/1.5, 1, num_objects);
    init_accleration = unifrnd(0, 5, 1, num_objects); %rand(1, num_objects);
    
    init_state = [init_x;init_vel;init_accleration];
    init_state = init_state(:);
    
    states = oneD_temporal_model_v2_constrained(init_state, temporal_params, trials, num_objects, global_vars);
    %plot the states (x-coordinates only)
    %a = plot_anim(states);
    
% % % %     %try to collect variances of the states
% % % %     %collect = zeros(num_objects*3, 0);
% % % %     for time_iter = 1: trials
% % % %         for iter = 1:num_iterations      
% % % %              tmp = oneD_temporal_model_v2_constrained(states(:,time_iter), temporal_params, 2, num_objects, global_vars);
% % % %             %tmp(1:3,2)
% % % %              %collect(:,iter) = tmp(:,2);
% % % %             all_collect(:,(ii-1)*num_iterations + iter, time_iter) = tmp(:,2);
% % % %         end
% % % %        % var_collect(:, time_iter) = var(collect,1,2);
% % % %     end
% % % %     
     
    
    measurements = oneD_measurement_model_v2(states, num_objects, measurement_params);
    
        
%     %Run Kalman Filter on the measurements
%     [kalman_mean, ~] = Kalman_v1(measurements, temporal_params, measurement_params);
%     tmp = kalman_mean(1:2,:) - states(1:2,:);
%     MSE_kalman(ii,:) = sum(tmp .^2,1); 
    
    %Run Particle filter on the measurements
   % [wts, particles] = oneD_Particle_filter_v1(measurements, temporal_params, measurement_params, num_particles, global_vars);
    
     [posterior_states_mean, posterior_states_cov, num_fail, posterior_states_mean_preupdate, posterior_states_cov_preupdate, posterior_states_resamp, posterior_cov_resamp, all_particles] = oneD_Particle_filter_occlusion_v1(init_state, logical(obj_idx_occlude), measurements, temporal_params, measurement_params, num_particles, global_vars, states);
    
     
     %tmp = abs(posterior_states_cov(:,:,1:end-1) - posterior_cov_resamp(:,:,:));
     %sum(tmp(:))
     %tmp = abs(posterior_states_mean(:,1:end-1) - posterior_states_resamp(:,:));
     %sum(tmp(:))
     
     %calculate MSE for only the x-position 
     %MSE_particle_filter = MSE_particle_filter + sum(sum((posterior_states(1:3:end,:)- states(1:3:end,:)).^2));
     %MSE_particle_filter_timepts = MSE_particle_filter_timepts + sum((posterior_states(1:3:end,:)- states(1:3:end,:)).^2,1);

     %calculate MSE (for full particle state)
     tmp = posterior_states_mean - states;
     %MSE_mean_filter_full_state_timepts(ii,:) = sum(tmp.^2,1);
     %MSE_mean_filter_full_state(ii) = sum(MSE_mean_filter_full_state_timepts(ii,:));
     error_est_filter_full_state(:,:, ii) = tmp;
     
     
     tmp_preupdate = posterior_states_mean_preupdate - states;
     %error_est_filter_full_state_preupdate(:,:, ii) = tmp_preupdate;
     
     
     for jj = 1: trials
         jj;
          try
         %original calculation of mahal. dist
         %fprintf('jj = %d, mse = %3.2f, trace= %3.2f\n',jj,sum(tmp(:,jj).^2), trace(posterior_states_cov(:,:,jj)));
         
         %fprintf('jj = %d, det = %d, det_pre= %d\n',jj,svds(posterior_states_cov(:,:,jj),1,0), svds(posterior_states_cov_preupdate(:,:,jj),1,0));
         
         
         
         
         
        MSE_mahal_filter_full_state_timepts_car1_x(ii,jj) = tmp(1,jj)' *pinv(posterior_states_cov(1,1,jj),tol) *tmp(1,jj);
        MSE_mahal_filter_full_state_timepts_preupdate_car1_x(ii,jj) = tmp(1,jj)' *pinv(posterior_states_cov_preupdate(1,1,jj),tol) *tmp(1,jj);

% %         MSE_mahal_filter_full_state_timepts_car2_x(ii,jj) = tmp(4,jj)' *pinv(posterior_states_cov(4,4,jj),tol) *tmp(4,jj);
% %         MSE_mahal_filter_full_state_timepts_preupdate_car2_x(ii,jj) = tmp(4,jj)' *pinv(posterior_states_cov_preupdate(4,4,jj),tol) *tmp(4,jj);
% % 
% %         MSE_mahal_filter_full_state_timepts_car3_x(ii,jj) = tmp(7,jj)' *pinv(posterior_states_cov(7,7,jj),tol) *tmp(7,jj);
% %         MSE_mahal_filter_full_state_timepts_preupdate_car3_x(ii,jj) = tmp(7,jj)' *pinv(posterior_states_cov_preupdate(7,7,jj),tol) *tmp(7,jj);

        MSE_mahal_filter_full_state_timepts_car1(ii,jj) = tmp(1:3,jj)' *pinv(posterior_states_cov(1:3,1:3,jj), tol) *tmp(1:3,jj);
        MSE_mahal_filter_full_state_timepts_preupdate_car1(ii,jj) = tmp(1:3,jj)' *pinv(posterior_states_cov_preupdate(1:3,1:3,jj), tol) *tmp(1:3,jj);

% %         MSE_mahal_filter_full_state_timepts_car2(ii,jj) = tmp(4:6,jj)' *pinv(posterior_states_cov(4:6,4:6,jj), tol) *tmp(4:6,jj);
% %         MSE_mahal_filter_full_state_timepts_preupdate_car2(ii,jj) = tmp(4:6,jj)' *pinv(posterior_states_cov_preupdate(4:6,4:6,jj), tol) *tmp(4:6,jj);
% % 
% %         MSE_mahal_filter_full_state_timepts_car3(ii,jj) = tmp(7:9,jj)' *pinv(posterior_states_cov(7:9,7:9,jj), tol) *tmp(7:9,jj);
% %         MSE_mahal_filter_full_state_timepts_preupdate_car3(ii,jj) = tmp(7:9,jj)' *pinv(posterior_states_cov_preupdate(7:9,7:9,jj), tol) *tmp(7:9,jj);
% % 
         
        MSE_mahal_filter_full_state_timepts(ii,jj) = tmp(:,jj)' *pinv(posterior_states_cov(:,:,jj), tol) *tmp(:,jj);
        MSE_mahal_filter_full_state_timepts_preupdate(ii,jj) = tmp(:,jj)' *pinv(posterior_states_cov_preupdate(:,:,jj), tol) *tmp(:,jj);

         
  
        
        
        MSE_mahal_filter_full_state_timepts_car1_v(ii,jj) = tmp(2,jj)' *pinv(posterior_states_cov(2,2,jj),tol) *tmp(2,jj);
        MSE_mahal_filter_full_state_timepts_preupdate_car1_v(ii,jj) = tmp(2,jj)' *pinv(posterior_states_cov_preupdate(2,2,jj)) *tmp(2,jj);

% %         MSE_mahal_filter_full_state_timepts_car2_v(ii,jj) = tmp(5,jj)' *pinv(posterior_states_cov(5,5,jj),tol) *tmp(5,jj);
% %         MSE_mahal_filter_full_state_timepts_preupdate_car2_v(ii,jj) = tmp(5,jj)' *pinv(posterior_states_cov_preupdate(5,5,jj),tol) *tmp(5,jj);
% % 
% %         MSE_mahal_filter_full_state_timepts_car3_v(ii,jj) = tmp(8,jj)' *pinv(posterior_states_cov(8,8,jj),tol) *tmp(8,jj);
% %         MSE_mahal_filter_full_state_timepts_preupdate_car3_v(ii,jj) = tmp(8,jj)' *pinv(posterior_states_cov_preupdate(8,8,jj),tol) *tmp(8,jj);

  
        
        
        MSE_mahal_filter_full_state_timepts_car1_a(ii,jj) = tmp(3,jj)' *pinv(posterior_states_cov(3,3,jj),tol) *tmp(3,jj);
        MSE_mahal_filter_full_state_timepts_preupdate_car1_a(ii,jj) = tmp(3,jj)' *pinv(posterior_states_cov_preupdate(3,3,jj)) *tmp(3,jj);

% %         MSE_mahal_filter_full_state_timepts_car2_a(ii,jj) = tmp(6,jj)' *pinv(posterior_states_cov(6,6,jj),tol) *tmp(6,jj);
% %         MSE_mahal_filter_full_state_timepts_preupdate_car2_a(ii,jj) = tmp(6,jj)' *pinv(posterior_states_cov_preupdate(6,6,jj),tol) *tmp(6,jj);
% % 
% %         MSE_mahal_filter_full_state_timepts_car3_a(ii,jj) = tmp(9,jj)' *pinv(posterior_states_cov(9,9,jj),tol) *tmp(9,jj);
% %         MSE_mahal_filter_full_state_timepts_preupdate_car3_a(ii,jj) = tmp(9,jj)' *pinv(posterior_states_cov_preupdate(9,9,jj),tol) *tmp(9,jj);

         %MSE_mahal_filter_full_state_timepts(ii,jj) = tmp(:,jj)' *pinv(posterior_states_cov(:,:,jj), tol) *tmp(:,jj);
         %MSE_mahal_filter_full_state_timepts_preupdate(ii,jj) = tmp_preupdate(:,jj)' *pinv(posterior_states_cov_preupdate(:,:,jj), tol) *tmp_preupdate(:,jj);
         %MSE_mahal_filter_full_state_timepts_preupdate(ii,jj) = tmp(:,jj)' *pinv(posterior_states_cov_preupdate(:,:,jj), tol) *tmp(:,jj);
         
         %MSE_mahal_filter_full_state_timepts_resamp(ii,jj) = tmp(:,jj)' *pinv(posterior_cov_resamp(:,:,jj), tol) *tmp(:,jj);
         
         %fprintf('jj = %d, det = %d, det_pre= %d\n',jj,svds(posterior_states_cov(:,:,jj),1,0), svds(posterior_states_cov_preupdate(:,:,jj),1,0));
         
         %only the x-coordinates mahal dist
%           MSE_mahal_filter_full_state_timepts(ii,jj) = tmp(1:3:end,jj)' *pinv(posterior_states_cov(1:3:end,1:3:end,jj), tol) *tmp(1:3:end,jj);
%           MSE_mahal_filter_full_state_timepts_preupdate(ii,jj) = tmp(1:3:end,jj)' *pinv(posterior_states_cov_preupdate(1:3:end,1:3:end,jj), tol) *tmp(1:3:end,jj);
% % % % % %          MSE_mahal_filter_full_state_timepts(ii,jj) = tmp(3,jj)' *pinv(posterior_states_cov(3,3,jj), tol) *tmp(3,jj);
% % % % % %          MSE_mahal_filter_full_state_timepts_preupdate(ii,jj) = tmp(3,jj)' *pinv(posterior_states_cov_preupdate(3,3,jj), tol) *tmp(3,jj);
      
         
         %fprintf('jj = %d, tot var = %d, tot var = %d\n',jj,rcond(posterior_states_cov(:,:,jj)), rcond(posterior_states_cov_preupdate(:,:,jj)));
         %fprintf('jj = %d, min var = %d, min var= %d\n',jj,min(diag(posterior_states_cov(:,:,jj))), min(diag(posterior_states_cov_preupdate(:,:,jj))) );
 
         
         MSE_mahal_filter_full_state_timepts(ii,jj);
         MSE_mahal_filter_full_state_timepts_preupdate(ii,jj);
         %MSE_mahal_filter_full_state_timepts_resamp(ii,jj);
         
            
            % new calcu for mahalanobis dist.
         %tmp1 = particles(:,:,jj) - repmat(posterior_states_mean);
         %MSE_mahal_filter_full_state_timepts(ii,jj) = trace(posterior_states_cov(:,:,jj));
         
         % %          error(lastwarn);
          catch
              pause;
% %              lastwarn('');
          end
         
         MSE_mahal_filter_full_state(ii) = MSE_mahal_filter_full_state(ii) + MSE_mahal_filter_full_state_timepts(ii,jj);    
         MSE_mahal_filter_full_state_preupdate(ii) = MSE_mahal_filter_full_state_preupdate(ii) + MSE_mahal_filter_full_state_timepts_preupdate(ii,jj);

         MSE_mahal_filter_full_state_car1(ii) = MSE_mahal_filter_full_state_car1(ii) + MSE_mahal_filter_full_state_timepts_car1(ii,jj);    
         MSE_mahal_filter_full_state_preupdate_car1(ii) = MSE_mahal_filter_full_state_preupdate_car1(ii) + MSE_mahal_filter_full_state_timepts_preupdate_car1(ii,jj);

% %          MSE_mahal_filter_full_state_car2(ii) = MSE_mahal_filter_full_state_car2(ii) + MSE_mahal_filter_full_state_timepts_car2(ii,jj);    
% %          MSE_mahal_filter_full_state_preupdate_car2(ii) = MSE_mahal_filter_full_state_preupdate_car2(ii) + MSE_mahal_filter_full_state_timepts_preupdate_car2(ii,jj);
% % 
% %          MSE_mahal_filter_full_state_car3(ii) = MSE_mahal_filter_full_state_car3(ii) + MSE_mahal_filter_full_state_timepts_car3(ii,jj);    
% %          MSE_mahal_filter_full_state_preupdate_car3(ii) = MSE_mahal_filter_full_state_preupdate_car3(ii) + MSE_mahal_filter_full_state_timepts_preupdate_car3(ii,jj);

         MSE_mahal_filter_full_state_car1_x(ii) = MSE_mahal_filter_full_state_car1_x(ii) + MSE_mahal_filter_full_state_timepts_car1_x(ii,jj);    
         MSE_mahal_filter_full_state_preupdate_car1_x(ii) = MSE_mahal_filter_full_state_preupdate_car1_x(ii) + MSE_mahal_filter_full_state_timepts_preupdate_car1_x(ii,jj);

% %          MSE_mahal_filter_full_state_car2_x(ii) = MSE_mahal_filter_full_state_car2_x(ii) + MSE_mahal_filter_full_state_timepts_car2_x(ii,jj);    
% %          MSE_mahal_filter_full_state_preupdate_car2_x(ii) = MSE_mahal_filter_full_state_preupdate_car2_x(ii) + MSE_mahal_filter_full_state_timepts_preupdate_car2_x(ii,jj);
% % 
% %          MSE_mahal_filter_full_state_car3_x(ii) = MSE_mahal_filter_full_state_car3_x(ii) + MSE_mahal_filter_full_state_timepts_car3_x(ii,jj);    
% %          MSE_mahal_filter_full_state_preupdate_car3_x(ii) = MSE_mahal_filter_full_state_preupdate_car3_x(ii) + MSE_mahal_filter_full_state_timepts_preupdate_car3_x(ii,jj);


         
         MSE_mahal_filter_full_state_car1_v(ii) = MSE_mahal_filter_full_state_car1_v(ii) + MSE_mahal_filter_full_state_timepts_car1_v(ii,jj);    
         MSE_mahal_filter_full_state_preupdate_car1_v(ii) = MSE_mahal_filter_full_state_preupdate_car1_v(ii) + MSE_mahal_filter_full_state_timepts_preupdate_car1_v(ii,jj);

% %          MSE_mahal_filter_full_state_car2_v(ii) = MSE_mahal_filter_full_state_car2_v(ii) + MSE_mahal_filter_full_state_timepts_car2_v(ii,jj);    
% %          MSE_mahal_filter_full_state_preupdate_car2_v(ii) = MSE_mahal_filter_full_state_preupdate_car2_v(ii) + MSE_mahal_filter_full_state_timepts_preupdate_car2_v(ii,jj);
% % 
% %          MSE_mahal_filter_full_state_car3_v(ii) = MSE_mahal_filter_full_state_car3_v(ii) + MSE_mahal_filter_full_state_timepts_car3_v(ii,jj);    
% %          MSE_mahal_filter_full_state_preupdate_car3_v(ii) = MSE_mahal_filter_full_state_preupdate_car3_v(ii) + MSE_mahal_filter_full_state_timepts_preupdate_car3_v(ii,jj);

    
         
         
         MSE_mahal_filter_full_state_car1_a(ii) = MSE_mahal_filter_full_state_car1_a(ii) + MSE_mahal_filter_full_state_timepts_car1_a(ii,jj);    
         MSE_mahal_filter_full_state_preupdate_car1_a(ii) = MSE_mahal_filter_full_state_preupdate_car1_a(ii) + MSE_mahal_filter_full_state_timepts_preupdate_car1_a(ii,jj);

% %          MSE_mahal_filter_full_state_car2_a(ii) = MSE_mahal_filter_full_state_car2_a(ii) + MSE_mahal_filter_full_state_timepts_car2_a(ii,jj);    
% %          MSE_mahal_filter_full_state_preupdate_car2_a(ii) = MSE_mahal_filter_full_state_preupdate_car2_a(ii) + MSE_mahal_filter_full_state_timepts_preupdate_car2_a(ii,jj);
% % 
% %          MSE_mahal_filter_full_state_car3_a(ii) = MSE_mahal_filter_full_state_car3_a(ii) + MSE_mahal_filter_full_state_timepts_car3_a(ii,jj);    
% %          MSE_mahal_filter_full_state_preupdate_car3_a(ii) = MSE_mahal_filter_full_state_preupdate_car3_a(ii) + MSE_mahal_filter_full_state_timepts_preupdate_car3_a(ii,jj);

         
     end
     %MSE_mahal_filter_full_state(ii) = sum(MSE_mahal_filter_full_state_timepts(ii,:));
     
     
     num_failures = num_failures + num_fail;
     
          
   % a = plot_anim(states, posterior_states_mean, measurements, logical(obj_idx_occlude), all_particles);
    
    
    % % % %     [posterior_states,fac, fac_curr_period] = oneD_Particle_filter_occlusion_bayes_fac_v1(init_state, logical(obj_idx_occlude), measurements, temporal_params, measurement_params, num_particles, global_vars);
% % % % 
% % % %     plot(fac,'o-'); title(['iteration= ',num2str(ii)]);pause(1);
% % % % 
% % % %     if any(diff(fac)<-5)
% % % %     
% % % %         pause(1);
% % % %        % a = plot_anim(states, posterior_states, measurements, logical(obj_idx_occlude));
% % % %     
% % % %     end

    
    
    
end
%toc;

% % 
% % plot_id =3;
% % aa = zeros(trials,simulations);
% % for ii = 1: trials
% %     for jj = 1: simulations
% %         tmp = all_collect(plot_id, (jj-1)*num_iterations+1: jj*num_iterations,ii);
% %         aa(ii,jj) = var(tmp(:),1);
% %     end
% % end
% % plot(mean(aa,2), '.-');
% % 
% % pause;


%calcukate MSE per car per simulation per time point
%MSE_particle_filter = MSE_particle_filter/(num_objects * simulations*trials);

% %calculate num of tracker failures (average number of failures over trial
% %time points)
% num_failures = num_failures/simulations;

%MSE per time point
%MSE_particle_filter_timepts = MSE_particle_filter_timepts / (num_objects * simulations);

tmp = zeros(dim, simulations* trials);
for ii = 1 : simulations
    %flatten the 3D array
   tmp(:,(ii-1)*trials+1: ii*trials) = error_est_filter_full_state(:,:,ii);   
end
tmp_mean = mean(tmp,2);
MSE_mean_filter_full_state = sum(sum((tmp - repmat(tmp_mean,1,simulations* trials)).^2))/ (simulations*trials);
%MSE_mean_filter_full_state = sum(MSE_mean_filter_full_state) /(simulations*trials);
MSE_mahal_filter_full_state = sum(MSE_mahal_filter_full_state) /(simulations*trials);

MSE_mahal_filter_full_state_preupdate = sum(MSE_mahal_filter_full_state_preupdate) /(simulations*trials);



tmp_timepts = zeros(dim, trials);

MSE_mean_filter_full_state_timepts = zeros(1,trials);
for jj = 1 : trials
    %flatten the 3D array
   tmp_timepts(:,jj) = mean(squeeze(error_est_filter_full_state(:,jj,:)),2);
   MSE_mean_filter_full_state_timepts(jj) = sum(sum( (squeeze(error_est_filter_full_state(:,jj,:))- repmat(tmp_timepts(:,jj),1,simulations)).^2));
end
MSE_mean_filter_full_state_timepts = MSE_mean_filter_full_state_timepts/simulations;


tmp_timepts = zeros(1, trials);
MSE_mean_filter_full_state_timepts = zeros(1,trials);
idx =1;
for jj = 1 : trials
    %flatten the 3D array
   tmp_timepts(:,jj) = mean(squeeze(error_est_filter_full_state(idx,jj,:)));
   MSE_mean_filter_full_state_timepts(jj) = sum(sum( (squeeze(error_est_filter_full_state(idx,jj,:))- repmat(tmp_timepts(:,jj),1,simulations)').^2));
end
MSE_mean_filter_full_state_timepts = MSE_mean_filter_full_state_timepts/simulations;




%MSE_mean_filter_full_state_timepts = sum(MSE_mean_filter_full_state_timepts,1) /simulations;
MSE_mahal_filter_full_state_timepts1 = sum(MSE_mahal_filter_full_state_timepts,1) /simulations;
%MSE_mahal_filter_full_state_timepts1 = median(MSE_mahal_filter_full_state_timepts,1);
%MSE_mahal_filter_full_state_timepts1 = trimmean(MSE_mahal_filter_full_state_timepts,25,1);


MSE_mahal_filter_full_state_timepts_preupdate1 = sum(MSE_mahal_filter_full_state_timepts_preupdate,1) /simulations;
%MSE_mahal_filter_full_state_timepts_preupdate1 = median(MSE_mahal_filter_full_state_timepts_preupdate,1) ;

%MSE_mahal_filter_full_state_timepts_preupdate1 = trimmean(MSE_mahal_filter_full_state_timepts_preupdate, 40,1);

MSE_mahal_filter_full_state_timepts_car1_1 = sum(MSE_mahal_filter_full_state_timepts_car1,1) /simulations;
MSE_mahal_filter_full_state_timepts_preupdate_car1_1 = sum(MSE_mahal_filter_full_state_timepts_preupdate_car1,1) /simulations;

% % MSE_mahal_filter_full_state_timepts_car2_1 = sum(MSE_mahal_filter_full_state_timepts_car2,1) /simulations;
% % MSE_mahal_filter_full_state_timepts_preupdate_car2_1 = sum(MSE_mahal_filter_full_state_timepts_preupdate_car2,1) /simulations;
% % 
% % MSE_mahal_filter_full_state_timepts_car3_1 = sum(MSE_mahal_filter_full_state_timepts_car3,1) /simulations;
% % MSE_mahal_filter_full_state_timepts_preupdate_car3_1 = sum(MSE_mahal_filter_full_state_timepts_preupdate_car3,1) /simulations;


MSE_mahal_filter_full_state_timepts_car1_x_1 = sum(MSE_mahal_filter_full_state_timepts_car1_x,1) /simulations;
MSE_mahal_filter_full_state_timepts_preupdate_car1_x_1 = sum(MSE_mahal_filter_full_state_timepts_preupdate_car1_x,1) /simulations;

% % MSE_mahal_filter_full_state_timepts_car2_x_1 = sum(MSE_mahal_filter_full_state_timepts_car2_x,1) /simulations;
% % MSE_mahal_filter_full_state_timepts_preupdate_car2_x_1 = sum(MSE_mahal_filter_full_state_timepts_preupdate_car2_x,1) /simulations;
% % 
% % MSE_mahal_filter_full_state_timepts_car3_x_1 = sum(MSE_mahal_filter_full_state_timepts_car3_x,1) /simulations;
% % MSE_mahal_filter_full_state_timepts_preupdate_car3_x_1 = sum(MSE_mahal_filter_full_state_timepts_preupdate_car3_x,1) /simulations;



MSE_mahal_filter_full_state_timepts_car1_v_1 = sum(MSE_mahal_filter_full_state_timepts_car1_v,1) /simulations;
MSE_mahal_filter_full_state_timepts_preupdate_car1_v_1 = sum(MSE_mahal_filter_full_state_timepts_preupdate_car1_v,1) /simulations;

% % MSE_mahal_filter_full_state_timepts_car2_v_1 = sum(MSE_mahal_filter_full_state_timepts_car2_v,1) /simulations;
% % MSE_mahal_filter_full_state_timepts_preupdate_car2_v_1 = sum(MSE_mahal_filter_full_state_timepts_preupdate_car2_v,1) /simulations;
% % 
% % MSE_mahal_filter_full_state_timepts_car3_v_1 = sum(MSE_mahal_filter_full_state_timepts_car3_v,1) /simulations;
% % MSE_mahal_filter_full_state_timepts_preupdate_car3_v_1 = sum(MSE_mahal_filter_full_state_timepts_preupdate_car3_v,1) /simulations;


MSE_mahal_filter_full_state_timepts_car1_a_1 = sum(MSE_mahal_filter_full_state_timepts_car1_a,1) /simulations;
MSE_mahal_filter_full_state_timepts_preupdate_car1_a_1 = sum(MSE_mahal_filter_full_state_timepts_preupdate_car1_a,1) /simulations;

% % MSE_mahal_filter_full_state_timepts_car2_a_1 = sum(MSE_mahal_filter_full_state_timepts_car2_a,1) /simulations;
% % MSE_mahal_filter_full_state_timepts_preupdate_car2_a_1 = sum(MSE_mahal_filter_full_state_timepts_preupdate_car2_a,1) /simulations;
% % 
% % MSE_mahal_filter_full_state_timepts_car3_a_1 = sum(MSE_mahal_filter_full_state_timepts_car3_a,1) /simulations;
% % MSE_mahal_filter_full_state_timepts_preupdate_car3_a_1 = sum(MSE_mahal_filter_full_state_timepts_preupdate_car3_a,1) /simulations;


pause;
% % % 
% % % %Plot results of MC simulations
% % % figure;
% % % plot(mean(MSE_kalman, 1));
% % % title('kalman results');
% % % 
% % % figure;
% % % plot(mean(MSE_particle_filter, 1));
% % % title('particle filter results');
% % % 
% % % 
% % % 
% % % %% 
% % % %Plot results of MC simulations
% % % plot(states(1,:), states(2,:),'r.-');
% % % hold on;
% % % plot(measurements(1,:), measurements(2,:),'k.-');
% % % 
% % % 
% % % %Kalman filter plot
% % % plot(kalman_results_mean(1,:,ii), kalman_results_mean(2,:,ii), 'g.-');
% % % 
% % % 
% % % 
% % % 
% % % %particle filter plot
% % % par_flt_measurements = zeros(2,trials);
% % % ii=1;
% % % for jj = 1:trials
% % %     tmp1 = repmat(particle_fltr_results_wts(:,jj,1)',2,1);
% % %     tmp2 = measurement_params.mean + measurement_params.emission * particle_fltr_results_particles(:,:,jj,ii);
% % %     
% % %     par_flt_measurements(:,jj) = sum(tmp1 .* tmp2, 2);
% % %     
% % % end
% % % plot(par_flt_measurements(1,:), par_flt_measurements(2,:), 'b.-');
% % % 
% % % 
% % % 
% % % 
% % % 
% % % 
