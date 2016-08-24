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


% % % %transition_mat_full = zeros(2*num_particles, 3*num_particles);
% % % transition_mat_full = sparse(2*num_particles, 3*num_particles);
% % % 
% % % % % transition_mat_full = temporal_params.transition;
% % % for ii = 1: num_particles
% % %     transition_mat_full((ii-1)*2+1:ii*2, (ii-1)*3+1:ii*3) =temporal_params.transition; 
% % % end

% num_iterations = 500;
% all_collect = zeros(dim,num_iterations*simulations, trials);

% which model (number of cars) to keep track of (assume at best 3 models)
%but need to transiton to model set dynamically

 

%% Monte Carlo simulations
%tic;
for ii = 1: simulations
    ii;
    %clear('best_model','store');
%     if num_objects >1
%         tmp = mnrnd(1,[prob_occl, 1- prob_occl],trials);
%         obj_idx_occlude(car_idx_occlusion,:) =  tmp(:,2);  % occlusion for object car_idx_occlusion set earlier
%     end
%     

  
    %Reset simulator and generate temporal movements and subsequent measurements
    %set starting positions of objects
    separation_dist = global_vars.safe_dist + randn;
    %init_x = sort( unifrnd(0,20*global_vars.safe_dist*(num_objects-1),1, num_objects), 'descend');
    init_x =  fliplr(0.3*separation_dist*[0: (num_objects-1)]);%[10, 6,0];;[6*separation_dist, 4*separation_dist, 0]; 
    init_vel = unifrnd(0, global_vars.v_max, 1, num_objects); %[1.5,6, 9]; %; %temporal_params.mean_target_vel + 0.5*randn(1, num_objects); %unifrnd(0, global_vars.v_max, 1, num_objects); %gamrnd(22.5,1/1.5, 1, num_objects);
    init_accleration = unifrnd(0, global_vars.accn_max, 1, num_objects); %[-1.5,0,1]; %rand(1, num_objects);[-1,0.5,1];
    
    init_state = [init_x;init_vel;init_accleration];
    init_state = init_state(:);
    
    [states, distance_of_event] = oneD_temporal_model_v2_constrained(init_state, temporal_params, trials/global_vars.delta_t, num_objects, global_vars, event);
    %states = oneD_temporal_model_v2_no_interaction(init_state, temporal_params, trials/global_vars.delta_t, num_objects, global_vars);
%r=1;
    
    plot(states(1,1:100),'r.-');
    %plot(states(5,:),'.-');
    
    %pause;
   hold on;
   plot(states(4,1:100),'g.-');
%   hold off;
   
   plot(states(7,1:100),'b.-');
%     tmp_x = 1:1000;
%    fill([tmp_x,flip(tmp_x)],[global_vars.occlusion_blocs_cord(1,1)*ones(size(tmp_x)),  global_vars.occlusion_blocs_cord(1,2)*ones(size(tmp_x))],'k','LineStyle','none');
%     alpha(0.4);
    %plot([1:50],repmat(global_vars.occlusion_blocs_cord(1,1),50,1),'k-', 'LineWidth',2);
    %plot([1:50],repmat(global_vars.occlusion_blocs_cord(1,2),50,1),'k-', 'LineWidth',2);
%     xlabel('time points {1}/{10}th of a second'); ylabel('position of vehicle'); title('Plot of position of vehicles vs. time - with Occlusion Zones');
%     xlabel('time points {1}/{10}th of a second'); ylabel('position of vehicle'); title('Plot of position of vehicles vs. time');
%     xlabel('time points {1}/{10}th of a second'); ylabel('position of vehicle'); title('Plot of position of vehicles vs. time (Dependent Motion)');
xlabel('time points {1}/{10}th of a second'); ylabel('position of vehicle'); title('Plot of position of vehicles vs. time (Independent Motion)');

%    legend('Vehicle 1', 'Vehicle 2 (Unobserved)', 'Vehicle 3', 'Location','northwest');
     %legend('Vehicle 1', 'Vehicle 2', 'Location','northwest');
     legend('Vehicle 1', 'Vehicle 2', 'Vehicle 3', 'Location','northwest');
     hold off;
     pause;
%     figure;
%     plot(exp(log_Bayes_fac(1:50)),'.-');
%      %pause;
%      xlabel('time points'); ylabel('Bayes Factor : {M}_{2}/M_{3}'); title('Plot of Bayes factor ({M}_{2}/M_{3}) vs. time');
%      
%     drawnow;
% %     %pause;
%     r=1;
%     
    skip = 1/global_vars.delta_t;
    states_copy = states;
    states = states(:, 1:skip:end);
    
    obj_idx_occlude = logical(ones(size(states,1)/3, trials));
    %Set the occlusion positions
    tmp = states(1:3:end,:);
    for iter = 1 : size(global_vars.occlusion_blocs_cord,1)
        obj_idx_occlude = obj_idx_occlude & ((tmp < global_vars.occlusion_blocs_cord(iter,1)) | (tmp > global_vars.occlusion_blocs_cord(iter,2))) & ~isinf(tmp);

    end
    
    if any(global_vars.measurement_id_occlusion>0)
        % any particular car(s) is/are always occluded
       obj_idx_occlude( global_vars.measurement_id_occlusion,:) =0;
    end
    
    
    
    
    
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
     
    
    measurements = oneD_measurement_model_v2(states, measurement_params);
       
    
       % correct obj_idx_occlude for vehicle disappaerance/appearance
       %so if measuremnets is -Inf, this also means measurment not received ... treat
       %is as if occluded measurment .. so we are not using its value in calculations 
       obj_idx_occlude(isinf(measurements))=0;
       
       %Simialrly, For those measurements which are occluded, set them to -infinity
       measurements(~obj_idx_occlude) = -Inf;
       
        
%     %Run Kalman Filter on the measurements
%     [kalman_mean, ~] = Kalman_v1(measurements, temporal_params, measurement_params);
%     tmp = kalman_mean(1:2,:) - states(1:2,:);
%     MSE_kalman(ii,:) = sum(tmp .^2,1); 
    
    %Run Particle filter on the measurements
   % [posterior_states_mean, posterior_states_cov, num_fail, posterior_states_mean_preupdate, posterior_states_cov_preupdate, posterior_states_resamp, posterior_cov_resamp, all_particles, eff_sample_size, sum_wts_unnormalised, log_lik, wts_hold ] = oneD_Particle_filter_occlusion_v1_vectorised(init_state, obj_idx_occlude, measurements, temporal_params, measurement_params, num_particles, global_vars, states);

   model_sum_wts_unnormalised = zeros(numel(models_to_consider), trials);
   
   for model_num =1 : numel(models_to_consider)
    
    num_cars = models_to_consider(model_num);
    if num_cars < num_objects
        
        %assume observations of first and last car seen .. car 2 not seen
        %(generalise later)
                
        cars_from_start = (num_cars/2) + mod(num_cars,2);
        cars_from_end = (num_cars/2);
        
        init_state_model = init_state([1:3*cars_from_start, end- 3*cars_from_end+1:end]);
        obj_idx_occlude_model = obj_idx_occlude([1:cars_from_start, end- cars_from_end+1:end],:);
        measurements_model = measurements([1:cars_from_start, end- cars_from_end+1:end],:);
        
    elseif num_cars > num_objects
        %Assume (for now) some middle cars are missing ...generalise later
        
% %     init_x = fliplr(separation_dist*[0: (num_objects-1)]);
% %     init_vel = unifrnd(0, global_vars.v_max, 1, num_objects); %gamrnd(22.5,1/1.5, 1, num_objects);
% %     init_accleration = unifrnd(0, global_vars.accn_max, 1, num_objects); %rand(1, num_objects);
        
        %find position where we should assume middle car exists
        pos = ceil(num_objects/2);
        insert_states = zeros(3*(num_cars-num_objects),1);
        
        tmp_dist = (init_state(3*(pos-1)+1) - init_state(3*pos+1))/((num_cars-num_objects)+1);       
        tmp = init_state(3*(pos-1)+1) - [1:(num_cars-num_objects)]' *tmp_dist;
        
        insert_states(1:3:end) = tmp;
        insert_states(2:3:end) = unifrnd(0, global_vars.v_max, (num_cars-num_objects), 1);
        insert_states(3:3:end) = unifrnd(0, global_vars.accn_max, (num_cars-num_objects), 1);
        
        init_state_model = [init_state(1:3*pos); insert_states; init_state(3*pos+1:end)];
                
        obj_idx_occlude_model = [obj_idx_occlude(1:pos,:);logical(zeros((num_cars-num_objects), trials)) ;obj_idx_occlude(pos+1:end,:)];
        
        %add dummy zero measurements (for extra cars) as anyways we assume
        %these are occluded .. so these values wont be used
        measurements_model = [measurements(1:pos,:);zeros((num_cars-num_objects), trials) ;measurements(pos+1:end,:)];
 
        
        
    else
        init_state_model = init_state;
        obj_idx_occlude_model = obj_idx_occlude;
        plot_obj_idx_occlude = obj_idx_occlude;
        measurements_model = measurements;
    end
    
    if num_cars == num_objects
        [posterior_states_mean, posterior_states_cov, num_fail, posterior_states_mean_preupdate, posterior_states_cov_preupdate, posterior_states_resamp, posterior_cov_resamp, all_particles, eff_sample_size, sum_wts_unnormalised] = oneD_Particle_filter_occlusion_v1_vectorised(init_state_model, obj_idx_occlude_model, measurements_model, temporal_params, measurement_params, num_particles, global_vars, states);  
        
    else
        [~, ~, ~, ~, ~, ~, ~, ~, ~, sum_wts_unnormalised] = oneD_Particle_filter_occlusion_v1_vectorised(init_state_model, obj_idx_occlude_model, measurements_model, temporal_params, measurement_params, num_particles, global_vars, states);

    end
        
    model_sum_wts_unnormalised(model_num, :) = sum_wts_unnormalised;
         %Compute bayes factor (model selection)
    %[log_bayes_fac(t), fac_curr_period(t)] = bayes_fac(log_bayes_fac(t-1), temporal_params, measurement_params, measurements(obj_idx_occlude(:,t) ,t), modelA, modelB, global_vars, obj_idx_occlude(:,t));

    
   end
   
   
   %=====start of bayes fac calcs============ 
   
     
     log_Bayes_fac = zeros(numel(models_to_consider)-1, trials);
          for model_num =1 : numel(models_to_consider)-1

              log_Bayes_fac(model_num,:) = cumsum(log(model_sum_wts_unnormalised(model_num,:)) - log(model_sum_wts_unnormalised(model_num+1,:)));

          end

          %calcukate relative probs of the various models (taking care of
          %underflow/overflow)
          log_Bayes_fac(log_Bayes_fac>100)=100;

          tmp1 = flipud(cumprod(flipud(exp(log_Bayes_fac)),1));
          tmp2 = sum(tmp1,1)+1;

          tmp3 = 1./ tmp2;

          relative_probs = repmat(tmp3,numel(models_to_consider),1) .* [tmp1; ones(1, trials)];
          
 
    %=====end of bayes fac calcs============      
          
          
     %plot(exp(log_Bayes_fac(1:50)),'.-');
     %pause;
     %xlabel('time points'); ylabel('Bayes Factor : {M}_{2}/M_{3}'); title('Plot of Bayes factor ({M}_{2}/M_{3}) vs. time');
     


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

        MSE_mahal_filter_full_state_timepts_car2_x(ii,jj) = tmp(4,jj)' *pinv(posterior_states_cov(4,4,jj),tol) *tmp(4,jj);
        MSE_mahal_filter_full_state_timepts_preupdate_car2_x(ii,jj) = tmp(4,jj)' *pinv(posterior_states_cov_preupdate(4,4,jj),tol) *tmp(4,jj);

        MSE_mahal_filter_full_state_timepts_car3_x(ii,jj) = tmp(7,jj)' *pinv(posterior_states_cov(7,7,jj),tol) *tmp(7,jj);
        MSE_mahal_filter_full_state_timepts_preupdate_car3_x(ii,jj) = tmp(7,jj)' *pinv(posterior_states_cov_preupdate(7,7,jj),tol) *tmp(7,jj);

        MSE_mahal_filter_full_state_timepts_car1(ii,jj) = tmp(1:3,jj)' *pinv(posterior_states_cov(1:3,1:3,jj), tol) *tmp(1:3,jj);
        MSE_mahal_filter_full_state_timepts_preupdate_car1(ii,jj) = tmp(1:3,jj)' *pinv(posterior_states_cov_preupdate(1:3,1:3,jj), tol) *tmp(1:3,jj);

        MSE_mahal_filter_full_state_timepts_car2(ii,jj) = tmp(4:6,jj)' *pinv(posterior_states_cov(4:6,4:6,jj), tol) *tmp(4:6,jj);
        MSE_mahal_filter_full_state_timepts_preupdate_car2(ii,jj) = tmp(4:6,jj)' *pinv(posterior_states_cov_preupdate(4:6,4:6,jj), tol) *tmp(4:6,jj);

        MSE_mahal_filter_full_state_timepts_car3(ii,jj) = tmp(7:9,jj)' *pinv(posterior_states_cov(7:9,7:9,jj), tol) *tmp(7:9,jj);
        MSE_mahal_filter_full_state_timepts_preupdate_car3(ii,jj) = tmp(7:9,jj)' *pinv(posterior_states_cov_preupdate(7:9,7:9,jj), tol) *tmp(7:9,jj);

         
        MSE_mahal_filter_full_state_timepts(ii,jj) = tmp(:,jj)' *pinv(posterior_states_cov(:,:,jj), tol) *tmp(:,jj);
        MSE_mahal_filter_full_state_timepts_preupdate(ii,jj) = tmp(:,jj)' *pinv(posterior_states_cov_preupdate(:,:,jj), tol) *tmp(:,jj);

         
  
        
        
        MSE_mahal_filter_full_state_timepts_car1_v(ii,jj) = tmp(2,jj)' *pinv(posterior_states_cov(2,2,jj),tol) *tmp(2,jj);
        MSE_mahal_filter_full_state_timepts_preupdate_car1_v(ii,jj) = tmp(2,jj)' *pinv(posterior_states_cov_preupdate(2,2,jj)) *tmp(2,jj);

        MSE_mahal_filter_full_state_timepts_car2_v(ii,jj) = tmp(5,jj)' *pinv(posterior_states_cov(5,5,jj),tol) *tmp(5,jj);
        MSE_mahal_filter_full_state_timepts_preupdate_car2_v(ii,jj) = tmp(5,jj)' *pinv(posterior_states_cov_preupdate(5,5,jj),tol) *tmp(5,jj);

        MSE_mahal_filter_full_state_timepts_car3_v(ii,jj) = tmp(8,jj)' *pinv(posterior_states_cov(8,8,jj),tol) *tmp(8,jj);
        MSE_mahal_filter_full_state_timepts_preupdate_car3_v(ii,jj) = tmp(8,jj)' *pinv(posterior_states_cov_preupdate(8,8,jj),tol) *tmp(8,jj);

  
        
        
        MSE_mahal_filter_full_state_timepts_car1_a(ii,jj) = tmp(3,jj)' *pinv(posterior_states_cov(3,3,jj),tol) *tmp(3,jj);
        MSE_mahal_filter_full_state_timepts_preupdate_car1_a(ii,jj) = tmp(3,jj)' *pinv(posterior_states_cov_preupdate(3,3,jj)) *tmp(3,jj);

        MSE_mahal_filter_full_state_timepts_car2_a(ii,jj) = tmp(6,jj)' *pinv(posterior_states_cov(6,6,jj),tol) *tmp(6,jj);
        MSE_mahal_filter_full_state_timepts_preupdate_car2_a(ii,jj) = tmp(6,jj)' *pinv(posterior_states_cov_preupdate(6,6,jj),tol) *tmp(6,jj);

        MSE_mahal_filter_full_state_timepts_car3_a(ii,jj) = tmp(9,jj)' *pinv(posterior_states_cov(9,9,jj),tol) *tmp(9,jj);
        MSE_mahal_filter_full_state_timepts_preupdate_car3_a(ii,jj) = tmp(9,jj)' *pinv(posterior_states_cov_preupdate(9,9,jj),tol) *tmp(9,jj);

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

         MSE_mahal_filter_full_state_car2(ii) = MSE_mahal_filter_full_state_car2(ii) + MSE_mahal_filter_full_state_timepts_car2(ii,jj);    
         MSE_mahal_filter_full_state_preupdate_car2(ii) = MSE_mahal_filter_full_state_preupdate_car2(ii) + MSE_mahal_filter_full_state_timepts_preupdate_car2(ii,jj);

         MSE_mahal_filter_full_state_car3(ii) = MSE_mahal_filter_full_state_car3(ii) + MSE_mahal_filter_full_state_timepts_car3(ii,jj);    
         MSE_mahal_filter_full_state_preupdate_car3(ii) = MSE_mahal_filter_full_state_preupdate_car3(ii) + MSE_mahal_filter_full_state_timepts_preupdate_car3(ii,jj);

         MSE_mahal_filter_full_state_car1_x(ii) = MSE_mahal_filter_full_state_car1_x(ii) + MSE_mahal_filter_full_state_timepts_car1_x(ii,jj);    
         MSE_mahal_filter_full_state_preupdate_car1_x(ii) = MSE_mahal_filter_full_state_preupdate_car1_x(ii) + MSE_mahal_filter_full_state_timepts_preupdate_car1_x(ii,jj);

         MSE_mahal_filter_full_state_car2_x(ii) = MSE_mahal_filter_full_state_car2_x(ii) + MSE_mahal_filter_full_state_timepts_car2_x(ii,jj);    
         MSE_mahal_filter_full_state_preupdate_car2_x(ii) = MSE_mahal_filter_full_state_preupdate_car2_x(ii) + MSE_mahal_filter_full_state_timepts_preupdate_car2_x(ii,jj);

         MSE_mahal_filter_full_state_car3_x(ii) = MSE_mahal_filter_full_state_car3_x(ii) + MSE_mahal_filter_full_state_timepts_car3_x(ii,jj);    
         MSE_mahal_filter_full_state_preupdate_car3_x(ii) = MSE_mahal_filter_full_state_preupdate_car3_x(ii) + MSE_mahal_filter_full_state_timepts_preupdate_car3_x(ii,jj);


         
         MSE_mahal_filter_full_state_car1_v(ii) = MSE_mahal_filter_full_state_car1_v(ii) + MSE_mahal_filter_full_state_timepts_car1_v(ii,jj);    
         MSE_mahal_filter_full_state_preupdate_car1_v(ii) = MSE_mahal_filter_full_state_preupdate_car1_v(ii) + MSE_mahal_filter_full_state_timepts_preupdate_car1_v(ii,jj);

         MSE_mahal_filter_full_state_car2_v(ii) = MSE_mahal_filter_full_state_car2_v(ii) + MSE_mahal_filter_full_state_timepts_car2_v(ii,jj);    
         MSE_mahal_filter_full_state_preupdate_car2_v(ii) = MSE_mahal_filter_full_state_preupdate_car2_v(ii) + MSE_mahal_filter_full_state_timepts_preupdate_car2_v(ii,jj);

         MSE_mahal_filter_full_state_car3_v(ii) = MSE_mahal_filter_full_state_car3_v(ii) + MSE_mahal_filter_full_state_timepts_car3_v(ii,jj);    
         MSE_mahal_filter_full_state_preupdate_car3_v(ii) = MSE_mahal_filter_full_state_preupdate_car3_v(ii) + MSE_mahal_filter_full_state_timepts_preupdate_car3_v(ii,jj);

    
         
         
         MSE_mahal_filter_full_state_car1_a(ii) = MSE_mahal_filter_full_state_car1_a(ii) + MSE_mahal_filter_full_state_timepts_car1_a(ii,jj);    
         MSE_mahal_filter_full_state_preupdate_car1_a(ii) = MSE_mahal_filter_full_state_preupdate_car1_a(ii) + MSE_mahal_filter_full_state_timepts_preupdate_car1_a(ii,jj);

         MSE_mahal_filter_full_state_car2_a(ii) = MSE_mahal_filter_full_state_car2_a(ii) + MSE_mahal_filter_full_state_timepts_car2_a(ii,jj);    
         MSE_mahal_filter_full_state_preupdate_car2_a(ii) = MSE_mahal_filter_full_state_preupdate_car2_a(ii) + MSE_mahal_filter_full_state_timepts_preupdate_car2_a(ii,jj);

         MSE_mahal_filter_full_state_car3_a(ii) = MSE_mahal_filter_full_state_car3_a(ii) + MSE_mahal_filter_full_state_timepts_car3_a(ii,jj);    
         MSE_mahal_filter_full_state_preupdate_car3_a(ii) = MSE_mahal_filter_full_state_preupdate_car3_a(ii) + MSE_mahal_filter_full_state_timepts_preupdate_car3_a(ii,jj);

         
     end
     %MSE_mahal_filter_full_state(ii) = sum(MSE_mahal_filter_full_state_timepts(ii,:));
     
     
     num_failures = num_failures + num_fail;
     
     
     
      %  a = plot_anim(states, posterior_states_mean, measurements, logical(obj_idx_occlude), all_particles, global_vars, temporal_params, eff_sample_size);
  %a = plot_anim_v2(states,posterior_states_mean, measurements, obj_idx_occlude, all_particles, global_vars,  eff_sample_size, relative_probs, models_to_consider);  
%a = plot_anim_v2(states,posterior_states_mean, measurements, obj_idx_occlude, all_particles, global_vars,  eff_sample_size, relative_probs, models_to_consider);  


%       %       [posterior_states,fac, fac_curr_period] = oneD_Particle_filter_occlusion_bayes_fac_v1(init_state, logical(obj_idx_occlude), measurements, temporal_params, measurement_params, num_particles, global_vars);
%     % % % % 
%     % % % %     plot(fac,'o-'); title(['iteration= ',num2str(ii)]);pause(1);
%     % % % % 
%     % % % %     if any(diff(fac)<-5)
%     % % % %     
%     % % % %         pause(1);
%     % % % %        % a = plot_anim(states, posterior_states, measurements, logical(obj_idx_occlude));
%     % % % %     
%     % % % %     end
% 
    

   %  tmp_print = trimmean(MSE_mahal_filter_full_state_timepts(1:ii,:),1,1);
   %  tmp_print(end-10:end)

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
MSE_mean_filter_x_state = sum(sum((tmp(1:3:end,:) - repmat(tmp_mean(1:3:end),1,simulations* trials)).^2))/ (simulations*trials);
MSE_mean_filter_x_state_car1 = sum(sum((tmp(1,:) - repmat(tmp_mean(1),1,simulations* trials)).^2))/ (simulations*trials);
MSE_mean_filter_x_state_car2 = sum(sum((tmp(4,:) - repmat(tmp_mean(4),1,simulations* trials)).^2))/ (simulations*trials);
MSE_mean_filter_x_state_car3 = sum(sum((tmp(7,:) - repmat(tmp_mean(7),1,simulations* trials)).^2))/ (simulations*trials);

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


%calcukate only for 1 position
tmp_timepts = zeros(1, trials);
MSE_mean_filter_full_state_timepts = zeros(1,trials);
idx =9;
for jj = 1 : trials
    %flatten the 3D array
   tmp_timepts(:,jj) = mean(squeeze(error_est_filter_full_state(idx,jj,:)));
   MSE_mean_filter_full_state_timepts(jj) = sum(sum( (squeeze(error_est_filter_full_state(idx,jj,:))- repmat(tmp_timepts(:,jj),1,simulations)').^2));
end
MSE_mean_filter_full_state_timepts = MSE_mean_filter_full_state_timepts/simulations;





MSE_mahal_filter_full_state_timepts1 = sum(MSE_mahal_filter_full_state_timepts,1) /simulations;
%MSE_mahal_filter_full_state_timepts1 = median(MSE_mahal_filter_full_state_timepts,1);
%MSE_mahal_filter_full_state_timepts1 = trimmean(MSE_mahal_filter_full_state_timepts,10,1);
%MSE_mahal_filter_full_state_timepts1 = sum(sum(MSE_mahal_filter_full_state_timepts,1)) /(simulations*trials);

MSE_mahal_filter_full_state_timepts_preupdate1 = sum(MSE_mahal_filter_full_state_timepts_preupdate,1) /simulations;
%MSE_mahal_filter_full_state_timepts_preupdate1 = median(MSE_mahal_filter_full_state_timepts_preupdate,1) ;

%MSE_mahal_filter_full_state_timepts_preupdate1 = trimmean(MSE_mahal_filter_full_state_timepts_preupdate, 40,1);

MSE_mahal_filter_full_state_timepts_car1_1 = sum(MSE_mahal_filter_full_state_timepts_car1,1) /simulations;
MSE_mahal_filter_full_state_timepts_preupdate_car1_1 = sum(MSE_mahal_filter_full_state_timepts_preupdate_car1,1) /simulations;

MSE_mahal_filter_full_state_timepts_car2_1 = sum(MSE_mahal_filter_full_state_timepts_car2,1) /simulations;
MSE_mahal_filter_full_state_timepts_preupdate_car2_1 = sum(MSE_mahal_filter_full_state_timepts_preupdate_car2,1) /simulations;

MSE_mahal_filter_full_state_timepts_car3_1 = sum(MSE_mahal_filter_full_state_timepts_car3,1) /simulations;
MSE_mahal_filter_full_state_timepts_preupdate_car3_1 = sum(MSE_mahal_filter_full_state_timepts_preupdate_car3,1) /simulations;


MSE_mahal_filter_full_state_timepts_car1_x_1 = sum(MSE_mahal_filter_full_state_timepts_car1_x,1) /simulations;
MSE_mahal_filter_full_state_timepts_preupdate_car1_x_1 = sum(MSE_mahal_filter_full_state_timepts_preupdate_car1_x,1) /simulations;

MSE_mahal_filter_full_state_timepts_car2_x_1 = sum(MSE_mahal_filter_full_state_timepts_car2_x,1) /simulations;
MSE_mahal_filter_full_state_timepts_preupdate_car2_x_1 = sum(MSE_mahal_filter_full_state_timepts_preupdate_car2_x,1) /simulations;

MSE_mahal_filter_full_state_timepts_car3_x_1 = sum(MSE_mahal_filter_full_state_timepts_car3_x,1) /simulations;
MSE_mahal_filter_full_state_timepts_preupdate_car3_x_1 = sum(MSE_mahal_filter_full_state_timepts_preupdate_car3_x,1) /simulations;



MSE_mahal_filter_full_state_timepts_car1_v_1 = sum(MSE_mahal_filter_full_state_timepts_car1_v,1) /simulations;
MSE_mahal_filter_full_state_timepts_preupdate_car1_v_1 = sum(MSE_mahal_filter_full_state_timepts_preupdate_car1_v,1) /simulations;

MSE_mahal_filter_full_state_timepts_car2_v_1 = sum(MSE_mahal_filter_full_state_timepts_car2_v,1) /simulations;
MSE_mahal_filter_full_state_timepts_preupdate_car2_v_1 = sum(MSE_mahal_filter_full_state_timepts_preupdate_car2_v,1) /simulations;

MSE_mahal_filter_full_state_timepts_car3_v_1 = sum(MSE_mahal_filter_full_state_timepts_car3_v,1) /simulations;
MSE_mahal_filter_full_state_timepts_preupdate_car3_v_1 = sum(MSE_mahal_filter_full_state_timepts_preupdate_car3_v,1) /simulations;


MSE_mahal_filter_full_state_timepts_car1_a_1 = sum(MSE_mahal_filter_full_state_timepts_car1_a,1) /simulations;
MSE_mahal_filter_full_state_timepts_preupdate_car1_a_1 = sum(MSE_mahal_filter_full_state_timepts_preupdate_car1_a,1) /simulations;

MSE_mahal_filter_full_state_timepts_car2_a_1 = sum(MSE_mahal_filter_full_state_timepts_car2_a,1) /simulations;
MSE_mahal_filter_full_state_timepts_preupdate_car2_a_1 = sum(MSE_mahal_filter_full_state_timepts_preupdate_car2_a,1) /simulations;

MSE_mahal_filter_full_state_timepts_car3_a_1 = sum(MSE_mahal_filter_full_state_timepts_car3_a,1) /simulations;
MSE_mahal_filter_full_state_timepts_preupdate_car3_a_1 = sum(MSE_mahal_filter_full_state_timepts_preupdate_car3_a,1) /simulations;
