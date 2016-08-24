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
    fprintf('simulation number = %d for Run_Num = %d\n',ii, run_num);
    clear('best_model','store');
%     if num_objects >1
%         tmp = mnrnd(1,[prob_occl, 1- prob_occl],trials);
%         obj_idx_occlude(car_idx_occlusion,:) =  tmp(:,2);  % occlusion for object car_idx_occlusion set earlier
%     end
%     

  
    %Reset simulator and generate temporal movements and subsequent measurements
    %set starting positions of objects
    separation_dist = global_vars.safe_dist + randn;
    %init_x = sort( unifrnd(0,20*global_vars.safe_dist*(num_objects-1),1, num_objects), 'descend');
    init_x = fliplr(0.8*separation_dist*[0: (num_objects-1)]);
    init_vel = temporal_params.mean_target_vel + 0.5*randn(1, num_objects); %unifrnd(0, global_vars.v_max, 1, num_objects); %gamrnd(22.5,1/1.5, 1, num_objects);
    init_accleration = unifrnd(0, global_vars.accn_max, 1, num_objects); %rand(1, num_objects);
    
    init_state = [init_x;init_vel;init_accleration];
    init_state = init_state(:);
    
    [states, distance_of_event] = oneD_temporal_model_v2_constrained(init_state, temporal_params, trials/global_vars.delta_t, num_objects, global_vars, event);
   
    
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
   % [wts, particles] = oneD_Particle_filter_v1(measurements, temporal_params, measurement_params, num_particles, global_vars);
    
   
   %% Here call all particle filters for all candidate models to decide which one is best (via bayes factor)
  
   
   % Determine which models to conisder based on number of measurments recieved.
   % revise the models to consider before and after vehciles enter
   % occlusion zones
   
   len1=1;
   len2=0;
   tmp_max_hold = zeros(global_vars.num_occlusion_blocks+1,1);
   
   for model_iter = 1 :global_vars.num_occlusion_blocks+1
       
       
        %CORRECT MEASUREMENTS IF NECESSARY (FOR CAR DISAPPEARANCE ONLY)
       if event.type==-1
        first_occlusion_id = find(sum( obj_idx_occlude ,1)==0, 1,'first');
        if (first_occlusion_id>1)
            temp_hold_occl = obj_idx_occlude;
            temp_hold_meas = measurements;

            issue_ind = diff(temp_hold_occl(:,1:first_occlusion_id-1));

            if any(issue_ind(:)==-1) 

                for c = 1:  size(issue_ind,2)

                    if any(issue_ind(:,c)==-1) %then need to fix this
                        temp_hold_occl(2:end, c)= temp_hold_occl(1:end-1,c); 
                        temp_hold_occl(1, c) = 0;

                        temp_hold_meas(2:end, c) = temp_hold_meas(1:end-1,c);
                        temp_hold_meas(1, c) = -Inf;
                    end


                end

            obj_idx_occlude = temp_hold_occl;
            measurements = temp_hold_meas;


            end
        end
            
        
       elseif event.type==1
           %CORRECT MEASUREMENTS IF NECESSARY (FOR CAR APPEARANCE ONLY)
           first_occlusion_id = find(sum( obj_idx_occlude(1:end-1,:) ,1)==0, 1,'first');
        if ~isempty(first_occlusion_id) && (first_occlusion_id>1)
            
            if obj_idx_occlude(end,first_occlusion_id)==1
            
                temp_hold_occl = obj_idx_occlude;
                temp_hold_meas = measurements;
                
                second_occlusion_id = find(obj_idx_occlude(end,first_occlusion_id+1) ==0, 1,'first')-1;
                
                for c = first_occlusion_id:  first_occlusion_id + second_occlusion_id
                    temp_hold_occl(1:end-1, c)= temp_hold_occl(2:end,c); 
                    temp_hold_occl(end, c) = 0;

                    temp_hold_meas(1:end-1, c) = temp_hold_meas(2:end,c);
                    temp_hold_meas(end, c) = -Inf;
                end


                obj_idx_occlude = temp_hold_occl;
                measurements = temp_hold_meas;
            
            end
            
            
        end
        
       end
            
    
            
       
   % first model selection is pre first occlusion zone =====
   % === followed by model selection after vehicles emerge from each
   % occlusion zone =====
       tmp_max = 0;
   
       if model_iter ==1
           % find max. number of indep measurments till first occlusion
           % zone hit
           counter =1;
           %tmp_max = 0;
           %tmp_min=size(measurements(obj_idx_occlude(:, 1), 1),1);
           
           if global_vars.num_occlusion_blocks ==0
               global_vars.occlusion_blocs_cord(1,1) = Inf; %first occlsuion zone starts at Inf              
           end
           
           while ( (counter<=trials) && ~(isinf(measurements(1, counter))) )
                tmp_max = max( tmp_max, size(measurements(obj_idx_occlude(:, counter), counter),1) );
                %tmp_min = min(tmp_min, size(measurements(obj_idx_occlude(:, counter), counter),1) );
                counter = counter+1;
           end
           %if tmp_min == tmp_max
           models_to_consider = [tmp_max, tmp_max + 1];
           tmp_max_hold(model_iter) =tmp_max;
%            else
%                models_to_consider = [tmp_min:tmp_max +1];
%            end
           
           len2 = counter-1;
           
       else
           % find max. number of indep measurments after vehicles emerge from occlusion zone till 
           % they enter next occlusion zone or hits end of simulation time period
           
           %Note in this case model transitions from best model in previous
           %stage to some model in this stage.
           

%            tmp_idx = global_vars.occlusion_blocs_cord(model_iter+1,2);
%            tmp_idx2 = find(measurements(1,:)<tmp_idx,1, 'last') ;
            
           %tmp_idx = global_vars.occlusion_blocs_cord(model_iter-1,1);
           %counter = find(measurements(1,:)>=tmp_idx,1, 'first') ;
            
           %tmp_min=size(measurements(obj_idx_occlude(:, counter), counter), 1);
           
           
       
           
           len1 = len2;
           counter = len1+1;
           occlusion_state = obj_idx_occlude(1, counter);
            
           %continue til next occlusion start or end
           occlusion_state_change=0;
           while ((counter<=trials) && (occlusion_state_change <2) )
               
                tmp_max = max( tmp_max, size(measurements(obj_idx_occlude(:, counter), counter),1) );
                %tmp_min = min(tmp_min, size(measurements(obj_idx_occlude(:, counter), counter),1) );

                if obj_idx_occlude(1, counter) ~= occlusion_state
                    occlusion_state_change = occlusion_state_change+1;
                    occlusion_state = obj_idx_occlude(1, counter);                    
                end 

                counter = counter+1;
           end
           
           if (tmp_max+1- tmp_max_hold(model_iter-1)>=2)
               models_to_consider = [tmp_max];
               tmp_max_hold(model_iter) = tmp_max;
           else
                models_to_consider = [tmp_max, tmp_max + 1];
                tmp_max_hold(model_iter) = tmp_max;
           end
% %            if tmp_max >= prev_model
% %                models_to_consider = unique([prev_model: tmp_max+1]);
% %            else
% %                models_to_consider = [tmp_max : prev_model];
% % %            else
% % %                models_to_consider = [prev_model: tmp_max +1];
% %            end
           
            len2 = counter-2;
       end
       
       
       clear('tmp_max');
   
       model_sum_wts_unnormalised = zeros(numel(models_to_consider), len2-(len1-1));
       

       %Easy for first model selection ...before car enters occlusion
       %zone..
        if model_iter == 1
            
            %clear('store');
            for model_num = 1 : numel(models_to_consider)

                num_cars = models_to_consider(model_num); %now num_cars = tmp_max or tmp_max+1
            
                if num_cars == tmp_max_hold(model_iter)
                    tmp_obj_idx_occlude = repmat(obj_idx_occlude(:,1)',3,1);
                    tmp_obj_idx_occlude = tmp_obj_idx_occlude(:);
                    
                    try
                    init_state_model = init_state(tmp_obj_idx_occlude);
                    catch
                        pause;
                    end
                    obj_idx_occlude_model = obj_idx_occlude(obj_idx_occlude(:,1), 1:len2);
                    %plot_obj_idx_occlude = obj_idx_occlude;
                    measurements_model = measurements(obj_idx_occlude(:,1), 1:len2);
                    
                    store.(strcat('model_iter_',num2str(model_iter),'init_state_model',num2str(num_cars))) = init_state_model;
                    store.(strcat('model_iter_',num2str(model_iter),'obj_idx_occlude_model',num2str(num_cars))) = obj_idx_occlude_model;
                    store.(strcat('model_iter_',num2str(model_iter),'measurements_model',num2str(num_cars))) = measurements_model;
                    

                    
                else
                    %in this case we assume one measurment is occluded... dont
                    %know which one ... so have to try out all once we get an 
                    % overwhelming answer and then pursue that one seelcted
                    % data association further

                    %Select best model via data association only
                    %make a list of all possible data associations and prepare the parameters appropriately
                    init_state_model = zeros(num_cars*3, num_cars);
                    obj_idx_occlude_model = logical(zeros(num_cars, len2, num_cars));
                    measurements_model =-Inf(num_cars, len2, num_cars);

                    insert_states = zeros(3,1);
                    insert_states(2) = temporal_params.mean_target_vel + 0.1*randn; %, 1, 1);
                    
                    %insert_states(3) = unifrnd(0, global_vars.accn_max); %, (num_cars-num_objects), 1);
                    

                    
% %                     tmp_measurements_hold = -Inf(tmp_max, len1);
% %                     for tmp_iter = l:len1
% %                         for tmp_iter2 = 1: tmp_max
% %                             tmp_measurements2 = measurements(tmp_iter2,tmp_iter);
% %                             if ~isinf(tmp_measurements2)
% %                                 tmp_measurements_hold(tmp_iter2,tmp_iter)  = tmp_measurements2;
% %                             end
% %                         end
% %                     end
                    
                    
                    
                    for iter = 1: num_cars
                        %assume each car is occluded in turn for each model of
                        %data association
   
                        if iter ==1
                            tmp = unifrnd(init_state(1), init_state(1) + global_vars.safe_dist);       
                            %tmp = init_state(1) + tmp_dist;
                            insert_states(1) = tmp;
                            
                            
                            lb1 = global_vars.accn_min;
                            lb2 = -insert_states(2);

                            ub1 = global_vars.accn_max;
                            ub2 = global_vars.v_max - insert_states(2);
                            ub3 = Inf; %2*( (states(3*(jj-2)+1,ii) - states(3*(jj-1)+1,ii)) +(states(3*(jj-2)+2,ii) - states(3*(jj-1)+2,ii)) + 0.5*states(3*(jj-2),ii) );

                            %it is possible that ub3 < lb1 (less imp as a min can be changed) but also ub3<lb2 (more
                            %imp.) If ub3 <lb1 or ub3<lb2 we will not get a valid accle. In this
                            %case choose acc. to be the min. possible. In the next step the
                            %position if violated will be generated from uniform.
                            if (ub3<lb1) || (ub3<lb2)
                                insert_states(3) = lb2; %min. acc. possible (so as not to make vel. in next step -ve)

                            else
                                insert_states(3) = unifrnd(max(lb1,lb2), min([ub1, ub2, ub3]));

                            end

                            
                            tmp_obj_idx_occlude = repmat(obj_idx_occlude(:,1)',3,1);
                            tmp_obj_idx_occlude = tmp_obj_idx_occlude(:);
                            
                            init_state_model(:,iter) = [insert_states; init_state(tmp_obj_idx_occlude)];
                            obj_idx_occlude_model(:, :, iter) = [logical(zeros(1,len2)); ~isinf(measurements(1:tmp_max_hold(model_iter), 1:len2))];
                            
                            measurements_model(:, :, iter) = [-Inf(1,len2); measurements(1:tmp_max_hold(model_iter), 1:len2)];
                            
                        elseif iter == num_cars
                            
                            tmp_idx = find(obj_idx_occlude(:,1)>0,1,'last');
                            tmp = unifrnd(init_state(3*tmp_idx-2) - global_vars.safe_dist, init_state(3*tmp_idx-2) );       
                            %tmp = init_state(end-2) - tmp_dist;
                            insert_states(1) = tmp;
                            tmp1 = init_state(tmp_obj_idx_occlude);
                            
                            lb1 = global_vars.accn_min;
                            lb2 = -insert_states(2);

                            ub1 = global_vars.accn_max;
                            ub2 = global_vars.v_max - insert_states(2);
                            ub3 = 2*( (tmp1(end-2) - insert_states(1)) +(tmp1(end-1) - insert_states(2)) + 0.5*tmp1(end) );

                            %it is possible that ub3 < lb1 (less imp as a min can be changed) but also ub3<lb2 (more
                            %imp.) If ub3 <lb1 or ub3<lb2 we will not get a valid accle. In this
                            %case choose acc. to be the min. possible. In the next step the
                            %position if violated will be generated from uniform.
                            if (ub3<lb1) || (ub3<lb2)
                                insert_states(3) = lb2; %min. acc. possible (so as not to make vel. in next step -ve)

                            else
                                insert_states(3) = unifrnd(max(lb1,lb2), min([ub1, ub2, ub3]));

                            end
                            
                            
                            tmp_obj_idx_occlude = repmat(obj_idx_occlude(:,1)',3,1);
                            tmp_obj_idx_occlude = tmp_obj_idx_occlude(:);
                            
                            init_state_model(:,iter) = [init_state(tmp_obj_idx_occlude); insert_states];
                            obj_idx_occlude_model(:, :, iter) = [~isinf(measurements(1:tmp_max_hold(model_iter), 1:len2)); logical(zeros(1,len2))];
                            
                            measurements_model(:, :, iter) = [measurements(1:tmp_max_hold(model_iter), 1:len2); -Inf(1,len2)];
                            
                        else
                           tmp_idx = find(obj_idx_occlude(:,1)>0,iter,'first');
                           tmp_idx1 = tmp_idx(end-1);
                           tmp_idx2 = tmp_idx(end);
                           
                          
                           tmp_dist = init_state(3*tmp_idx1-2) - init_state(3*tmp_idx2-2);
                           %tmp_dist = (init_state(3*(iter-2)+1) - init_state(3*(iter-1)+1))/2;       
                           tmp = unifrnd(0, tmp_dist) + init_state(3*tmp_idx2-2);
                           insert_states(1) = tmp;
                           
                           lb1 = global_vars.accn_min;
                            lb2 = -insert_states(2);

                            ub1 = global_vars.accn_max;
                            ub2 = global_vars.v_max - insert_states(2);
                            ub3 = 2*( (init_state(3*tmp_idx1-2) - insert_states(1)) +(init_state(3*tmp_idx1-1)  - insert_states(2)) + 0.5*init_state(3*tmp_idx1)  );

                            %it is possible that ub3 < lb1 (less imp as a min can be changed) but also ub3<lb2 (more
                            %imp.) If ub3 <lb1 or ub3<lb2 we will not get a valid accle. In this
                            %case choose acc. to be the min. possible. In the next step the
                            %position if violated will be generated from uniform.
                            if (ub3<lb1) || (ub3<lb2)
                                insert_states(3) = lb2; %min. acc. possible (so as not to make vel. in next step -ve)

                            else
                                insert_states(3) = unifrnd(max(lb1,lb2), min([ub1, ub2, ub3]));

                            end
                            
                           
                           
                           tmp_hold = measurements(1:tmp_max_hold(model_iter), 1:len2);
                           
                           init_state_model(:,iter) = [init_state(1:3*tmp_idx1); insert_states; init_state(3*(tmp_idx2-1)+1:end)];
                           obj_idx_occlude_model(:, :, iter) = [~isinf(tmp_hold(1:tmp_idx1, :)); logical(zeros(1,len2)); ~isinf(tmp_hold(tmp_idx2:end, :))];  
                           measurements_model(:, :, iter) = [tmp_hold(1:tmp_idx1, :); -Inf(1,len2); tmp_hold(tmp_idx2:end, :)];
                             
                        end
                           
    
                    end
                        
                        
                    
                    
                    [model_selected, model_wts,posterior_mean] = select_data_association(init_state_model, obj_idx_occlude_model, measurements_model, temporal_params, measurement_params, num_particles, global_vars, states);
                    
                    
                    % prepare model parameters for seelcted data association
                    init_state_model = init_state_model(:, model_selected);
                    obj_idx_occlude_model = obj_idx_occlude_model(:, :, model_selected);
                    measurements_model = measurements_model(:, :, model_selected);
                           
                    store.(strcat('model_iter_',num2str(model_iter),'init_state_model',num2str(num_cars))) = init_state_model;
                    store.(strcat('model_iter_',num2str(model_iter),'obj_idx_occlude_model',num2str(num_cars))) = obj_idx_occlude_model;
                    store.(strcat('model_iter_',num2str(model_iter),'measurements_model',num2str(num_cars))) = measurements_model;
                    store.(strcat('model_iter_',num2str(model_iter),'last_est',num2str(num_cars))) = posterior_mean;
                    

                    model_sum_wts_unnormalised(model_num, :) = model_wts;

                end
            
            if sum(model_sum_wts_unnormalised(model_num, :))==0
                [posterior_mean, ~, ~, ~, ~, ~, ~, ~, ~, sum_wts_unnormalised,~] = oneD_Particle_filter_occlusion_v1_vectorised(init_state_model, obj_idx_occlude_model, measurements_model, temporal_params, measurement_params, num_particles, global_vars, states);  
                 model_sum_wts_unnormalised(model_num, :) = sum_wts_unnormalised;
                 posterior_mean = posterior_mean(:,end);
                 store.(strcat('model_iter_',num2str(model_iter),'last_est',num2str(num_cars))) = posterior_mean;
            end
             
            end
            
             
            
            %Now do model selection==========
            
             % Calculate Bayes factors and decide best model (of num. of cars)
             model_sum_wts_unnormalised(model_sum_wts_unnormalised==-1)=1; %setting to 1 makes their log zero (i.e no contribution)
             log_Bayes_fac = zeros(numel(models_to_consider)-1, len2);
              for model_num =1 : numel(models_to_consider)-1

                  log_Bayes_fac(model_num,:) = cumsum(log(model_sum_wts_unnormalised(model_num,:)) - log(model_sum_wts_unnormalised(model_num+1,:)));

              end

              %calcukate relative probs of the various models (taking care of
              %underflow/overflow)
              log_Bayes_fac(log_Bayes_fac>100)=100;

              tmp1 = flipud(cumprod(flipud(exp(log_Bayes_fac)),1));
              tmp2 = sum(tmp1,1)+1;

              tmp3 = 1./ (tmp2 + 1e-100);

              relative_probs = repmat(tmp3,numel(models_to_consider),1) .* [tmp1; ones(1, len2)];

              tmp_convergence = double([relative_probs>0.99]);
              thresh = 3;

              %if len2<50 %otherwise takes too long to run
              %  max_idx = seq_convergence_idx(tmp_convergence, thresh);
              %else
                max_idx = len2;
              %end
              
              best_model.num_cars(model_iter) = models_to_consider(relative_probs(:, max_idx)==max(relative_probs(:, max_idx)));

% %               if model_iter >1
% %                   best_model.event_type(model_iter) = best_model.num_cars -prev_model;
% % 
% %                    if best_model.event_type ~=0
% %                        % num of cars chnaged accoording to model selection in
% %                        % occlusion zone ---give estimate of when this happened
% %         %               
% %         %               %call optimisation routine
% %         %               best_model.dist(model_iter) = ;
% %                   end
% % 
% %               end

              %set prev_model to best model for next model selection procedure
              prev_model = best_model.num_cars(model_iter);

              %Now pass this model through the occlusion zone to get the
              %init_states for the next model selection step -- This assumes we
              %have selected the model correctly at this step .. should be so if
              %have seen enough interactions/observations
              model_id = best_model.num_cars(model_iter);
              
              init_state_model = store.(strcat('model_iter_',num2str(model_iter),'init_state_model',num2str(model_id)));
              obj_idx_occlude_model = store.(strcat('model_iter_',num2str(model_iter),'obj_idx_occlude_model',num2str(model_id)));
              measurements_model = store.(strcat('model_iter_',num2str(model_iter),'measurements_model',num2str(model_id)));

              %[init_state_est, ~, ~, ~, ~, ~, ~, ~, ~, ~] = oneD_Particle_filter_occlusion_v1_vectorised(init_state_model, obj_idx_occlude_model, measurements_model, temporal_params, measurement_params, num_particles, global_vars, states);  

              %init_state_est = init_state_est(:, end);
              init_state_est = store.(strcat('model_iter_',num2str(model_iter),'last_est',num2str(model_id))) ;
              obj_idx_occlude_est = obj_idx_occlude_model(:,end);
              measurements_model_est = measurements_model(:,end);

            
            
            

        else
            
            
           %% ====model selection ANYIMTE after the first occlusion zone (MUCH MORE DIFFICULT)
           %clear('tmp_max');
          for model_num = 1 : numel(models_to_consider)
              

            num_cars = models_to_consider(model_num); 
            
                if num_cars == prev_model
                    init_state_model = init_state_est;
                    
                    obj_idx_occlude_model = logical(zeros(prev_model, len2-(len1-1)));
                    measurements_model = -Inf(prev_model, len2-(len1-1));
                    %params.start = 0;
                    %params.end = 0;
                    %params.dim_change = 0;
                    
                    
                    %find first time all cars occluded
                    occl_idx = find(sum( ~isinf(measurements(:,len1:end)),1 )==0,1,'first' )-1;
                    tmp_obj_occlude = repmat(obj_idx_occlude_est, 1, occl_idx);
                    try
                    tmp_obj_occlude = tmp_obj_occlude & obj_idx_occlude(1:size(obj_idx_occlude_est,1), len1: len1-1+occl_idx);
                    catch
                       pause; 
                    end
                    
                    obj_idx_occlude_model(:,1:occl_idx) = repmat(tmp_obj_occlude,1,1);


                    tmp_measurements = measurements(:, len1:len2);
                    for tmp_iter =1:occl_idx
                        
                        for tmp_iter2 =1:prev_model
                            tmp_measurements2 = tmp_measurements(tmp_iter2,tmp_iter);
                            if ~isinf(tmp_measurements2)                                
                                measurements_model(tmp_iter2,tmp_iter) = tmp_measurements2;

                            end
                        end
                    end
                    
                    
                    
                    
                    if num_cars == tmp_max_hold(model_iter)
                        % so this model says no event happened in occlusion
                        % zone 
                        %----------------------------------

                        for tmp_iter = (len1-1)+occl_idx+1:len2
                            for tmp_iter2 = 1: tmp_max_hold(model_iter)
                                tmp_measurements2 = measurements(tmp_iter2,tmp_iter);
                                if ~isinf(tmp_measurements2)
                                    measurements_model(tmp_iter2,tmp_iter-(len1-1))  = tmp_measurements2;
                                    obj_idx_occlude_model(tmp_iter2,tmp_iter-(len1-1)) = logical(1);
                                end
                            end
                        end
                                        
                    
                    
                        store.(strcat('model_iter_',num2str(model_iter),'init_state_model',num2str(num_cars))) = init_state_model;
                        store.(strcat('model_iter_',num2str(model_iter),'obj_idx_occlude_model',num2str(num_cars))) = obj_idx_occlude_model;
                        store.(strcat('model_iter_',num2str(model_iter),'measurements_model',num2str(num_cars))) = measurements_model;
                    
                    else
                        % we are saying that one vehicle is occluded
                        % So need to do data association to figure out
                        % which vehicle being occluded is the best
                        % candidate model. Here num_cars = tmp_max+1
                        
                        %===============================
                    %in this case we assume one measurment is occluded... dont
                    %know which one ... so have to try out all once we get an 
                    % overwhelming answer and then pursue that one seelcted
                    % data association further

                    %Select model via data association only
                    %make a list of all possible data associations and prepare the parameters appropriately
                    init_state_model = repmat(init_state_model,1,num_cars);
                    obj_idx_occlude_model = repmat(obj_idx_occlude_model, 1,1,num_cars);
                    measurements_model =repmat(measurements_model, 1,1,num_cars);

                    
                    tmp_measurements_hold = -Inf(tmp_max_hold(model_iter), len2-occl_idx-(len1-1));
                    for tmp_iter = (len1-1)+occl_idx+1:len2
                        
                        for tmp_iter2 = 1: tmp_max_hold(model_iter)
                            tmp_measurements2 = measurements(tmp_iter2,tmp_iter);
                            if ~isinf(tmp_measurements2)                             
                                tmp_measurements_hold(tmp_iter2,tmp_iter-(len1-1)-occl_idx)  = tmp_measurements2;
                            end
                        end
                    end
                    
                    
                    
                    for iter = 1: num_cars
                        %assume each car is occluded in turn for each model of
                        %data association
   
                        if iter ==1
                            
                            obj_idx_occlude_model(:, occl_idx+1:end, iter) = [logical(zeros(1,len2-(len1-1)-occl_idx)); ~isinf(tmp_measurements_hold)];
                            measurements_model(:, occl_idx+1:end, iter) = [-Inf(1,len2-(len1-1)-occl_idx); tmp_measurements_hold];
                            
                            %params.start(iter) =2;
                            
                            %first time when u see the first measurements (after
                            %occlusion) thatis the max. time till when evnt can
                            %take place
                            %params.end(iter) = find(sum(obj_idx_occlude_model(:, occl_idx+1:end, iter),1)>0,1,'first')-1+occl_idx;
                            
                            
                            
                        elseif iter == num_cars

                            obj_idx_occlude_model(:, occl_idx+1:end, iter) = [~isinf(tmp_measurements_hold); logical(zeros(1,len2-(len1-1)-occl_idx))];
                            measurements_model(:, occl_idx+1:end, iter) = [tmp_measurements_hold; -Inf(1,len2-(len1-1)-occl_idx)];
                            
                            %params.start(iter) = find(obj_idx_occlude_model(iter, :, iter)==0, 1,'first');
                            %params.end(iter) = find(sum(obj_idx_occlude_model(:, occl_idx+1:end, iter),1)==tmp_max,1,'first')-1+occl_idx;                            
                            
                        else
                           
                           tmp_idx = find(obj_idx_occlude_est>0,iter,'first');
                           tmp_idx1 = tmp_idx(end-1);
                           tmp_idx2 = tmp_idx(end);
                           
                                                     
                           obj_idx_occlude_model(:, occl_idx+1:end, iter) = [~isinf(tmp_measurements_hold(1:tmp_idx1,:) ); logical(zeros(1,len2-occl_idx-(len1-1) )); ~isinf(tmp_measurements_hold(tmp_idx2:end,:) )];  
                           
                           measurements_model(:, occl_idx+1:end, iter) = [tmp_measurements_hold(1:tmp_idx1,:); -Inf(1,len2-occl_idx-(len1-1)); tmp_measurements_hold(tmp_idx2:end, :)];
                                                                                  
                          % params.start(iter) = find(obj_idx_occlude_model(iter, :, iter)==0, 1,'first');
                           %params.end(iter) = find(sum(obj_idx_occlude_model(:, occl_idx+1:end, iter),1)>=iter,1,'first')-1+occl_idx;
                            
                           
                        end
                              
                    end
                        
                    
                    [model_selected, model_wts, posterior_mean] = select_data_association(init_state_model, obj_idx_occlude_model, measurements_model, temporal_params, measurement_params, num_particles, global_vars, states);
                    
                    
                    % prepare model parameters for seelcted data association
                    init_state_model = init_state_model(:, model_selected);
                    obj_idx_occlude_model = obj_idx_occlude_model(:, :, model_selected);
                    measurements_model = measurements_model(:, :, model_selected);
                           
                    store.(strcat('model_iter_',num2str(model_iter),'init_state_model',num2str(num_cars))) = init_state_model;
                    store.(strcat('model_iter_',num2str(model_iter),'obj_idx_occlude_model',num2str(num_cars))) = obj_idx_occlude_model;
                    store.(strcat('model_iter_',num2str(model_iter),'measurements_model',num2str(num_cars))) = measurements_model;
                    store.(strcat('model_iter_',num2str(model_iter),'last_est',num2str(num_cars))) = posterior_mean;
                    
                    model_sum_wts_unnormalised(model_num, :) = model_wts; 
                        
                        %===============================
                        
                    end
                    

                elseif (num_cars == prev_model-1)
                    %in this case either num_cars is larger or smaller ... 
                    %if num_cars is larger than measurements then as beforeassume one measurment is occluded... dont
                    %know which one ... so have to try out all once we get an 
                    % overwhelming answer and then pursue that one seelcted
                    % by data association further
                    
                    
                    % in this case num_cars is equal to tmp_max
                    % So need to do data association to figure out which
                    % vehcile died  .. i.e choose best model                  
                    
                    %Select model via data association only ... but also
                    %need to assume a distance of event
                    %make a list of all possible data associations and prepare the parameters appropriately
                    
                    
                    %obj_idx_occlude_model = obj_idx_occlude(:, len1:len2);
                    %measurements_model = measurements(:, len1:len2);
                                          
                    init_state_model = repmat(init_state_est,1, prev_model);
                    obj_idx_occlude_model = logical(zeros(prev_model, len2-(len1-1), prev_model));
                    measurements_model =-Inf(prev_model, len2-(len1-1), prev_model);
                    params.start = zeros(prev_model,1);
                    params.end = zeros(prev_model,1);
                    params.dim_change = -1;
                    
                    %obj_idx_occlude_model(:, 1, :) = repmat(obj_idx_occlude_est,1,1,prev_model);
                    %measurements_model(:,1,:) = repmat(measurements_model_est,1,1,prev_model); %measurements(:, len1);
                    
                    %find first time all cars occluded
                    occl_idx = find(sum( ~isinf(measurements(:,len1:end)),1 )==0,1,'first' )-1;
                    tmp_obj_occlude = repmat(obj_idx_occlude_est, 1, occl_idx);
                    try
                    tmp_obj_occlude = tmp_obj_occlude & obj_idx_occlude(1:size(obj_idx_occlude_est,1), len1: len1-1+occl_idx);
                    catch
                      % pause; 
                        error_caught=1;
                        err_total = err_total+1;
                        break; 
                    end
                    
                    
                    
                    %correct tmp_obj_occlude for any "inconsistencies" 
                    %(caused by as if cars appearing after they have disappeared"
                    %this happens if event occurs before the last car
                    %enters the occlusion zone .. also correct for
                    %measurements simultaneosuly
                    tmp_measurements = measurements(:, len1:len2);
                    
                    for fix_iter_row=1: size(tmp_obj_occlude,1)-1
                        for fix_iter_col = 1: size(tmp_obj_occlude,2)
                            %find first occurance of occlusion
                            occ_first = find(tmp_obj_occlude(fix_iter_row, :)==0,1,'first');
                            %find if it remains ot be occluded
                            occ_status = find(tmp_obj_occlude(fix_iter_row, occ_first+1:end)==1,1,'first');
                            
                            if~isempty(occ_status)
                               %need to fix  tmp_obj_occlude
                               tmp_obj_occlude(fix_iter_row+1, occ_first+occ_status:end) = tmp_obj_occlude(fix_iter_row, occ_first+occ_status:end);
                               tmp_obj_occlude(fix_iter_row, occ_first+occ_status:end) =0;
                               try
                               tmp_measurements(fix_iter_row+1, occ_first+occ_status:occl_idx) = tmp_measurements(fix_iter_row, occ_first+occ_status:occl_idx);
                               tmp_measurements(fix_iter_row, occ_first+occ_status:occl_idx) =-Inf;
                               catch
                                   pause;
                               end
                                
                            end
                            
                            
                        end
                    end
                    
                    
                    obj_idx_occlude_model(:,1:occl_idx,:) = repmat(tmp_obj_occlude,1,1,prev_model);
                    
                    
                    for tmp_iter =1:occl_idx
                        for tmp_iter2 =1:prev_model
                            tmp_measurements2 = tmp_measurements(tmp_iter2,tmp_iter);
                            if tmp_obj_occlude(tmp_iter2,tmp_iter,1)>0
                                measurements_model(tmp_iter2,tmp_iter,:) = repmat(tmp_measurements2,1,1,prev_model);
                            end
                        end
                    end
                    
                    
                    
                    tmp_measurements_hold = -Inf(num_cars, len2-occl_idx-(len1-1));
                    for tmp_iter = (len1-1)+occl_idx+1:len2
                        for tmp_iter2 = 1: tmp_max_hold(model_iter)
                            tmp_measurements2 = tmp_measurements(tmp_iter2,tmp_iter-(len1-1));
                            if ~isinf(tmp_measurements2)
                                tmp_measurements_hold(tmp_iter2,tmp_iter-(len1-1)-occl_idx)  = tmp_measurements2;
                            end
                        end
                    end
                        
                     
                    
                     
                     
                     for iter = 1: prev_model
                        %assume each car has died in turn for each model of
                        %data association
   
                        if iter ==1
                            obj_idx_occlude_model(:, occl_idx+1:end, iter) = [logical( zeros(1,len2-occl_idx-(len1-1) )); ~isinf(tmp_measurements_hold) ];

                            measurements_model(:, occl_idx+1:end, iter) = [-Inf(1,len2-occl_idx-(len1-1)); tmp_measurements_hold];
                            
                            params.start(iter) =2;
                            
                            %first time when u see all the measurements (after
                            %occlusion) thatis the max. time till when evnt can
                            %take place
                            try
                            params.end(iter) = find(sum(obj_idx_occlude_model(:, occl_idx+1:end, iter),1)>0,1,'first')-1+occl_idx;
                            catch
                                pause;
                            end
                            %params.end(2:end) = params.end(1);
                            
                        elseif iter == prev_model
                            obj_idx_occlude_model(:, occl_idx+1:end, iter) = [~isinf(tmp_measurements_hold); logical(zeros(1,len2-occl_idx-(len1-1) ))];                     
                            
                            measurements_model(:, occl_idx+1:end, iter) = [tmp_measurements_hold; -Inf(1,len2-occl_idx-(len1-1))];
                            
                            params.start(iter) = find(obj_idx_occlude_model(iter, :, iter)==0, 1,'first');
                            params.end(iter) = find(sum(obj_idx_occlude_model(:, occl_idx+1:end, iter),1)==tmp_max_hold(model_iter),1,'first')-1+occl_idx;
                            
                        else
                           tmp_idx = find(obj_idx_occlude_est>0,iter,'first');
                           tmp_idx1 = tmp_idx(end-1);
                           tmp_idx2 = tmp_idx(end);
                           
                                                     
                           obj_idx_occlude_model(:, occl_idx+1:end, iter) = [~isinf(tmp_measurements_hold(1:tmp_idx1,:) ); logical(zeros(1,len2-occl_idx-(len1-1) )); ~isinf(tmp_measurements_hold(tmp_idx2:end,:) )];  
                           
                           measurements_model(:, occl_idx+1:end, iter) = [tmp_measurements_hold(1:tmp_idx1,:); -Inf(1,len2-occl_idx-(len1-1)); tmp_measurements_hold(tmp_idx2:end, :)];
                           
                           params.start(iter) = find(obj_idx_occlude_model(iter, :, iter)==0, 1,'first');
                           params.end(iter) = find(sum(obj_idx_occlude_model(:, occl_idx+1:end, iter),1)>=iter,1,'first')-1+occl_idx;
                  
                        end
                           
    
                      end
                    
  
                    
                    
% % %                     params.start = 2;
% % %                     
% % % % % %                     %find how long till last car leaves occlusion zone
% % %                     tmp_counter=0;
% % %                     while(obj_idx_occlude_model(end-tmp_counter,end)~=1)
% % %                         tmp_counter = tmp_counter+1;
% % %                     end
% % %                     params.end = find(obj_idx_occlude_model(end-tmp_counter, :)==0,1,'last');
% % %                     
                    
                    [model_selected, d, model_wts, prob_d, posterior_mean, est_dist, best_model2, est_dist2] = select_data_association_with_dist(init_state_model, obj_idx_occlude_model, measurements_model, temporal_params, measurement_params, num_particles, global_vars, states, params, model_iter-1);
                    
                                        
                    % prepare model parameters for seelcted data association
                    init_state_model = init_state_model(:, model_selected);
                    obj_idx_occlude_model = obj_idx_occlude_model(:, :, model_selected);
                    measurements_model = measurements_model(:, :, model_selected);
  
                    %Store these numbers if required in future
                    store.(strcat('model_iter_',num2str(model_iter),'init_state_model',num2str(num_cars))) = init_state_model;
                    store.(strcat('model_iter_',num2str(model_iter),'obj_idx_occlude_model',num2str(num_cars))) = obj_idx_occlude_model;
                    store.(strcat('model_iter_',num2str(model_iter),'measurements_model',num2str(num_cars))) = measurements_model;
                    store.(strcat('model_iter_',num2str(model_iter),'dist',num2str(num_cars))) = d;
                    store.(strcat('model_iter_',num2str(model_iter),'car_affected',num2str(num_cars))) =model_selected;
                    store.(strcat('model_iter_',num2str(model_iter),'prob_d',num2str(num_cars))) =prob_d;
                    store.(strcat('model_iter_',num2str(model_iter),'last_est',num2str(num_cars))) = posterior_mean;
                    store.(strcat('model_iter_',num2str(model_iter),'est_dist',num2str(num_cars))) = est_dist;
                    store.(strcat('model_iter_',num2str(model_iter),'car_affected2',num2str(num_cars))) = best_model2;
                    store.(strcat('model_iter_',num2str(model_iter),'est_dist2',num2str(num_cars))) = est_dist2;
                    
                    model_sum_wts_unnormalised(model_num, :) = model_wts;
                    
                   
                elseif ( (num_cars == prev_model+1) && (num_cars == tmp_max_hold(model_iter)) )
                    %????????????????????+++++++++++++==============
                    % In this case model is num_cars increase by 1. We know
                    % as number_of_obs is equal to num_cars and model
                    % dimension has increased by 1 ... there must be 1
                    % extra car. So just find which car is new (assumes
                    % more than 1 event cant happen)
                    
                    
                    
                    init_state_model = repmat(init_state_est,1, prev_model+1);
                    obj_idx_occlude_model = logical(zeros(prev_model+1, len2-(len1-1), prev_model+1));
                    measurements_model =-Inf(prev_model+1, len2-(len1-1), prev_model+1);
                    params.start = zeros(prev_model+1,1);
                    params.end = zeros(prev_model+1,1);
                    params.dim_change = 1;
                    
                    %obj_idx_occlude_model(:, 1, :) = repmat(obj_idx_occlude_est,1,1,prev_model);
                    %measurements_model(:,1,:) = repmat(measurements_model_est,1,1,prev_model); %measurements(:, len1);
                    
                    %find first time all cars occluded
                    occl_idx = find(sum( ~isinf(measurements(:,len1:end)),1 )==0,1,'first' )-1;
                    if isempty(occl_idx)
                        occl_idx = find(isinf(measurements(size(obj_idx_occlude_est,1), len1:end)),1,'first' )-1;
                    end
                    tmp_obj_occlude = repmat(obj_idx_occlude_est, 1, occl_idx);
                    try
                    tmp_obj_occlude = tmp_obj_occlude & obj_idx_occlude(1:size(obj_idx_occlude_est,1), len1: len1-1+occl_idx);
                    catch
                       pause; 
                    end
                    
                    %test of new code to correct ofr possible
                    %inconsitenices in tmp_obj_occlude
                     %measurements simultaneosuly
                    tmp_measurements = measurements(:, len1:len2);
                    
                    %look at the next row of tmpmeasurements till
                    %occlusion... it should all be -Inf .. otherwise fix.
                    if any( ~isinf(tmp_measurements(num_cars,1:occl_idx)) )
                       
                        %find the corresponding col. numbers and shift them
                        %up and continue till u hit a row which has no
                        %infinte measurements
                        c=1;
                        occ_first = find(~isinf(tmp_measurements(num_cars,1:occl_idx)),1,'first');
                        ori = tmp_measurements(num_cars,occ_first:occl_idx);
                        tmp_measurements(num_cars,occ_first:occl_idx) = -Inf;
                        while((c<num_cars))                           
                           replaced = tmp_measurements(num_cars-c,occ_first:occl_idx);
                           tmp_measurements(num_cars-c,occ_first:occl_idx) = ori;
                           c=c+1;
                           
                           if all(isinf(replaced))
                               break;
                           else
                               ori = replaced;
                           end
                           
                        end
                        
                       tmp_obj_occlude = ~isinf(tmp_measurements(1:size(tmp_obj_occlude,1),1:occl_idx)); 
                    end
                    
% % %                     
% % %                     for fix_iter_row=1: size(tmp_obj_occlude,1)-1
% % %                         for fix_iter_col = 1: size(tmp_obj_occlude,2)
% % %                             %find first occurance of occlusion
% % %                             occ_first = find(tmp_obj_occlude(fix_iter_row, :)==0,1,'first');
% % %                             %find if it remains ot be occluded
% % %                             occ_status = find(tmp_obj_occlude(fix_iter_row, occ_first+1:end)==1,1,'first');
% % %                             
% % %                             if~isempty(occ_status)
% % %                                %need to fix  tmp_obj_occlude
% % %                                tmp_obj_occlude(fix_iter_row+1, occ_first+occ_status:end) = tmp_obj_occlude(fix_iter_row, occ_first+occ_status:end);
% % %                                tmp_obj_occlude(fix_iter_row, occ_first+occ_status:end) =0;
% % %                                
% % %                                tmp_measurements(fix_iter_row+1, occ_first+occ_status:occl_idx) = tmp_measurements(fix_iter_row, occ_first+occ_status:occl_idx);
% % %                                tmp_measurements(fix_iter_row, occ_first+occ_status:occl_idx) =-Inf;
% % %                                
% % %                                 
% % %                             end
% % %                             
% % %                             
% % %                         end
% % %                     end
                    % end of checking inconsistencies code ....
                    
                    
                    
                    
                    
                    obj_idx_occlude_model(1:size(tmp_obj_occlude,1),1:occl_idx,:) = repmat(tmp_obj_occlude,1,1,prev_model+1);
                    
                    %tmp_measurements = measurements(:, len1:len2);
                    for tmp_iter =1:occl_idx
                        for tmp_iter2 =1:prev_model
                            tmp_measurements2 = tmp_measurements(tmp_iter2,tmp_iter);
                            if tmp_obj_occlude(tmp_iter2,tmp_iter,1)>0
                                measurements_model(tmp_iter2,tmp_iter,:) = repmat(tmp_measurements2,1,1,prev_model+1);
                            end
                        end
                    end
                    
                    
                    
                    tmp_measurements_hold = -Inf(tmp_max_hold(model_iter), len2-occl_idx-(len1-1));
                    for tmp_iter = (len1-1)+occl_idx+1:len2
                        for tmp_iter2 = 1: tmp_max_hold(model_iter)
                            tmp_measurements2 = tmp_measurements(tmp_iter2,tmp_iter-(len1-1));
                            if ~isinf(tmp_measurements2)
                                tmp_measurements_hold(tmp_iter2,tmp_iter-(len1-1)-occl_idx)  = tmp_measurements2;
                            end
                        end
                    end
                        
                     
                    
                     
                     for iter = 1: num_cars
                        %assume each car is new in turn for each model of
                        %data association
                            obj_idx_occlude_model(:, occl_idx+1:end, iter) = ~isinf(tmp_measurements_hold);
                            measurements_model(:, occl_idx+1:end, iter) =  tmp_measurements_hold;
                            
                        if iter ==1
                            
                            
                            
                            params.start(iter) =2;
                            
                            %first time when u see all the measurements (after
                            %occlusion) thatis the max. time till when evnt can
                            %take place
                            params.end(iter) = find(sum(obj_idx_occlude_model(:, occl_idx+1:end, iter),1)>0,1,'first')-1+occl_idx;
                            %params.end(2:end) = params.end(1);
                            
                        elseif iter == num_cars
                            %obj_idx_occlude_model(:, occl_idx+1:end, iter) = [~isinf(tmp_measurements_hold); logical(zeros(1,len2-occl_idx-(len1-1) ))];                     
                            
                            %measurements_model(:, occl_idx+1:end, iter) = [tmp_measurements_hold; -Inf(1,len2-occl_idx-(len1-1))];
                            
                            params.start(iter) = max(params.start(iter-1),find(obj_idx_occlude_model(iter, :, iter)==0, 1,'first'));
                            params.end(iter) = find(sum(obj_idx_occlude_model(:, occl_idx+1:end, iter),1)==tmp_max_hold(model_iter),1,'first')-1+occl_idx;
                            
                        else
                           tmp_idx = find(obj_idx_occlude_est>0,iter,'first');
                           tmp_idx1 = tmp_idx(end-1);
                           tmp_idx2 = tmp_idx(end);
                           
                                                     
                           %obj_idx_occlude_model(:, occl_idx+1:end, iter) = [~isinf(tmp_measurements_hold(1:tmp_idx1,:) ); logical(zeros(1,len2-occl_idx-(len1-1) )); ~isinf(tmp_measurements_hold(tmp_idx2:end,:) )];  
                           
                           %measurements_model(:, occl_idx+1:end, iter) = [tmp_measurements_hold(1:tmp_idx1,:); -Inf(1,len2-occl_idx-(len1-1)); tmp_measurements_hold(tmp_idx2:end, :)];
                           
                           params.start(iter) = find(obj_idx_occlude_model(iter, :, iter)==0, 1,'first');
                           params.end(iter) = find(sum(obj_idx_occlude_model(:, occl_idx+1:end, iter),1)>=iter,1,'first')-1+occl_idx;
                  
                        end
                           
    
                      end
                    
  
                                    
                    
                    [model_selected, d, model_wts, prob_d, posterior_mean, est_dist, best_model2, est_dist2] = select_data_association_with_dist(init_state_model, obj_idx_occlude_model, measurements_model, temporal_params, measurement_params, num_particles, global_vars, states, params, model_iter-1);
                    
                                        
                    % prepare model parameters for seelcted data association
                    init_state_model = init_state_model(:, model_selected);
                    obj_idx_occlude_model = obj_idx_occlude_model(:, :, model_selected);
                    measurements_model = measurements_model(:, :, model_selected);
  
                    %Store these numbers if required in future
                    store.(strcat('model_iter_',num2str(model_iter),'init_state_model',num2str(num_cars))) = init_state_model;
                    store.(strcat('model_iter_',num2str(model_iter),'obj_idx_occlude_model',num2str(num_cars))) = obj_idx_occlude_model;
                    store.(strcat('model_iter_',num2str(model_iter),'measurements_model',num2str(num_cars))) = measurements_model;
                    store.(strcat('model_iter_',num2str(model_iter),'dist',num2str(num_cars))) = d;
                    store.(strcat('model_iter_',num2str(model_iter),'car_affected',num2str(num_cars))) =model_selected;
                    store.(strcat('model_iter_',num2str(model_iter),'prob_d',num2str(num_cars))) =prob_d;
                    store.(strcat('model_iter_',num2str(model_iter),'last_est',num2str(num_cars))) = posterior_mean;
                    store.(strcat('model_iter_',num2str(model_iter),'est_dist',num2str(num_cars))) = est_dist;
                    store.(strcat('model_iter_',num2str(model_iter),'car_affected2',num2str(num_cars))) = best_model2;
                    store.(strcat('model_iter_',num2str(model_iter),'est_dist2',num2str(num_cars))) = est_dist2;
                    
                    
                    model_sum_wts_unnormalised(model_num, :) = model_wts;

                    
                    
                    
                    
                
                else
                    %LAST CASE ----%
                %((num_cars == prev_model+1) && (num_cars == tmp_max+1))
                    %num_cars == prev_model +1 == tmp_max+1
                    %For simplicity we ignore that prev_model+2 can NOT
                    %happen.. as only max 1 event in 1 occlusion zone
                    % Also num_cars can be tmp_max+1 only .
                    
                    %So this assumes that one car is permannently occluded 
                    %use data association to decide which car and at what
                    %distance event happened

                    %----------------------------
                    
                   
                    init_state_model = repmat(init_state_est,1, prev_model+1);
                    obj_idx_occlude_model = logical(zeros(prev_model+1, len2-(len1-1), prev_model+1));
                    measurements_model =-Inf(prev_model+1, len2-(len1-1), prev_model+1);
                    params.start = zeros(prev_model+1,1);
                    params.end = zeros(prev_model+1,1);
                    params.dim_change = 1;
                    
                    %obj_idx_occlude_model(:, 1, :) = repmat(obj_idx_occlude_est,1,1,prev_model);
                    %measurements_model(:,1,:) = repmat(measurements_model_est,1,1,prev_model); %measurements(:, len1);
                    
                    %find first time all cars occluded
                    occl_idx = find(sum( ~isinf(measurements(:,len1:end)),1 )==0,1,'first' )-1;
                    tmp_obj_occlude = repmat(obj_idx_occlude_est, 1, occl_idx);
                    try
                    tmp_obj_occlude = tmp_obj_occlude & obj_idx_occlude(1:size(obj_idx_occlude_est,1), len1: len1-1+occl_idx);
                    catch
                       error_caught=1;
                       err_total = err_total+1;
                       break; 
                    end
                    
                    
% %                     %test of new code to correct ofr possible------->>
% %                     %inconsitenices in tmp_obj_occlude
% %                      %measurements simultaneosuly
% %                     tmp_measurements = measurements(:, len1:len2);
% %                     
% %                     %look at the next row of tmpmeasurements till
% %                     %occlusion... it should all be -Inf .. otherwise fix.
% %                     if any( ~isinf(tmp_measurements(num_cars,1:occl_idx)) )
% %                        
% %                         %find the corresponding col. numbers and shift them
% %                         %up and continue till u hit a row which has no
% %                         %infinte measurements
% %                         c=1;
% %                         occ_first = find(~isinf(tmp_measurements(num_cars,1:occl_idx)),1,'first');
% %                         ori = tmp_measurements(num_cars,occ_first:occl_idx);
% %                         tmp_measurements(num_cars,occ_first:occl_idx) = -Inf;
% %                         while((c<num_cars))                           
% %                            replaced = tmp_measurements(num_cars-c,occ_first:occl_idx);
% %                            tmp_measurements(num_cars-c,occ_first:occl_idx) = ori;
% %                            c=c+1;
% %                            
% %                            if all(isinf(replaced))
% %                                break;
% %                            else
% %                                ori = replaced;
% %                            end
% %                            
% %                         end
% %                         
% %                        tmp_obj_occlude = ~isinf(tmp_measurements(1:size(tmp_obj_occlude,1),1:occl_idx)); 
% %                     end
% % 
% %                     %----end of code for checking consistency
% %                     
                    
                    
                    
                    obj_idx_occlude_model(1:size(tmp_obj_occlude,1),1:occl_idx,:) = repmat(tmp_obj_occlude,1,1,prev_model+1);
                    
                    tmp_measurements = measurements(:, len1:len2);
                    for tmp_iter =1:occl_idx
                        for tmp_iter2 =1:prev_model
                            tmp_measurements2 = tmp_measurements(tmp_iter2,tmp_iter);
                            if tmp_obj_occlude(tmp_iter2,tmp_iter,1)>0
                                measurements_model(tmp_iter2,tmp_iter,:) = repmat(tmp_measurements2,1,1,prev_model+1);
                            end
                        end
                    end
                    
                    
                    
                    tmp_measurements_hold = -Inf(tmp_max_hold(model_iter), len2-occl_idx-(len1-1));
                    for tmp_iter = (len1-1)+occl_idx+1:len2
                        for tmp_iter2 = 1: tmp_max_hold(model_iter)
                            tmp_measurements2 = tmp_measurements(tmp_iter2,tmp_iter-(len1-1));
                            if ~isinf(tmp_measurements2)
                                tmp_measurements_hold(tmp_iter2,tmp_iter-(len1-1)-occl_idx)  = tmp_measurements2;
                            end
                        end
                    end
                        
                     
                    
                     
                     for iter = 1: num_cars
                        %assume each car is new in turn for each model of
                        %data association
                            %obj_idx_occlude_model(:, occl_idx+1:end, iter) = ~isinf(tmp_measurements_hold);
                            %measurements_model(:, occl_idx+1:end, iter) =  tmp_measurements_hold;
                            
                        if iter ==1
                            obj_idx_occlude_model(:, occl_idx+1:end, iter) = [logical(zeros(1,len2-occl_idx-(len1-1) )); ~isinf(tmp_measurements_hold) ];                     
                            
                            measurements_model(:, occl_idx+1:end, iter) = [ -Inf(1,len2-occl_idx-(len1-1)); tmp_measurements_hold];
                           
                            
                            
                            params.start(iter) =2;
                            
                            %first time when u see all the measurements (after
                            %occlusion) thatis the max. time till when evnt can
                            %take place
                            params.end(iter) = find(sum(obj_idx_occlude_model(:, occl_idx+1:end, iter),1)>0,1,'first')-1+occl_idx;
                            %params.end(2:end) = params.end(1);
                            
                        elseif iter == num_cars
                            obj_idx_occlude_model(:, occl_idx+1:end, iter) = [~isinf(tmp_measurements_hold); logical(zeros(1,len2-occl_idx-(len1-1) ))];                     
                            
                            measurements_model(:, occl_idx+1:end, iter) = [tmp_measurements_hold; -Inf(1,len2-occl_idx-(len1-1))];
                            
                            params.start(iter) = max(params.start(iter-1),find(obj_idx_occlude_model(iter, :, iter)==0, 1,'first'));
                            params.end(iter) = find(sum(obj_idx_occlude_model(:, occl_idx+1:end, iter),1)==tmp_max_hold(model_iter),1,'first')-1+occl_idx;
                            
                        else
                           tmp_idx = find(obj_idx_occlude_est>0,iter,'first');
                           tmp_idx1 = tmp_idx(end-1);
                           tmp_idx2 = tmp_idx(end);
                           
                                                     
                           obj_idx_occlude_model(:, occl_idx+1:end, iter) = [~isinf(tmp_measurements_hold(1:tmp_idx1,:) ); logical(zeros(1,len2-occl_idx-(len1-1) )); ~isinf(tmp_measurements_hold(tmp_idx2:end,:) )];  
                           
                           measurements_model(:, occl_idx+1:end, iter) = [tmp_measurements_hold(1:tmp_idx1,:); -Inf(1,len2-occl_idx-(len1-1)); tmp_measurements_hold(tmp_idx2:end, :)];
                           try
                           params.start(iter) = find(obj_idx_occlude_model(iter, :, iter)==0, 1,'first');
                           params.end(iter) = find(sum(obj_idx_occlude_model(:, occl_idx+1:end, iter),1)>=iter,1,'first')-1+occl_idx;
                           catch
                              pause; 
                           end
                        end
                           
    
                      end
                    
  
                                    
                    
                    [model_selected, d, model_wts, prob_d, posterior_mean, est_dist, best_model2, est_dist2] = select_data_association_with_dist(init_state_model, obj_idx_occlude_model, measurements_model, temporal_params, measurement_params, num_particles, global_vars, states, params, model_iter-1);
                    
                                        
                    % prepare model parameters for seelcted data association
                    init_state_model = init_state_model(:, model_selected);
                    obj_idx_occlude_model = obj_idx_occlude_model(:, :, model_selected);
                    measurements_model = measurements_model(:, :, model_selected);
  
                    %Store these numbers if required in future
                    store.(strcat('model_iter_',num2str(model_iter),'init_state_model',num2str(num_cars))) = init_state_model;
                    store.(strcat('model_iter_',num2str(model_iter),'obj_idx_occlude_model',num2str(num_cars))) = obj_idx_occlude_model;
                    store.(strcat('model_iter_',num2str(model_iter),'measurements_model',num2str(num_cars))) = measurements_model;
                    store.(strcat('model_iter_',num2str(model_iter),'dist',num2str(num_cars))) = d;
                    store.(strcat('model_iter_',num2str(model_iter),'car_affected',num2str(num_cars))) =model_selected;
                    store.(strcat('model_iter_',num2str(model_iter),'prob_d',num2str(num_cars))) =prob_d;
                    store.(strcat('model_iter_',num2str(model_iter),'last_est',num2str(num_cars))) = posterior_mean;
                    store.(strcat('model_iter_',num2str(model_iter),'est_dist',num2str(num_cars))) = est_dist;
                    store.(strcat('model_iter_',num2str(model_iter),'car_affected2',num2str(num_cars))) = best_model2;
                    store.(strcat('model_iter_',num2str(model_iter),'est_dist2',num2str(num_cars))) = est_dist2;
                    
                    
                    model_sum_wts_unnormalised(model_num, :) = model_wts;

                    
                    
                    %------------------------------
                    
                    
%                 else 
%                     %num_cars == prev_model+2 and == tmp_max+1
%                     
                    
                end
            
            if sum(model_sum_wts_unnormalised(model_num, :))==0
                [posterior_mean, ~, ~, ~, ~, ~, ~, ~, ~, sum_wts_unnormalised] = oneD_Particle_filter_occlusion_v1_vectorised(init_state_model, obj_idx_occlude_model, measurements_model, temporal_params, measurement_params, num_particles, global_vars, states);  
                model_sum_wts_unnormalised(model_num, :) = sum_wts_unnormalised;
                store.(strcat('model_iter_',num2str(model_iter),'last_est',num2str(num_cars))) = posterior_mean(:,end);

            end
            
            
             
          end
            
          if error_caught ==1
              error_caught=0;
              break;
          end
          
         %Now do model selection========== if at least 2 competing models
         %are there
         
     if numel(models_to_consider)>1
            len = size(model_sum_wts_unnormalised,2);
         % Calculate Bayes factors and decide best model (of num. of cars)
         model_sum_wts_unnormalised(model_sum_wts_unnormalised==-1)=1;
         log_Bayes_fac = zeros(numel(models_to_consider)-1, len);
          for model_num =1 : numel(models_to_consider)-1

              log_Bayes_fac(model_num,:) = cumsum(log(model_sum_wts_unnormalised(model_num,:)) - log(model_sum_wts_unnormalised(model_num+1,:)));

          end

          %calcukate relative probs of the various models (taking care of
          %underflow/overflow)
          log_Bayes_fac(log_Bayes_fac>100)=100;

          tmp1 = flipud(cumprod(flipud(exp(log_Bayes_fac)),1));
          tmp2 = sum(tmp1,1)+1;

          tmp3 = 1./ tmp2;

          relative_probs = repmat(tmp3,numel(models_to_consider),1) .* [tmp1; ones(1, len)];
          tmp_convergence = double([relative_probs>0.99]);
          thresh = 3;

          %if (len<50) %|| (size(relative_probs,1)<5) %otherwise takes too long to run
          %  max_idx = seq_convergence_idx(tmp_convergence, thresh);
          %else
            max_idx = len;
          %end
            
          best_model.num_cars(model_iter) = models_to_consider(relative_probs(:,max_idx) == max(relative_probs(:, max_idx)));

          model_id = best_model.num_cars(model_iter);
     else
         model_id = models_to_consider(1);
         best_model.num_cars(model_iter) = model_id;
         
     end
          
      if model_iter >1
          best_model.event_type(model_iter) = best_model.num_cars(model_iter) -prev_model;

           if best_model.event_type(model_iter) ~=0
               % num of cars chnaged accoording to model selection in
               % occlusion zone ---give estimate of when this happened
%               
%               %call optimisation routine
%               best_model.dist(model_iter) = ;
                best_model.dist(model_iter) = store.(strcat('model_iter_',num2str(model_iter),'dist',num2str( model_id )));
                best_model.(strcat('event_num_',num2str(model_iter),'_dist_prob')) = store.(strcat('model_iter_',num2str(model_iter),'prob_d',num2str( model_id ))) ;
                best_model.(strcat('event_num_',num2str(model_iter),'_car_affected')) = store.(strcat('model_iter_',num2str(model_iter),'car_affected',num2str( model_id )));
                best_model.(strcat('event_num_',num2str(model_iter),'_est_event_dist')) = store.(strcat('model_iter_',num2str(model_iter),'est_dist',num2str(model_id)));
                best_model.(strcat('event_num_',num2str(model_iter),'_car_affected2')) = store.(strcat('model_iter_',num2str(model_iter),'car_affected2',num2str(model_id)));
                best_model.(strcat('event_num_',num2str(model_iter),'_est_event_dist2')) = store.(strcat('model_iter_',num2str(model_iter),'est_dist2',num2str(model_id)));
           
           end

      end

     
          %set prev_model to best model for next model selection procedure
          prev_model = model_id;

          %Now pass this model through the occlusion zone to get the
          %init_states for the next model selection step -- This assumes we
          %have selected the model correctly at this step .. should be so if
          %have seen enough interactions/observations
          
          % need to do more work to say how long and from where particle
          % filter fill go
          

          init_state_model = store.(strcat('model_iter_',num2str(model_iter),'init_state_model',num2str(model_id)));
          obj_idx_occlude_model = store.(strcat('model_iter_',num2str(model_iter),'obj_idx_occlude_model',num2str(model_id)));
          measurements_model = store.(strcat('model_iter_',num2str(model_iter),'measurements_model',num2str(model_id)));

          
          %check this line below -------------- NO need to run again--store
          %[init_state_est, ~, ~, ~, ~, ~, ~, ~, ~, ~] = oneD_Particle_filter_occlusion_v1_vectorised(init_state_model, obj_idx_occlude_model, measurements_model, temporal_params, measurement_params, num_particles, global_vars, states);  

          %init_state_est = init_state_est(:, end);
          init_state_est = store.(strcat('model_iter_',num2str(model_iter),'last_est',num2str(model_id)));

            
        end


% % %      % Calculate Bayes factors and decide best model (of num. of cars)
% % %      log_Bayes_fac = zeros(numel(models_to_consider)-1, len);
% % %       for model_num =1 : numel(models_to_consider)-1
% % % 
% % %           log_Bayes_fac(model_num,:) = cumsum(log(model_sum_wts_unnormalised(model_num,:)) - log(model_sum_wts_unnormalised(model_num+1,:)));
% % % 
% % %       end
% % % 
% % %       %calcukate relative probs of the various models (taking care of
% % %       %underflow/overflow)
% % %       log_Bayes_fac(log_Bayes_fac>700)=700;
% % % 
% % %       tmp1 = flipud(cumprod(flipud(exp(log_Bayes_fac)),1));
% % %       tmp2 = sum(tmp1,1)+1;
% % % 
% % %       tmp3 = 1./ tmp2;
% % % 
% % %       relative_probs = repmat(tmp3,numel(models_to_consider),1) .* [tmp1; ones(1, trials)];
% % %       
% % %       best_model.num_cars(model_iter) = find(relative_probs(:,end) == max(relative_probs(:, end)));
% % %       
% % %       if model_iter >1
% % %           best_model.event_type(model_iter) = best_model.num_cars -prev_model;
% % %           
% % %            if best_model.event_type ~=0
% % %                % num of cars chnaged accoording to model selection in
% % %                % occlusion zone ---give estimate of when this happened
% % % %               
% % % %               %call optimisation routine
% % % %               best_model.dist(model_iter) = ;
% % %           end
% % %           
% % %       end
% % %       
% % %       %set prev_model to best model for next model selection procedure
% % %       prev_model = best_model.num_cars(model_iter);
% % %       
% % %       %Now pass this model through the occlusion zone to get the
% % %       %init_states for the next model selection step -- This assumes we
% % %       %have selected the model correctly at this step .. should be so if
% % %       %have seen enough interactions/observations
% % %       
% % %       best_model.num_cars(model_iter)
% % %       
% % %       [init_state_est, ~, ~, ~, ~, ~, ~, ~, ~, ~] = oneD_Particle_filter_occlusion_v1_vectorised(init_state_model, obj_idx_occlude_model, measurements_model, temporal_params, measurement_params, num_particles, global_vars, states);  
% % %     
% % %       init_state_est = init_state_est(:, end);

% 
% 
% 
% 
% 
% 
%     % % % % % % %% Code post all model calculations call
%     % % % % % % 
%     % % % % % %      %tmp = abs(posterior_states_cov(:,:,1:end-1) - posterior_cov_resamp(:,:,:));
%     % % % % % %      %sum(tmp(:))
%     % % % % % %      %tmp = abs(posterior_states_mean(:,1:end-1) - posterior_states_resamp(:,:));
%     % % % % % %      %sum(tmp(:))
%     % % % % % %      
%     % % % % % %      %calculate MSE for only the x-position 
%     % % % % % %      %MSE_particle_filter = MSE_particle_filter + sum(sum((posterior_states(1:3:end,:)- states(1:3:end,:)).^2));
%     % % % % % %      %MSE_particle_filter_timepts = MSE_particle_filter_timepts + sum((posterior_states(1:3:end,:)- states(1:3:end,:)).^2,1);
%     % % % % % % 
%     % % % % % %      %calculate MSE (for full particle state)
%     % % % % % %      tmp = posterior_states_mean - states;
%     % % % % % %      %MSE_mean_filter_full_state_timepts(ii,:) = sum(tmp.^2,1);
%     % % % % % %      %MSE_mean_filter_full_state(ii) = sum(MSE_mean_filter_full_state_timepts(ii,:));
%     % % % % % %      error_est_filter_full_state(:,:, ii) = tmp;
%     % % % % % %      
%     % % % % % %      
%     % % % % % %      tmp_preupdate = posterior_states_mean_preupdate - states;
%     % % % % % %      %error_est_filter_full_state_preupdate(:,:, ii) = tmp_preupdate;
%     % % % % % %      
%     % % % % % %      
%     % % % % % %      for jj = 1: trials
%     % % % % % %          jj;
%     % % % % % %           try
%     % % % % % %          %original calculation of mahal. dist
%     % % % % % %          %fprintf('jj = %d, mse = %3.2f, trace= %3.2f\n',jj,sum(tmp(:,jj).^2), trace(posterior_states_cov(:,:,jj)));
%     % % % % % %          
%     % % % % % %          %fprintf('jj = %d, det = %d, det_pre= %d\n',jj,svds(posterior_states_cov(:,:,jj),1,0), svds(posterior_states_cov_preupdate(:,:,jj),1,0));
%     % % % % % %          
%     % % % % % %          
%     % % % % % %          
%     % % % % % %          
%     % % % % % %          
%     % % % % % %         MSE_mahal_filter_full_state_timepts_car1_x(ii,jj) = tmp(1,jj)' *pinv(posterior_states_cov(1,1,jj),tol) *tmp(1,jj);
%     % % % % % %         MSE_mahal_filter_full_state_timepts_preupdate_car1_x(ii,jj) = tmp(1,jj)' *pinv(posterior_states_cov_preupdate(1,1,jj),tol) *tmp(1,jj);
%     % % % % % % 
%     % % % % % %         MSE_mahal_filter_full_state_timepts_car2_x(ii,jj) = tmp(4,jj)' *pinv(posterior_states_cov(4,4,jj),tol) *tmp(4,jj);
%     % % % % % %         MSE_mahal_filter_full_state_timepts_preupdate_car2_x(ii,jj) = tmp(4,jj)' *pinv(posterior_states_cov_preupdate(4,4,jj),tol) *tmp(4,jj);
%     % % % % % % 
%     % % % % % %         MSE_mahal_filter_full_state_timepts_car3_x(ii,jj) = tmp(7,jj)' *pinv(posterior_states_cov(7,7,jj),tol) *tmp(7,jj);
%     % % % % % %         MSE_mahal_filter_full_state_timepts_preupdate_car3_x(ii,jj) = tmp(7,jj)' *pinv(posterior_states_cov_preupdate(7,7,jj),tol) *tmp(7,jj);
%     % % % % % % 
%     % % % % % %         MSE_mahal_filter_full_state_timepts_car1(ii,jj) = tmp(1:3,jj)' *pinv(posterior_states_cov(1:3,1:3,jj), tol) *tmp(1:3,jj);
%     % % % % % %         MSE_mahal_filter_full_state_timepts_preupdate_car1(ii,jj) = tmp(1:3,jj)' *pinv(posterior_states_cov_preupdate(1:3,1:3,jj), tol) *tmp(1:3,jj);
%     % % % % % % 
%     % % % % % %         MSE_mahal_filter_full_state_timepts_car2(ii,jj) = tmp(4:6,jj)' *pinv(posterior_states_cov(4:6,4:6,jj), tol) *tmp(4:6,jj);
%     % % % % % %         MSE_mahal_filter_full_state_timepts_preupdate_car2(ii,jj) = tmp(4:6,jj)' *pinv(posterior_states_cov_preupdate(4:6,4:6,jj), tol) *tmp(4:6,jj);
%     % % % % % % 
%     % % % % % %         MSE_mahal_filter_full_state_timepts_car3(ii,jj) = tmp(7:9,jj)' *pinv(posterior_states_cov(7:9,7:9,jj), tol) *tmp(7:9,jj);
%     % % % % % %         MSE_mahal_filter_full_state_timepts_preupdate_car3(ii,jj) = tmp(7:9,jj)' *pinv(posterior_states_cov_preupdate(7:9,7:9,jj), tol) *tmp(7:9,jj);
%     % % % % % % 
%     % % % % % %          
%     % % % % % %         MSE_mahal_filter_full_state_timepts(ii,jj) = tmp(:,jj)' *pinv(posterior_states_cov(:,:,jj), tol) *tmp(:,jj);
%     % % % % % %         MSE_mahal_filter_full_state_timepts_preupdate(ii,jj) = tmp(:,jj)' *pinv(posterior_states_cov_preupdate(:,:,jj), tol) *tmp(:,jj);
%     % % % % % % 
%     % % % % % %          
%     % % % % % %   
%     % % % % % %         
%     % % % % % %         
%     % % % % % %         MSE_mahal_filter_full_state_timepts_car1_v(ii,jj) = tmp(2,jj)' *pinv(posterior_states_cov(2,2,jj),tol) *tmp(2,jj);
%     % % % % % %         MSE_mahal_filter_full_state_timepts_preupdate_car1_v(ii,jj) = tmp(2,jj)' *pinv(posterior_states_cov_preupdate(2,2,jj)) *tmp(2,jj);
%     % % % % % % 
%     % % % % % %         MSE_mahal_filter_full_state_timepts_car2_v(ii,jj) = tmp(5,jj)' *pinv(posterior_states_cov(5,5,jj),tol) *tmp(5,jj);
%     % % % % % %         MSE_mahal_filter_full_state_timepts_preupdate_car2_v(ii,jj) = tmp(5,jj)' *pinv(posterior_states_cov_preupdate(5,5,jj),tol) *tmp(5,jj);
%     % % % % % % 
%     % % % % % %         MSE_mahal_filter_full_state_timepts_car3_v(ii,jj) = tmp(8,jj)' *pinv(posterior_states_cov(8,8,jj),tol) *tmp(8,jj);
%     % % % % % %         MSE_mahal_filter_full_state_timepts_preupdate_car3_v(ii,jj) = tmp(8,jj)' *pinv(posterior_states_cov_preupdate(8,8,jj),tol) *tmp(8,jj);
%     % % % % % % 
%     % % % % % %   
%     % % % % % %         
%     % % % % % %         
%     % % % % % %         MSE_mahal_filter_full_state_timepts_car1_a(ii,jj) = tmp(3,jj)' *pinv(posterior_states_cov(3,3,jj),tol) *tmp(3,jj);
%     % % % % % %         MSE_mahal_filter_full_state_timepts_preupdate_car1_a(ii,jj) = tmp(3,jj)' *pinv(posterior_states_cov_preupdate(3,3,jj)) *tmp(3,jj);
%     % % % % % % 
%     % % % % % %         MSE_mahal_filter_full_state_timepts_car2_a(ii,jj) = tmp(6,jj)' *pinv(posterior_states_cov(6,6,jj),tol) *tmp(6,jj);
%     % % % % % %         MSE_mahal_filter_full_state_timepts_preupdate_car2_a(ii,jj) = tmp(6,jj)' *pinv(posterior_states_cov_preupdate(6,6,jj),tol) *tmp(6,jj);
%     % % % % % % 
%     % % % % % %         MSE_mahal_filter_full_state_timepts_car3_a(ii,jj) = tmp(9,jj)' *pinv(posterior_states_cov(9,9,jj),tol) *tmp(9,jj);
%     % % % % % %         MSE_mahal_filter_full_state_timepts_preupdate_car3_a(ii,jj) = tmp(9,jj)' *pinv(posterior_states_cov_preupdate(9,9,jj),tol) *tmp(9,jj);
%     % % % % % % 
%     % % % % % %          %MSE_mahal_filter_full_state_timepts(ii,jj) = tmp(:,jj)' *pinv(posterior_states_cov(:,:,jj), tol) *tmp(:,jj);
%     % % % % % %          %MSE_mahal_filter_full_state_timepts_preupdate(ii,jj) = tmp_preupdate(:,jj)' *pinv(posterior_states_cov_preupdate(:,:,jj), tol) *tmp_preupdate(:,jj);
%     % % % % % %          %MSE_mahal_filter_full_state_timepts_preupdate(ii,jj) = tmp(:,jj)' *pinv(posterior_states_cov_preupdate(:,:,jj), tol) *tmp(:,jj);
%     % % % % % %          
%     % % % % % %          %MSE_mahal_filter_full_state_timepts_resamp(ii,jj) = tmp(:,jj)' *pinv(posterior_cov_resamp(:,:,jj), tol) *tmp(:,jj);
%     % % % % % %          
%     % % % % % %          %fprintf('jj = %d, det = %d, det_pre= %d\n',jj,svds(posterior_states_cov(:,:,jj),1,0), svds(posterior_states_cov_preupdate(:,:,jj),1,0));
%     % % % % % %          
%     % % % % % %          %only the x-coordinates mahal dist
%     % % % % % % %           MSE_mahal_filter_full_state_timepts(ii,jj) = tmp(1:3:end,jj)' *pinv(posterior_states_cov(1:3:end,1:3:end,jj), tol) *tmp(1:3:end,jj);
%     % % % % % % %           MSE_mahal_filter_full_state_timepts_preupdate(ii,jj) = tmp(1:3:end,jj)' *pinv(posterior_states_cov_preupdate(1:3:end,1:3:end,jj), tol) *tmp(1:3:end,jj);
%     % % % % % % % % % % % %          MSE_mahal_filter_full_state_timepts(ii,jj) = tmp(3,jj)' *pinv(posterior_states_cov(3,3,jj), tol) *tmp(3,jj);
%     % % % % % % % % % % % %          MSE_mahal_filter_full_state_timepts_preupdate(ii,jj) = tmp(3,jj)' *pinv(posterior_states_cov_preupdate(3,3,jj), tol) *tmp(3,jj);
%     % % % % % %       
%     % % % % % %          
%     % % % % % %          %fprintf('jj = %d, tot var = %d, tot var = %d\n',jj,rcond(posterior_states_cov(:,:,jj)), rcond(posterior_states_cov_preupdate(:,:,jj)));
%     % % % % % %          %fprintf('jj = %d, min var = %d, min var= %d\n',jj,min(diag(posterior_states_cov(:,:,jj))), min(diag(posterior_states_cov_preupdate(:,:,jj))) );
%     % % % % % %  
%     % % % % % %          
%     % % % % % %          MSE_mahal_filter_full_state_timepts(ii,jj);
%     % % % % % %          MSE_mahal_filter_full_state_timepts_preupdate(ii,jj);
%     % % % % % %          %MSE_mahal_filter_full_state_timepts_resamp(ii,jj);
%     % % % % % %          
%     % % % % % %             
%     % % % % % %             % new calcu for mahalanobis dist.
%     % % % % % %          %tmp1 = particles(:,:,jj) - repmat(posterior_states_mean);
%     % % % % % %          %MSE_mahal_filter_full_state_timepts(ii,jj) = trace(posterior_states_cov(:,:,jj));
%     % % % % % %          
%     % % % % % %          % %          error(lastwarn);
%     % % % % % %           catch
%     % % % % % %               pause;
%     % % % % % % % %              lastwarn('');
%     % % % % % %           end
%     % % % % % %          
%     % % % % % %          MSE_mahal_filter_full_state(ii) = MSE_mahal_filter_full_state(ii) + MSE_mahal_filter_full_state_timepts(ii,jj);    
%     % % % % % %          MSE_mahal_filter_full_state_preupdate(ii) = MSE_mahal_filter_full_state_preupdate(ii) + MSE_mahal_filter_full_state_timepts_preupdate(ii,jj);
%     % % % % % % 
%     % % % % % %          MSE_mahal_filter_full_state_car1(ii) = MSE_mahal_filter_full_state_car1(ii) + MSE_mahal_filter_full_state_timepts_car1(ii,jj);    
%     % % % % % %          MSE_mahal_filter_full_state_preupdate_car1(ii) = MSE_mahal_filter_full_state_preupdate_car1(ii) + MSE_mahal_filter_full_state_timepts_preupdate_car1(ii,jj);
%     % % % % % % 
%     % % % % % %          MSE_mahal_filter_full_state_car2(ii) = MSE_mahal_filter_full_state_car2(ii) + MSE_mahal_filter_full_state_timepts_car2(ii,jj);    
%     % % % % % %          MSE_mahal_filter_full_state_preupdate_car2(ii) = MSE_mahal_filter_full_state_preupdate_car2(ii) + MSE_mahal_filter_full_state_timepts_preupdate_car2(ii,jj);
%     % % % % % % 
%     % % % % % %          MSE_mahal_filter_full_state_car3(ii) = MSE_mahal_filter_full_state_car3(ii) + MSE_mahal_filter_full_state_timepts_car3(ii,jj);    
%     % % % % % %          MSE_mahal_filter_full_state_preupdate_car3(ii) = MSE_mahal_filter_full_state_preupdate_car3(ii) + MSE_mahal_filter_full_state_timepts_preupdate_car3(ii,jj);
%     % % % % % % 
%     % % % % % %          MSE_mahal_filter_full_state_car1_x(ii) = MSE_mahal_filter_full_state_car1_x(ii) + MSE_mahal_filter_full_state_timepts_car1_x(ii,jj);    
%     % % % % % %          MSE_mahal_filter_full_state_preupdate_car1_x(ii) = MSE_mahal_filter_full_state_preupdate_car1_x(ii) + MSE_mahal_filter_full_state_timepts_preupdate_car1_x(ii,jj);
%     % % % % % % 
%     % % % % % %          MSE_mahal_filter_full_state_car2_x(ii) = MSE_mahal_filter_full_state_car2_x(ii) + MSE_mahal_filter_full_state_timepts_car2_x(ii,jj);    
%     % % % % % %          MSE_mahal_filter_full_state_preupdate_car2_x(ii) = MSE_mahal_filter_full_state_preupdate_car2_x(ii) + MSE_mahal_filter_full_state_timepts_preupdate_car2_x(ii,jj);
%     % % % % % % 
%     % % % % % %          MSE_mahal_filter_full_state_car3_x(ii) = MSE_mahal_filter_full_state_car3_x(ii) + MSE_mahal_filter_full_state_timepts_car3_x(ii,jj);    
%     % % % % % %          MSE_mahal_filter_full_state_preupdate_car3_x(ii) = MSE_mahal_filter_full_state_preupdate_car3_x(ii) + MSE_mahal_filter_full_state_timepts_preupdate_car3_x(ii,jj);
%     % % % % % % 
%     % % % % % % 
%     % % % % % %          
%     % % % % % %          MSE_mahal_filter_full_state_car1_v(ii) = MSE_mahal_filter_full_state_car1_v(ii) + MSE_mahal_filter_full_state_timepts_car1_v(ii,jj);    
%     % % % % % %          MSE_mahal_filter_full_state_preupdate_car1_v(ii) = MSE_mahal_filter_full_state_preupdate_car1_v(ii) + MSE_mahal_filter_full_state_timepts_preupdate_car1_v(ii,jj);
%     % % % % % % 
%     % % % % % %          MSE_mahal_filter_full_state_car2_v(ii) = MSE_mahal_filter_full_state_car2_v(ii) + MSE_mahal_filter_full_state_timepts_car2_v(ii,jj);    
%     % % % % % %          MSE_mahal_filter_full_state_preupdate_car2_v(ii) = MSE_mahal_filter_full_state_preupdate_car2_v(ii) + MSE_mahal_filter_full_state_timepts_preupdate_car2_v(ii,jj);
%     % % % % % % 
%     % % % % % %          MSE_mahal_filter_full_state_car3_v(ii) = MSE_mahal_filter_full_state_car3_v(ii) + MSE_mahal_filter_full_state_timepts_car3_v(ii,jj);    
%     % % % % % %          MSE_mahal_filter_full_state_preupdate_car3_v(ii) = MSE_mahal_filter_full_state_preupdate_car3_v(ii) + MSE_mahal_filter_full_state_timepts_preupdate_car3_v(ii,jj);
%     % % % % % % 
%     % % % % % %     
%     % % % % % %          
%     % % % % % %          
%     % % % % % %          MSE_mahal_filter_full_state_car1_a(ii) = MSE_mahal_filter_full_state_car1_a(ii) + MSE_mahal_filter_full_state_timepts_car1_a(ii,jj);    
%     % % % % % %          MSE_mahal_filter_full_state_preupdate_car1_a(ii) = MSE_mahal_filter_full_state_preupdate_car1_a(ii) + MSE_mahal_filter_full_state_timepts_preupdate_car1_a(ii,jj);
%     % % % % % % 
%     % % % % % %          MSE_mahal_filter_full_state_car2_a(ii) = MSE_mahal_filter_full_state_car2_a(ii) + MSE_mahal_filter_full_state_timepts_car2_a(ii,jj);    
%     % % % % % %          MSE_mahal_filter_full_state_preupdate_car2_a(ii) = MSE_mahal_filter_full_state_preupdate_car2_a(ii) + MSE_mahal_filter_full_state_timepts_preupdate_car2_a(ii,jj);
%     % % % % % % 
%     % % % % % %          MSE_mahal_filter_full_state_car3_a(ii) = MSE_mahal_filter_full_state_car3_a(ii) + MSE_mahal_filter_full_state_timepts_car3_a(ii,jj);    
%     % % % % % %          MSE_mahal_filter_full_state_preupdate_car3_a(ii) = MSE_mahal_filter_full_state_preupdate_car3_a(ii) + MSE_mahal_filter_full_state_timepts_preupdate_car3_a(ii,jj);
%     % % % % % % 
%     % % % % % %          
%     % % % % % %      end
%     % % % % % %      %MSE_mahal_filter_full_state(ii) = sum(MSE_mahal_filter_full_state_timepts(ii,:));


         %num_failures = num_failures + num_fail;


       % a = plot_anim_v2(states, posterior_states_mean, measurements, plot_obj_idx_occlude, all_particles, global_vars, eff_sample_size, relative_probs, models_to_consider);


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
    end
    %end of model iteration loop
     %best_model
     
    %distance_of_event
    
    if (best_model.num_cars(1) ~= num_objects)
        overall_car_id__pre_occl_error(run_num) = overall_car_id__pre_occl_error(run_num)+1;
    end
    

    
    if ~isempty(event.type)
        if isfield(best_model,'event_type')
         if (best_model.event_type(2) ~= event.type)
            overall_car_event_type_error(run_num) = overall_car_event_type_error(run_num)+1;
         end
        else
            overall_car_event_type_error(run_num) = overall_car_event_type_error(run_num)+1;
        end
        
    elseif numel(best_model.num_cars)>1
        if (best_model.num_cars(2)~= num_objects)
            overall_car_event_type_error(run_num) = overall_car_event_type_error(run_num)+1;
        end
    end
    
    
    if isfield(best_model, 'event_num_2_car_affected') && isfield(event, 'car_id')
        
        car_prediction1(ii,run_num) = best_model.event_num_2_car_affected;
        car_prediction2(ii, run_num) = best_model.event_num_2_car_affected2;
        
       if (best_model.event_num_2_car_affected ~= event.car_id)
            overall_car_id_est1_error(run_num) = overall_car_id_est1_error(run_num)+1;
            
            if (best_model.event_num_2_car_affected2 ~= event.car_id)
                overall_car_id_est2_error(run_num) = overall_car_id_est2_error(run_num)+1;
            end
       end

    elseif ~isfield(best_model, 'event_num_2_car_affected') && isfield(event, 'car_id')
         overall_car_id_est1_error(run_num) = overall_car_id_est1_error(run_num)+1;
          overall_car_id_est2_error(run_num) = overall_car_id_est2_error(run_num)+1;
         
    elseif isfield(best_model, 'event_num_2_car_affected') && ~isfield(event, 'car_id')
        car_prediction1(ii,run_num) = best_model.event_num_2_car_affected;
        car_prediction2(ii, run_num) = best_model.event_num_2_car_affected2;
        
         overall_car_id_est1_error(run_num) = overall_car_id_est1_error(run_num)+1;
          overall_car_id_est2_error(run_num) = overall_car_id_est2_error(run_num)+1;
    end
    
    
    if isfield(best_model, 'event_num_2_est_event_dist') &&  exist('distance_of_event','var') && ~isempty(distance_of_event)
        overall_dist_error.(strcat('run_num_',num2str(run_num)))(ii) = best_model.event_num_2_est_event_dist -distance_of_event;
    end
    
    %overall_car_event_type_error(2)
    %err_total = err_total+1;
end


%toc;

% % % % % % % % 
% % % % % % % % plot_id =3;
% % % % % % % % aa = zeros(trials,simulations);
% % % % % % % % for ii = 1: trials
% % % % % % % %     for jj = 1: simulations
% % % % % % % %         tmp = all_collect(plot_id, (jj-1)*num_iterations+1: jj*num_iterations,ii);
% % % % % % % %         aa(ii,jj) = var(tmp(:),1);
% % % % % % % %     end
% % % % % % % % end
% % % % % % % % plot(mean(aa,2), '.-');
% % % % % % % % 
% % % % % % % % pause;
% % % % % % 
% % % % % % 
% % % % % % %calcukate MSE per car per simulation per time point
% % % % % % %MSE_particle_filter = MSE_particle_filter/(num_objects * simulations*trials);
% % % % % % 
% % % % % % % %calculate num of tracker failures (average number of failures over trial
% % % % % % % %time points)
% % % % % % % num_failures = num_failures/simulations;
% % % % % % 
% % % % % % %MSE per time point
% % % % % % %MSE_particle_filter_timepts = MSE_particle_filter_timepts / (num_objects * simulations);
% % % % % % 
% % % % % % tmp = zeros(dim, simulations* trials);
% % % % % % for ii = 1 : simulations
% % % % % %     %flatten the 3D array
% % % % % %    tmp(:,(ii-1)*trials+1: ii*trials) = error_est_filter_full_state(:,:,ii);   
% % % % % % end
% % % % % % tmp_mean = mean(tmp,2);
% % % % % % MSE_mean_filter_full_state = sum(sum((tmp - repmat(tmp_mean,1,simulations* trials)).^2))/ (simulations*trials);
% % % % % % %MSE_mean_filter_full_state = sum(MSE_mean_filter_full_state) /(simulations*trials);
% % % % % % MSE_mahal_filter_full_state = sum(MSE_mahal_filter_full_state) /(simulations*trials);
% % % % % % 
% % % % % % MSE_mahal_filter_full_state_preupdate = sum(MSE_mahal_filter_full_state_preupdate) /(simulations*trials);
% % % % % % 
% % % % % % 
% % % % % % 
% % % % % % tmp_timepts = zeros(dim, trials);
% % % % % % 
% % % % % % MSE_mean_filter_full_state_timepts = zeros(1,trials);
% % % % % % for jj = 1 : trials
% % % % % %     %flatten the 3D array
% % % % % %    tmp_timepts(:,jj) = mean(squeeze(error_est_filter_full_state(:,jj,:)),2);
% % % % % %    MSE_mean_filter_full_state_timepts(jj) = sum(sum( (squeeze(error_est_filter_full_state(:,jj,:))- repmat(tmp_timepts(:,jj),1,simulations)).^2));
% % % % % % end
% % % % % % MSE_mean_filter_full_state_timepts = MSE_mean_filter_full_state_timepts/simulations;
% % % % % % 
% % % % % % 
% % % % % % tmp_timepts = zeros(1, trials);
% % % % % % MSE_mean_filter_full_state_timepts = zeros(1,trials);
% % % % % % idx =1;
% % % % % % for jj = 1 : trials
% % % % % %     %flatten the 3D array
% % % % % %    tmp_timepts(:,jj) = mean(squeeze(error_est_filter_full_state(idx,jj,:)));
% % % % % %    MSE_mean_filter_full_state_timepts(jj) = sum(sum( (squeeze(error_est_filter_full_state(idx,jj,:))- repmat(tmp_timepts(:,jj),1,simulations)').^2));
% % % % % % end
% % % % % % MSE_mean_filter_full_state_timepts = MSE_mean_filter_full_state_timepts/simulations;
% % % % % % 
% % % % % % 
% % % % % % 
% % % % % % 
% % % % % % 
% % % % % % MSE_mahal_filter_full_state_timepts1 = sum(MSE_mahal_filter_full_state_timepts,1) /simulations;
% % % % % % %MSE_mahal_filter_full_state_timepts1 = median(MSE_mahal_filter_full_state_timepts,1);
% % % % % % %MSE_mahal_filter_full_state_timepts1 = trimmean(MSE_mahal_filter_full_state_timepts,10,1);
% % % % % % 
% % % % % % 
% % % % % % MSE_mahal_filter_full_state_timepts_preupdate1 = sum(MSE_mahal_filter_full_state_timepts_preupdate,1) /simulations;
% % % % % % %MSE_mahal_filter_full_state_timepts_preupdate1 = median(MSE_mahal_filter_full_state_timepts_preupdate,1) ;
% % % % % % 
% % % % % % %MSE_mahal_filter_full_state_timepts_preupdate1 = trimmean(MSE_mahal_filter_full_state_timepts_preupdate, 40,1);
% % % % % % 
% % % % % % MSE_mahal_filter_full_state_timepts_car1_1 = sum(MSE_mahal_filter_full_state_timepts_car1,1) /simulations;
% % % % % % MSE_mahal_filter_full_state_timepts_preupdate_car1_1 = sum(MSE_mahal_filter_full_state_timepts_preupdate_car1,1) /simulations;
% % % % % % 
% % % % % % MSE_mahal_filter_full_state_timepts_car2_1 = sum(MSE_mahal_filter_full_state_timepts_car2,1) /simulations;
% % % % % % MSE_mahal_filter_full_state_timepts_preupdate_car2_1 = sum(MSE_mahal_filter_full_state_timepts_preupdate_car2,1) /simulations;
% % % % % % 
% % % % % % MSE_mahal_filter_full_state_timepts_car3_1 = sum(MSE_mahal_filter_full_state_timepts_car3,1) /simulations;
% % % % % % MSE_mahal_filter_full_state_timepts_preupdate_car3_1 = sum(MSE_mahal_filter_full_state_timepts_preupdate_car3,1) /simulations;
% % % % % % 
% % % % % % 
% % % % % % MSE_mahal_filter_full_state_timepts_car1_x_1 = sum(MSE_mahal_filter_full_state_timepts_car1_x,1) /simulations;
% % % % % % MSE_mahal_filter_full_state_timepts_preupdate_car1_x_1 = sum(MSE_mahal_filter_full_state_timepts_preupdate_car1_x,1) /simulations;
% % % % % % 
% % % % % % MSE_mahal_filter_full_state_timepts_car2_x_1 = sum(MSE_mahal_filter_full_state_timepts_car2_x,1) /simulations;
% % % % % % MSE_mahal_filter_full_state_timepts_preupdate_car2_x_1 = sum(MSE_mahal_filter_full_state_timepts_preupdate_car2_x,1) /simulations;
% % % % % % 
% % % % % % MSE_mahal_filter_full_state_timepts_car3_x_1 = sum(MSE_mahal_filter_full_state_timepts_car3_x,1) /simulations;
% % % % % % MSE_mahal_filter_full_state_timepts_preupdate_car3_x_1 = sum(MSE_mahal_filter_full_state_timepts_preupdate_car3_x,1) /simulations;
% % % % % % 
% % % % % % 
% % % % % % 
% % % % % % MSE_mahal_filter_full_state_timepts_car1_v_1 = sum(MSE_mahal_filter_full_state_timepts_car1_v,1) /simulations;
% % % % % % MSE_mahal_filter_full_state_timepts_preupdate_car1_v_1 = sum(MSE_mahal_filter_full_state_timepts_preupdate_car1_v,1) /simulations;
% % % % % % 
% % % % % % MSE_mahal_filter_full_state_timepts_car2_v_1 = sum(MSE_mahal_filter_full_state_timepts_car2_v,1) /simulations;
% % % % % % MSE_mahal_filter_full_state_timepts_preupdate_car2_v_1 = sum(MSE_mahal_filter_full_state_timepts_preupdate_car2_v,1) /simulations;
% % % % % % 
% % % % % % MSE_mahal_filter_full_state_timepts_car3_v_1 = sum(MSE_mahal_filter_full_state_timepts_car3_v,1) /simulations;
% % % % % % MSE_mahal_filter_full_state_timepts_preupdate_car3_v_1 = sum(MSE_mahal_filter_full_state_timepts_preupdate_car3_v,1) /simulations;
% % % % % % 
% % % % % % 
% % % % % % MSE_mahal_filter_full_state_timepts_car1_a_1 = sum(MSE_mahal_filter_full_state_timepts_car1_a,1) /simulations;
% % % % % % MSE_mahal_filter_full_state_timepts_preupdate_car1_a_1 = sum(MSE_mahal_filter_full_state_timepts_preupdate_car1_a,1) /simulations;
% % % % % % 
% % % % % % MSE_mahal_filter_full_state_timepts_car2_a_1 = sum(MSE_mahal_filter_full_state_timepts_car2_a,1) /simulations;
% % % % % % MSE_mahal_filter_full_state_timepts_preupdate_car2_a_1 = sum(MSE_mahal_filter_full_state_timepts_preupdate_car2_a,1) /simulations;
% % % % % % 
% % % % % % MSE_mahal_filter_full_state_timepts_car3_a_1 = sum(MSE_mahal_filter_full_state_timepts_car3_a,1) /simulations;
% % % % % % MSE_mahal_filter_full_state_timepts_preupdate_car3_a_1 = sum(MSE_mahal_filter_full_state_timepts_preupdate_car3_a,1) /simulations;
% % % % % % 
% % % % % % %toc;
