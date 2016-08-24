%% State evolution model implementation (with constraints)

function states_final = oneD_temporal_model_v2_no_interaction_vectorised(init_state, temporal_params,num_objects, global_vars)

% simulate x position, velocity and acceleration

%T is the number of time points simulated
% % states = zeros(size(init_state,1),T); % first col. stores intial state
% % states(:,1) = init_state; % all states initialise at T=0

num_particles = size(init_state,2);
states_final = zeros(size(init_state));

mean_rev_speed = temporal_params.mean_rev_speed; %0.01;
mean_target_vel = temporal_params.mean_target_vel; %00; %15;
AR_coeff = temporal_params.AR_coeff; % 0.75;


for jj =1:num_objects
  states_final((jj-1)*3+1:(jj-1)*3+2,:) = temporal_params.mean +  temporal_params.transition * init_state((jj-1)*3+1:3*jj,:);
 
end
    
states_final(3:3:end,:) = max(min( mean_rev_speed*(mean_target_vel-states_final(2:3:end,:)) + AR_coeff* init_state(3:3:end,:) + normrnd(0, temporal_params.noise_sigma, num_objects, num_particles) , global_vars.accn_max), global_vars.accn_min);
    
states_final(3:3:end,:) = min(max(states_final(3:3:end,:), -states_final(2:3:end,:)), global_vars.v_max - states_final(2:3:end,:));
    
     
    
% % %     
% % % for ii = 2: T
% % %     
% % %     %remove all interactions and simulate all objects----
% % %     
% % %     for jj = 1: num_objects
% % %         % simulate first object state
% % %         % here noise represents perturbation to the accelartaion
% % %         %Other assumptions - vel can't be negative --car cant go in reverse
% % % 
% % %         %motion model ==step1 ( x position and velocity)
% % %         states((jj-1)*3+1:(jj-1)*3+2,ii) = temporal_params.mean +  temporal_params.transition * states((jj-1)*3+1:jj*3,ii-1);
% % % 
% % % 
% % %         %motion model ==step2 ( acceleration - mena reverting AR process with
% % %         %gaussian noise -- so smooth function, bounded by max and min acceleration)
% % %         states(3*jj,ii) = max(min( mean_rev_speed*(mean_target_vel-states((jj-1)*3+2,ii)) + AR_coeff* states(3*jj,ii-1) + temporal_params.noise_sigma*randn, global_vars.accn_max), global_vars.accn_min);
% % % 
% % %         %velocity in next step cant be negative and cannot be greater than v_max, so correct acceleration for this
% % %         %states(3,ii) = max(states(3,ii), -states(2,ii));
% % %         states(3*jj,ii) = min(max(states(3*jj,ii), -states((jj-1)*3+2,ii)), global_vars.v_max - states((jj-1)*3+2,ii));
% % % 
% % %     end
% % %     
% % % % % %     for jj = 2: num_objects
% % % % % %         %simulate the other objects in queue (one by one - conditional on
% % % % % %         %the preceding object's position) - similar to Gibb's sampling
% % % % % %     
% % % % % %         %motion model ==step1 ( x position and velocity)
% % % % % %         states((jj-1)*3+1:(jj-1)*3+2,ii) = temporal_params.mean +  temporal_params.transition * states((jj-1)*3+1:3*jj,ii-1);
% % % % % %  
% % % % % %         %Ensure that state values do not lead to cars "crossing over" each other
% % % % % %         %chance of this happening should anyway be small due to the braking
% % % % % %         %effect introduced later
% % % % % %         if states((jj-1)*3+1,ii) >= states((jj-2)*3+1,ii)
% % % % % %             prev_pos = states((jj-1)*3+1,ii-1);
% % % % % %             curr_pos_car_ahead = states((jj-2)*3+1,ii);
% % % % % %             states((jj-1)*3+1,ii) =  prev_pos + unifrnd(0, curr_pos_car_ahead - prev_pos);       
% % % % % %         end
% % % % % %     
% % % % % %         %motion model ==step2 ( acceleration - mean reverting AR process with
% % % % % %         %gaussian noise -- so smooth function, bounded by max and min acceleration)
% % % % % %         a1 = max(min( mean_rev_speed*(mean_target_vel-states((jj-1)*3+2,ii)) + AR_coeff* states(3*jj,ii-1) + temporal_params.noise_sigma*randn, global_vars.accn_max), global_vars.accn_min);
% % % % % % 
% % % % % %         %introduce a braking if car is close to car ahead (i.e. within
% % % % % %         %distance safe_dist)
% % % % % %         dist_to_car_ahead = states((jj-2)*3+1,ii) - states((jj-1)*3+1,ii);
% % % % % %         
% % % % % %         a2 = global_vars.accn_min; % max. allowabale deccelleration
% % % % % %         
% % % % % %         %final accelaration is a weighted combination of above two
% % % % % %         weight = max(0, 1- dist_to_car_ahead/global_vars.safe_dist );
% % % % % %         states(3*jj,ii) = (1-weight)*a1 + weight*a2;
% % % % % %         
% % % % % %         
% % % % % %         %velocity in next step cant be negative and cannot be greater than v_max, so correct acceleration for this
% % % % % %         %states(3*jj,ii) = max(states(3*jj,ii), -states((jj-1)*3+2,ii));
% % % % % %         states(3*jj,ii) = min(max(states(3*jj,ii), -states((jj-1)*3+2,ii)), global_vars.v_max - states((jj-1)*3+2,ii));
% % % % % % 
% % % % % %     
% % % % % %                
% % % % % %     end
% % %    
% % %     
% % % end
% % % 





