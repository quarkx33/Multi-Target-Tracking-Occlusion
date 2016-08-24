%% State evolution model implementation (with constraints)

function states_final = oneD_temporal_model_v2_constrained_vectorised(init_state, temporal_params, num_objects, global_vars)

% simulate x position, velocity and acceleration
%dim = size(init_state,1)/num_objects;
num_particles = size(init_state,2);

counter =0;
%T is the number of time points simulated
%states = zeros(size(init_state,1),T,num_particles ); % first col. stores intial state
%states(:,1) = init_state; % all states initialise at T=0

%apply vectorised calculation (multiple particles passsed)
%states_unrolled = init_state(:);
states_final = zeros(size(init_state));

mean_rev_speed = temporal_params.mean_rev_speed; %0.01;
mean_target_vel = temporal_params.mean_target_vel; %00; %15;
AR_coeff = temporal_params.AR_coeff; % 0.75;

% % % for ii = 1: num_particles
% % %     
% % %     % simulate first object state
% % %     % here noise represents perturbation to the accelartaion
% % %     %Other assumptions - vel can't be negative --car cant go in reverse
% % %     
% % %     %motion model ==step1 ( x position and velocity)
% % %  
% % %     
% % %     states_final(1:2,ii) = temporal_params.mean +  temporal_params.transition * init_state(1:3,ii);
% % % 
% % %     
% % % end



%     tmp_idx = repmat([ones(3,1);zeros(3*(num_objects-1),1)],1,num_particles);
%     tmp = temporal_params.mean +  transition_mat_full * states_unrolled(logical(tmp_idx(:)));
%     states_final(1:2, :) = reshape(tmp,2,num_particles);


    
    

    
        
        states_final(1:2,:) = temporal_params.mean +  temporal_params.transition * init_state(1:3,:);


        %motion model ==step2 ( acceleration - mena reverting AR process with
        %gaussian noise -- so smooth function, bounded by max and min acceleration)
        %states(3,ii) = max(min( mean_rev_speed*(mean_target_vel-states(2,ii)) + AR_coeff* states(3,ii-1) + temporal_params.noise_sigma*randn, global_vars.accn_max), global_vars.accn_min);
        states_final(3,:) = max(min( mean_rev_speed*(mean_target_vel-states_final(2,:)) + AR_coeff* init_state(3,:) + normrnd(0, temporal_params.noise_sigma, 1, num_particles) , global_vars.accn_max), global_vars.accn_min);


        %velocity in next step cant be negative and cannot be greater than v_max, so correct acceleration for this
        %states(3,ii) = min(max(states(3,ii), -states(2,ii)), global_vars.v_max - states(2,ii));  
        states_final(3,:) = min(max(states_final(3,:), -states_final(2,:)/global_vars.delta_t), (global_vars.v_max - states_final(2,:))/global_vars.delta_t);


        for jj = 2: num_objects
            %simulate the other objects in queue (one by one - conditional on
            %the preceding object's position) - similar to Gibb's sampling

            %motion model ==step1 ( x position and velocity)
            %states((jj-1)*3+1:(jj-1)*3+2,ii) = temporal_params.mean +  temporal_params.transition * states((jj-1)*3+1:3*jj,ii-1);
            states_final((jj-1)*3+1:(jj-1)*3+2,:) = temporal_params.mean +  temporal_params.transition * init_state((jj-1)*3+1:3*jj,:);


            %Ensure that state values do not lead to cars "crossing over" each other
            %chance of this happening should anyway be small due to the braking
            %effect introduced later
            tmp = states_final((jj-1)*3+1,:) > states_final((jj-2)*3+1,:);

            if any(tmp)
                prev_pos = init_state((jj-1)*3+1,:);
                curr_pos_car_ahead = states_final((jj-2)*3+1,:);
                correctedstates = prev_pos + unifrnd(0, curr_pos_car_ahead - prev_pos);
                states_final((jj-1)*3+1,tmp) =  correctedstates(tmp);       
            end

            %motion model ==step2 ( acceleration - mean reverting AR process with
            %gaussian noise -- so smooth function, bounded by max and min acceleration)
            a1 = max(min( mean_rev_speed*(mean_target_vel-states_final((jj-1)*3+2,:)) + AR_coeff* init_state(3*jj,:) + normrnd(0, temporal_params.noise_sigma, 1, num_particles), global_vars.accn_max), global_vars.accn_min);

            %introduce a braking if car is close to car ahead (i.e. within
            %distance safe_dist). However if car ahead is far ahead then
            %intoduce an additional acceleration (driver behind) tries to close
            %the "large" gap
            dist_to_car_ahead = states_final((jj-2)*3+1,:) - states_final((jj-1)*3+1,:);

            %====new code from here =====
            %weight_accl_increase = max(0,exp(-global_vars.safe_dist./dist_to_car_ahead ) - exp(-1));
            weight_accl_increase  = min(1,max(0,exp(1-global_vars.safe_dist./dist_to_car_ahead ) - 1));
            states_final(3*jj,:) = (1-weight_accl_increase).*a1 + weight_accl_increase*global_vars.accn_max;


            a2 = global_vars.accn_min; % max. allowabale deccelleration

            %final accelaration is a weighted combination of above two
            weight_accl_decrease = max(0, 1- dist_to_car_ahead/global_vars.safe_dist );
            states_final(3*jj,:) = (1-weight_accl_decrease).*states_final(3*jj,:) + weight_accl_decrease*a2;







            %==== end of modified code =============


            %velocity in next step cant be negative and cannot be greater than v_max, so correct acceleration for this
            
            %tmp_final(3*jj,:) = min(max(states_final(3*jj,:), -states_final((jj-1)*3+2,:)), global_vars.v_max - states_final((jj-1)*3+2,:));

            
            
            %check if a valid acc. is posisble satisfying the bounds...
            lb1 = global_vars.accn_min;
            lb2 = -states_final((jj-1)*3+2,:)/global_vars.delta_t;
            lb_final = max(lb1, lb2);

            ub1 = global_vars.accn_max;
            ub2 = (global_vars.v_max - states_final((jj-1)*3+2,:))/global_vars.delta_t;
            ub3 = (2/(global_vars.delta_t^2))* ( (states_final(3*(jj-2)+1,:) - states_final(3*(jj-1)+1,:)) +global_vars.delta_t * (states_final(3*(jj-2)+2,:) - states_final(3*(jj-1)+2,:)) + 0.5*(global_vars.delta_t^2) *states_final(3*(jj-2)+3,:) );
            %ub3 = Inf(size(ub3));
            ub_final = min(min(ub1,ub2), ub3);
            %it is possible that ub3 < lb1 (less imp as a min can be changed) but also ub3<lb2 (more
            %imp.) If ub3 <lb1 or ub3<lb2 we will not get a valid accle. In this
            %case choose acc. to be the min. possible. In the next step the
            %position if violated will be generated from uniform.
            k = (ub3<lb1) | (ub3<lb2);
%             if any(k)
%                fprintf('issue found in particles\n'); 
%             end
%             counter= counter + sum(k);
            
            states_final(3*jj,k) = lb2(k); %min. acc. possible (so as not to make vel. in next step -ve)
            states_final(3*jj,~k) = min( max(states_final(3*jj,~k), lb_final(~k)), ub_final(~k)); %min. acc. possible (so as not to make vel. in next step -ve)
            
%             if (ub3<lb1) || (ub3<lb2)
%                 states(3*jj,ii) = lb2; %min. acc. possible (so as not to make vel. in next step -ve)
% 
%             else
%                 states(3*jj,ii) = min( [max([states(3*jj,ii), lb1,lb2]), ub1, ub2, ub3]);
% 
%             end

        %sum(tmp_final(3*jj,:) - states_final(3*jj,:))
            
            


        end


% if counter >0
% counter/(num_objects* num_particles)
% end    
%         


    
    
end






