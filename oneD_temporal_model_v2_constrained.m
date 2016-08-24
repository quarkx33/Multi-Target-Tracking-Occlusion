%% State evolution model implementation (with constraints)

function [states, distance_of_event] = oneD_temporal_model_v2_constrained(init_state, temporal_params, T, num_objects, global_vars, event)

% simulate x position, velocity and acceleration
counter =0;
%eps = 0.01;
%T is the number of time points simulated
states = -Inf(size(init_state,1),T); % first col. stores intial state
states(:,1) = init_state; % all states initialise at T=0

mean_rev_speed = temporal_params.mean_rev_speed; %0.01;
mean_target_vel = temporal_params.mean_target_vel; %00; %15;
AR_coeff = temporal_params.AR_coeff; % 0.75;


    %event is a struct storing event.type and event.occlusion_zone and
    %event.car_id on which event occurs
    
    %filter is an indicator showing is this a simulation of ground truth or
    %run of the particle filter
    
if isempty(event.type)
   % No events have happend all throughtout simulation
    num_events =0;
    dist_event = Inf; % as if event occurs at infinite distance.. so no event in our simulation range)
    distance_of_event=[];
else
    num_events = numel(event.type);

    %simulate a distance for event for given car id
    dist_event = zeros(numel(event.type),1);
    distance_of_event = zeros(numel(event.type),1);
    
    for k = 1: numel(event.type)
        id = event.occlusion_zone(k);
        tmp =  max(1,(global_vars.occlusion_blocs_cord(id,2) - global_vars.occlusion_blocs_cord(id,1)-global_vars.v_max)/2);
        pd = makedist('Normal', 'mu',0,'sigma',tmp/2);
        t = truncate(pd,-tmp,tmp);
        dist_event(k) = random(t) + tmp + global_vars.occlusion_blocs_cord(id,1);
        %if event.type ==1
%             if event.car_id ==1
        %    dist_event(k) = unifrnd(global_vars.occlusion_blocs_cord(id,1)- 2*global_vars.safe_dist, global_vars.occlusion_blocs_cord(id,2));
%             else
%                 dist_event(k) = unifrnd(global_vars.occlusion_blocs_cord(id,1), global_vars.occlusion_blocs_cord(id,2));
%             end
        %else
        dist_event(k) = unifrnd(global_vars.occlusion_blocs_cord(id,1), global_vars.occlusion_blocs_cord(id,2)-global_vars.v_max);
  
        %end
    end
        



end
    
    
% % %     % figure out the start and end points of each simulation zone.
% % %     
   
event_counter =0;   
 

for ii = 2: T
    
    % simulate first object state
    % here noise represents perturbation to the accelartaion
    %Other assumptions - vel can't be negative --car cant go in reverse
    
    %motion model ==step1 ( x position and velocity)
    states(1:2,ii) = temporal_params.mean +  temporal_params.transition * states(1:3,ii-1);
 
    
    %motion model ==step2 ( acceleration - mena reverting AR process with
    %gaussian noise -- so smooth function, bounded by max and min acceleration)
    states(3,ii) = max(min( mean_rev_speed*(mean_target_vel-states(2,ii)) + AR_coeff* states(3,ii-1) + temporal_params.noise_sigma*randn, global_vars.accn_max), global_vars.accn_min);
    
    %velocity in next step cant be negative and cannot be greater than v_max, so correct acceleration for this
    %states(3,ii) = max(states(3,ii), -states(2,ii));
    states(3,ii) = min(max(states(3,ii), -states(2,ii)/global_vars.delta_t), (global_vars.v_max - states(2,ii))/global_vars.delta_t);
    
        
    for jj = 2: num_objects
        %simulate the other objects in queue (one by one - conditional on
        %the preceding object's position) - similar to Gibb's sampling
    
        %motion model ==step1 ( x position and velocity)
        states((jj-1)*3+1:(jj-1)*3+2,ii) = temporal_params.mean +  temporal_params.transition * states((jj-1)*3+1:3*jj,ii-1);
 
        %Ensure that state values do not lead to cars "crossing over" each other
        %chance of this happening should anyway be small due to the braking
        %effect introduced later
        if states((jj-1)*3+1,ii) > states((jj-2)*3+1,ii)
            prev_pos = states((jj-1)*3+1,ii-1);
            curr_pos_car_ahead = states((jj-2)*3+1,ii);
            states((jj-1)*3+1,ii) =  prev_pos + unifrnd(0, curr_pos_car_ahead - prev_pos);       
        end
    
        %motion model ==step2 ( acceleration - mean reverting AR process with
        %gaussian noise -- so smooth function, bounded by max and min acceleration)
        a1 = max(min( mean_rev_speed*(mean_target_vel-states((jj-1)*3+2,ii)) + AR_coeff* states(3*jj,ii-1) + temporal_params.noise_sigma*randn, global_vars.accn_max), global_vars.accn_min);

        %introduce a braking if car is close to car ahead (i.e. within
        %distance safe_dist) or acceleration is car is "far" ahead
        
        dist_to_car_ahead = states((jj-2)*3+1,ii) - states((jj-1)*3+1,ii);
        
       
          %====new code from here =====
        %weight_accl_increase = max(0,exp(-global_vars.safe_dist/dist_to_car_ahead ) - exp(-1));
        weight_accl_increase = min(1,max(0,exp(1-global_vars.safe_dist/dist_to_car_ahead ) - 1));
        
        states(3*jj,ii) = (1-weight_accl_increase)*a1 + weight_accl_increase*global_vars.accn_max;
        
        
        a2 = global_vars.accn_min; % max. allowabale deccelleration
        
        %final accelaration is a weighted combination of above two
        weight_accl_decrease = max(0, 1- dist_to_car_ahead/global_vars.safe_dist );
        states(3*jj,ii) = (1-weight_accl_decrease)*states(3*jj,ii) + weight_accl_decrease*a2;
 
        
        
        
        
        %a2 = global_vars.accn_min; % max. allowabale deccelleration
        
% % % %         %final accelaration is a weighted combination of above two
% % % %         weight = max(0, 1- dist_to_car_ahead/global_vars.safe_dist );
% % % %         states(3*jj,ii) = (1-weight)*a1 + weight*a2;
% % % %         
% % % %         
        %velocity in next step cant be negative and cannot be greater than v_max, so correct acceleration for this
        %Also position in next step shouldn't be greater than vehicle ahead
        
%        states(3*jj,ii) = min(max(states(3*jj,ii), -states((jj-1)*3+2,ii)), global_vars.v_max - states((jj-1)*3+2,ii));
        
        %check if a valid acc. is posisble satisfying the bounds...
        lb1 = global_vars.accn_min;
        lb2 = -states((jj-1)*3+2,ii)/global_vars.delta_t;
        
        ub1 = global_vars.accn_max;
        ub2 = (global_vars.v_max - states((jj-1)*3+2,ii))/global_vars.delta_t;
        ub3 = (2/(global_vars.delta_t^2)) *( (states(3*(jj-2)+1,ii) - states(3*(jj-1)+1,ii)) + global_vars.delta_t * (states(3*(jj-2)+2,ii) - states(3*(jj-1)+2,ii)) + 0.5*(global_vars.delta_t^2) *states(3*(jj-2)+3,ii) );
        %ub3 = Inf;
        %it is possible that ub3 < lb1 (less imp as a min can be changed) but also ub3<lb2 (more
        %imp.) If ub3 <lb1 or ub3<lb2 we will not get a valid accle. In this
        %case choose acc. to be the min. possible. In the next step the
        %position if violated will be generated from uniform.
        if (ub3<lb1) || (ub3<lb2)
            states(3*jj,ii) = lb2; %min. acc. possible (so as not to make vel. in next step -ve)
            %fprintf('issue found in ground truth\n');
        else
            states(3*jj,ii) = min( [max([states(3*jj,ii), lb1,lb2]), ub1, ub2, ub3]);
       
        end
        
% % %         %         states(3*jj,ii) = min( [max([states(3*jj,ii), lb1,lb2]), ub1, ub2]);
% % %  
%counter = counter + abs(tmp_states(3*jj,ii) - states(3*jj,ii))

    end
   
    
    % check if we need to generate event (if any)
    if (event_counter< num_events)
        %Still left events to simulate
        % Check if next event is due
        
        if event.car_id(event_counter+1) > num_objects
            last_car_id=1;
        else
            last_car_id=0;
        end

        if ((last_car_id ==0) && (states(3*event.car_id(event_counter+1)-2,ii)> dist_event(event_counter+1))) || ( (last_car_id ==1) && (states(3*event.car_id(event_counter+1)-2-3,ii)> dist_event(event_counter+1)) )
            % event should be simulated now (car appearance or
            % disappearance)
            
            event_counter = event_counter+1;
            car_id = event.car_id(event_counter);
%             if last_car_id ==0
%                 distance_of_event(event_counter) = states(3*event.car_id(event_counter)-2,ii);
%             else
%                 distance_of_event(event_counter) =  states(3*event.car_id(event_counter)-2-3,ii);
%             end
                            
            if event.type(event_counter) ==-1
               % this particular car disappears
                num_objects = num_objects-1;
               
                distance_of_event(event_counter) = states(3*event.car_id(event_counter)-2,ii);
                            
                
                % Adjust the states vector accordingly

                states_tmp = states(:,ii);
                states(1:3*(car_id-1),ii) = states_tmp(1:3*(car_id-1));
                states(3*(car_id-1)+1:end-3,ii) = states_tmp(3*car_id+1:3*(num_objects+1));
                
                states(end-2:end, ii) = -Inf; % negative indicates this state dimension is no longer used
                    
                
            else
                % a new car appears in front of this car at a "max." of safe
                % distance away but within the end of the occlusion zone
                
                %first figure out if the bew object appears after the last
                %car
             
                num_objects = num_objects+1;
               
                
                
                
                % Adjust the states vector accordingly
                states_tmp = states(:,1:ii);
                if size(states,1)< 3*num_objects
                   %create space
                   states = -Inf(3*num_objects,T);
                   states(1: 3*(num_objects-1),1:ii-1) = states_tmp(:,1:ii-1);
                   states(end-2:end,1:ii-1)=-Inf;
                   
                   states(1:3*(car_id-1),ii) = states_tmp(1:3*(car_id-1),ii);
                   
                   if last_car_id ==0
                       
                       d1 = states_tmp(3*(car_id-1)+1,ii);
                       if car_id ==1
                           %new car comes out before 1st car
                            d2 = Inf;
                       else
                            d2 = states_tmp(3*(car_id-2)+1,ii);
                       end

                       %find the end of occlusion block in which event happens
                       tmp = find(global_vars.occlusion_blocs_cord(:,1)< dist_event(event_counter),1,'last');
                       d3 = global_vars.occlusion_blocs_cord(tmp,2);

                        %restrict new car coming out not too far ahead ..
                        %otherwise at no point in time all cars may be
                        %occluded .. also new car coming too far ahead ..
                        %will come out much earlier than the group .. so
                        %easy and no interaction.
                        if (car_id ==1)&& ((d3-d1)>global_vars.safe_dist)
                            d3= d1+ global_vars.safe_dist;
                        end
                        
                       states(3*(car_id-1)+1,ii) = unifrnd(d1, min(d2, d3));
                       distance_of_event(event_counter) = states(3*(car_id-1)+1,ii);
%                        if isnan(states(3*(car_id-1)+1,ii))
%                            pause; 
%                        end
                       states(3*(car_id-1)+2,ii) = temporal_params.mean_target_vel + 0.1*randn;
                       
                        lb1 = global_vars.accn_min;
                        lb2 = -states(3*(car_id-1)+2,ii)/global_vars.delta_t;

                        ub1 = global_vars.accn_max;
                        ub2 = (global_vars.v_max - states(3*(car_id-1)+2,ii))/global_vars.delta_t;
                        
                        if car_id ==1
                            %first car 
                            states(3*(car_id-1)+3,ii) = unifrnd(max(lb1,lb2), min(ub1, ub2));
                        else
                            ub3 = (2/(global_vars.delta_t^2))*( (states(3*(car_id-2)+1,ii) - states(3*(car_id-1)+1,ii)) + global_vars.delta_t * (states(3*(car_id-2)+2,ii) - states(3*(car_id-1)+2,ii)) + 0.5*(global_vars.delta_t^2) * states(3*(car_id-2)+3,ii) );
        
                            if (ub3<lb1) || (ub3<lb2)
                                states(3*(car_id-1)+3,ii) = lb2; %min. acc. possible (so as not to make vel. in next step -ve)

                            else
                                states(3*(car_id-1)+3,ii) = unifrnd(max(lb1,lb2), min([ub1, ub2, ub3]));

                            end
                        
                        end
                        
                        
                       
                       
%                        states(3*(car_id-1)+3,ii) = min(states(3*(car_id-1)+3,ii), global_vars.v_max - states(3*(car_id-1)+2,ii));


                       states(3*car_id+1:end,ii) = states_tmp(3*(car_id-1)+1:3*(num_objects-1),ii);
                   
                   else
                       % a car has been added at the end
                                             
                       d1 = states_tmp(3*(car_id-1)+1-3,ii);
                       
                       %find the beginning of occlusion block in which event happens
                       tmp = find(global_vars.occlusion_blocs_cord(:,1)< dist_event(event_counter),1,'last');
                       d2 = max(global_vars.occlusion_blocs_cord(tmp,1), d1 - global_vars.safe_dist);

                       
                       states(end-2,ii) = unifrnd(d2, d1);
                       distance_of_event(event_counter) = states(end-2,ii);
                       states(end-1,ii) = temporal_params.mean_target_vel + 0.1*randn;
                       
                        lb1 = global_vars.accn_min;
                        lb2 = -states(end-1,ii)/global_vars.delta_t;

                        ub1 = global_vars.accn_max;
                        ub2 = (global_vars.v_max - states(end-1,ii))/global_vars.delta_t;
                        ub3 = (2/(global_vars.delta_t^2))*( (states(3*(car_id-2)+1,ii) - states(end-2,ii)) +global_vars.delta_t * (states(3*(car_id-2)+2,ii) - states(end-1,ii)) + 0.5*(global_vars.delta_t^2) * states(3*(car_id-2)+3,ii) );
        
                        if (ub3<lb1) || (ub3<lb2)
                            states(end,ii) = lb2; %min. acc. possible (so as not to make vel. in next step -ve)

                        else
                            states(end,ii) = unifrnd(max(lb1,lb2), min([ub1, ub2, ub3]));

                        end
                        

                       
                   end
                       
                   
                   
                else
                    states_tmp = states(:,ii);
                    states(1:3*(car_id-1),ii) = states_tmp(1:3*(car_id-1));
                    
                    if last_car_id ==0
                    
                        d1 = states_tmp(3*(car_id-1)+1);
                        if car_id ==1
                           %new car comes out before 1st car
                            d2 = Inf;
                        else
                            d2 = states_tmp(3*(car_id-2)+1);
                        end

                        %find the end of occlusion block in which event happens
                        tmp = find(global_vars.occlusion_blocs_cord(:,1)< dist_event(event_counter),1,'last');
                        d3 = global_vars.occlusion_blocs_cord(tmp,2);

                        %restrict new car coming out not too far ahead ..
                        %otherwise at no pint in time all cars may be
                        %occluded
                        if (car_id ==1)&& ((d3-d1)>global_vars.safe_dist)
                            d3= d1+ global_vars.safe_dist;
                        end
                        
                        states(3*(car_id-1)+1,ii) = unifrnd(d1, min(d2, d3));
                        distance_of_event(event_counter) = states(3*(car_id-1)+1,ii);
%                         if isnan(states(3*(car_id-1)+1,ii))
%                            pause; 
%                         end
                        states(3*(car_id-1)+2,ii) = temporal_params.mean_target_vel + 0.1*randn;
                        
                        lb1 = global_vars.accn_min;
                        lb2 = -states(3*(car_id-1)+2,ii)/global_vars.delta_t;

                        ub1 = global_vars.accn_max;
                        ub2 = (global_vars.v_max - states(3*(car_id-1)+2,ii))/global_vars.delta_t;
                        
                        if car_id ==1
                            %first car 
                            states(3*(car_id-1)+3,ii) = unifrnd(max(lb1,lb2), min(ub1, ub2));
                        else
                            ub3 = (2/(global_vars.delta_t^2))*( (states(3*(car_id-2)+1,ii) - states(3*(car_id-1)+1,ii)) +global_vars.delta_t * (states(3*(car_id-2)+2,ii) - states(3*(car_id-1)+2,ii)) + 0.5*(global_vars.delta_t^2) * states(3*(car_id-2)+3,ii) );

                            if (ub3<lb1) || (ub3<lb2)
                                states(3*(car_id-1)+3,ii) = lb2; %min. acc. possible (so as not to make vel. in next step -ve)

                            else
                                states(3*(car_id-1)+3,ii) = unifrnd(max(lb1,lb2), min([ub1, ub2, ub3]));

                            end
                        end
                        

                        states(3*car_id+1:end, ii ) = states_tmp(3*(car_id-1)+1:3*(num_objects-1)); 

                    else
                        
                        d1 = states_tmp(3*(car_id-1)+1-3);
                       
                       %find the beginning of occlusion block in which event happens
                       tmp = find(global_vars.occlusion_blocs_cord(:,1)< dist_event(event_counter),1,'last');
                       d2 = max(global_vars.occlusion_blocs_cord(tmp,1), d1 - global_vars.safe_dist);
                       
                       % a car has been added at the end
                       states(end-2,ii) = unifrnd(d2, d1);
                       distance_of_event(event_counter) = states(end-2,ii);
                       states(end-1,ii) = temporal_params.mean_target_vel + 0.1*randn;
                       
                        lb1 = global_vars.accn_min;
                        lb2 = -states(end-1,ii)/global_vars.delta_t;

                        ub1 = global_vars.accn_max;
                        ub2 = (global_vars.v_max - states(3*(car_id-1)+2,ii))/global_vars.delta_t;
                        ub3 = (2/(global_vars.delta_t^2))*( (states(3*(car_id-2)+1,ii) - states(end-2,ii)) +global_vars.delta_t * (states(3*(car_id-2)+2,ii) - states(end-2,ii)) + 0.5*(global_vars.delta_t^2) * states(3*(car_id-2)+3,ii) );
        
                        if (ub3<lb1) || (ub3<lb2)
                            states(end,ii) = lb2; %min. acc. possible (so as not to make vel. in next step -ve)

                        else
                            states(end,ii) = unifrnd(max(lb1,lb2), min([ub1, ub2, ub3]));

                        end
                        
                        
                       
                    end
                    
                   
                end
                
 
                
            end       
        end
    end
    
    

    
    

end

if any(isnan(states(:)))
   pause; 
end



