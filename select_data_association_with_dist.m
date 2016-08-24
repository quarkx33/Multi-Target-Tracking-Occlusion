function [best_model, dist2, weights, prob_dist, posterior_mean, est_dist1, best_model2, est_dist2] = select_data_association_with_dist(init_state_model, obj_idx_occlude_model, measurements_model, temporal_params, measurement_params, num_particles, global_vars, states, other_params, block_id)

%This is again like another Bayesian model selection
%Here number of cars remain same but data association chnages maintaining
%"order" of vehicles. Also this has the extra complication of finding the
%optimal distance of event (the event occurs in the occlusion zone)

len = size(measurements_model,2);
models_to_consider =size(init_state_model, 2); 
model_sum_wts_unnormalised = zeros(models_to_consider, len);
dist = zeros(models_to_consider,1);
log_lik_hold = -Inf(max(other_params.end - other_params.start +1),models_to_consider);
posterior_mean_hold = zeros( size(init_state_model,1)+3*other_params.dim_change, max(other_params.end - other_params.start +1), models_to_consider);
est_dist = zeros(max(other_params.end - other_params.start +1), models_to_consider);
est_dist_final = zeros(models_to_consider,1);

insert_states = zeros(3,1);
insert_states(2) = temporal_params.mean_target_vel + 0.1*randn; %unifrnd(0, global_vars.v_max) ; %, 1, 1);
%insert_states(3) = unifrnd(-global_vars.accn_min, global_vars.accn_max); %, (num_cars-num_objects), 1);


for model_iter = 1: models_to_consider
    
    if other_params.dim_change == 1
        tmp_idx = logical([ ones(size(obj_idx_occlude_model,1)-1,1);0] );
        
    else
        tmp_idx = logical(ones(size(obj_idx_occlude_model,1),1));
    end
    
    %---run the prev_model from start till a max. time 
    [posterior_states_mean, ~, ~, ~, ~, ~, ~, ~, ~, sum_wts_unnormalised_prev_model, ~, wts_sample] = oneD_Particle_filter_occlusion_v1_vectorised(init_state_model(:, model_iter), obj_idx_occlude_model(tmp_idx,1:other_params.end(model_iter),model_iter), measurements_model(tmp_idx,1:other_params.end(model_iter),model_iter), temporal_params, measurement_params, num_particles, global_vars, states);
    
    t = other_params.start(model_iter);
    
    if other_params.dim_change == -1
        tmp = (sum(obj_idx_occlude_model(:,t:end,model_iter),2)>0);
        
        %figure out which dimension to be reduced as dimesnions can also be
        %occluded throughout previously
        tmp_idx = (double(tmp) == double(obj_idx_occlude_model(:,1,model_iter)));
              
    else
        tmp_idx =logical( ones(size(obj_idx_occlude_model,1), 1) );
    end
    
    

     
    wts_hold = zeros(other_params.end(model_iter) - other_params.start(model_iter) +1,len);
    
    while(t<=other_params.end(model_iter))
        
        % now find the optimal d.
        if other_params.dim_change == -1    
            init_state_tmp = posterior_states_mean(:,t);
            dist_tmp = posterior_states_mean(1:3:end,t);
            try
            est_dist(t - other_params.start(model_iter)+1, model_iter) = dist_tmp(~tmp_idx);
                
                if est_dist(t - other_params.start(model_iter)+1, model_iter)> global_vars.occlusion_blocs_cord(block_id,2)
                        break;
                end
                
            catch
               pause; 
            end
            
            tmp_idx2 = repmat(tmp_idx',3,1);
            init_state_tmp = init_state_tmp(tmp_idx2(:));
        else
            
            if model_iter ==1
                init_state_tmp = posterior_states_mean(:,t);
                est_dist(t - other_params.start(model_iter)+1, model_iter) = init_state_tmp(1);
                if est_dist(t - other_params.start(model_iter)+1, model_iter)> global_vars.occlusion_blocs_cord(block_id,2)
                        break;
                end
                
                d_lim = min(init_state_tmp(1)+ global_vars.safe_dist, global_vars.occlusion_blocs_cord(block_id,2));

                tmp = unifrnd(init_state_tmp(1), d_lim);       
                %tmp = init_state(1) + tmp_dist;
                insert_states(1) = tmp;
                est_dist(t - other_params.start(model_iter)+1, model_iter) = insert_states(1);
                
                insert_states(3) = unifrnd(max(global_vars.accn_min, -insert_states(2)), min(global_vars.accn_max, global_vars.v_max-insert_states(2) ) );
                
                init_state_tmp = [insert_states; init_state_tmp];
                
            elseif model_iter == models_to_consider
                init_state_tmp = posterior_states_mean(:,t);
                est_dist(t - other_params.start(model_iter)+1, model_iter) = init_state_tmp(end-2);
                if est_dist(t - other_params.start(model_iter)+1, model_iter)> global_vars.occlusion_blocs_cord(block_id,2)
                        break;
                end
                
                tmp_idx1 = size(init_state_tmp,1);
                d_lim = max(init_state_tmp(tmp_idx1-2) - global_vars.safe_dist,  global_vars.occlusion_blocs_cord(block_id,1));
                tmp = unifrnd(d_lim, init_state_tmp(tmp_idx1-2) ); 
                
                insert_states(1) = tmp;
                est_dist(t - other_params.start(model_iter)+1, model_iter) = insert_states(1);
                
                insert_states(3) = unifrnd(max(global_vars.accn_min, -insert_states(2)), min([global_vars.accn_max, global_vars.v_max-insert_states(2), 2*(init_state_tmp(tmp_idx1-2)- insert_states(1)+(init_state_tmp(tmp_idx1-1)-insert_states(2)) + 0.5*init_state_tmp(tmp_idx1) ) ]));
                if isnan(insert_states(3))
                    insert_states(3) = -insert_states(2); %unifrnd(max(global_vars.accn_min, -insert_states(2)), min(global_vars.accn_max, global_vars.v_max-insert_states(2) ) );
                end
                
                init_state_tmp = [init_state_tmp; insert_states];
                
            else
               init_state_tmp = posterior_states_mean(:,t);
               %tmp_idx = find(obj_idx_occlude(:,1)>0,iter,'first');
               tmp_idx1 = model_iter-1;
               tmp_idx2 = model_iter;

               est_dist(t - other_params.start(model_iter)+1, model_iter) = init_state_tmp(3*tmp_idx2-2);
               if est_dist(t - other_params.start(model_iter)+1, model_iter)> global_vars.occlusion_blocs_cord(block_id,2)
                        break;
               end
               
               d_lim1 = global_vars.occlusion_blocs_cord(block_id,1);
               d_lim2 = global_vars.occlusion_blocs_cord(block_id,2);
               %tmp_dist = init_state_tmp(3*tmp_idx1-2) - init_state_tmp(3*tmp_idx2-2);
               %tmp_dist = (init_state(3*(iter-2)+1) - init_state(3*(iter-1)+1))/2;       
               tmp = unifrnd( max(d_lim1, init_state_tmp(3*tmp_idx2-2)), min(d_lim2, init_state_tmp(3*tmp_idx1-2)) );
               insert_states(1) = tmp;
               est_dist(t - other_params.start(model_iter)+1, model_iter) = insert_states(1);
               
               insert_states(3) = unifrnd(max(global_vars.accn_min, -insert_states(2)), min([global_vars.accn_max, global_vars.v_max-insert_states(2), 2*(init_state_tmp(3*tmp_idx1-2)- insert_states(1)+(init_state_tmp(3*tmp_idx1-1)-insert_states(2)) + 0.5*init_state_tmp(3*tmp_idx1) )] ));
                if isnan(insert_states(3))
                    insert_states(3) = -insert_states(2); %unifrnd(max(global_vars.accn_min, -insert_states(2)), min(global_vars.accn_max, global_vars.v_max-insert_states(2) ) );
                end
                
               init_state_tmp = [init_state_tmp(1:3*tmp_idx1); insert_states; init_state_tmp(3*(tmp_idx2-1)+1:end)];
            end
            
        end
            
    
        
        
        obj_idx_occlude_model_t = obj_idx_occlude_model(tmp_idx,t:end,model_iter);
        measurements_model_t = measurements_model(tmp_idx,t:end,model_iter);
        
        if other_params.dim_change == 1 
            %row=model_iter;
            first_occlusion_id = find(sum(obj_idx_occlude_model_t,1)==0, 1,'first');
            if (~isempty(first_occlusion_id) && (first_occlusion_id>1))
                issue_ind = obj_idx_occlude_model_t(end,1:first_occlusion_id-1);
            
            
                if any(issue_ind) %&& (model_iter<size(obj_idx_occlude_model_t,1)) )
                    
                    for c = 1:  size(issue_ind,2)
                        
                        if issue_ind(c)>0 %then need to fix this
                            obj_idx_occlude_model_t(1:end-1, c)= obj_idx_occlude_model_t(2:end,c); 
                            obj_idx_occlude_model_t(end, c) =0;

                            measurements_model_t(1:end-1, c) = measurements_model_t(2:end,c);
                            measurements_model_t(end, c) = -Inf;
                        end
                        
                       
                    end
                    
                end

                
            end
            
            
        elseif other_params.dim_change == -1 
            
            first_occlusion_id = find(sum( obj_idx_occlude_model(:,t:end,model_iter) ,1)==0, 1,'first');
            if (~isempty(first_occlusion_id) && (first_occlusion_id>1))
                temp_hold_occl = obj_idx_occlude_model(:,t:end,model_iter);
                temp_hold_meas = measurements_model(:,t:end,model_iter);
                
                issue_ind = diff(temp_hold_occl(:,1:first_occlusion_id-1));
            
            
                if any(issue_ind(:)==-1) %&& (model_iter<size(obj_idx_occlude_model_t,1)) )
                    
                    for c = 1:  size(issue_ind,2)
                        
                        if any(issue_ind(:,c)==-1) %then need to fix this
                            temp_hold_occl(2:end, c)= temp_hold_occl(1:end-1,c); 
                            temp_hold_occl(1, c) =0;

                            temp_hold_meas(2:end, c) = temp_hold_meas(1:end-1,c);
                            temp_hold_meas(1, c) = -Inf;
                        end
                        
                       
                    end
                    
                    obj_idx_occlude_model_t = temp_hold_occl(tmp_idx,:);
                    measurements_model_t = temp_hold_meas(tmp_idx,:);
        
        
                end
            end
                
            
            
        end
        
        
        
            if (model_iter == models_to_consider)
                if (other_params.dim_change == 1) && (sum(obj_idx_occlude_model_t(model_iter,:),2)==0)
                    break;
                    %wts_hold(t - other_params.start(model_iter)+1,:) =0;
    %             elseif  (other_params.dim_change == -1) && (sum(obj_idx_occlude_model(model_iter,t:end, model_iter),2)==0)
    %                 wts_hold(t - other_params.start(model_iter)+1,:) =0;
                end
            end
                
        
        
        %[~, ~, num_failures, ~, ~, ~, ~, ~, ~, ~, log_lik] = oneD_Particle_filter_occlusion_v1_vectorised(init_state_tmp, obj_idx_occlude_model(:,t:end,model_iter), measurements_model(:,t:end,model_iter), temporal_params, measurement_params, num_particles, global_vars);
        [posterior_mean, ~, num_failures, ~, ~, ~, ~, ~, ~, temp_wts, log_lik] = oneD_Particle_filter_occlusion_v1_vectorised(init_state_tmp, obj_idx_occlude_model_t, measurements_model_t, temporal_params, measurement_params, num_particles, global_vars, states,wts_sample(t,:));
        
        %try
        posterior_mean_hold(:,t - other_params.start(model_iter)+1, model_iter) = posterior_mean(:,end);
        %catch
        %   pause; 
        %end

        %log_lik_hold(t - other_params.start(model_iter)+1,model_iter) = log_lik + sum(log(sum_wts_unnormalised_prev_model(1:t-1)/num_particles));
        %store.(strcat('wts_',num2str(t), 'model_iter_',num2str(model_iter))) = temp_wts;
        wts_hold(t - other_params.start(model_iter)+1,:) = [sum_wts_unnormalised_prev_model(1:t-1), temp_wts];
        

        %log_lik_hold(t - other_params.start(model_iter)+1,model_iter) - sum(log(wts_hold(t - other_params.start(model_iter)+1,:)/num_particles))
        
%         if (log_lik > likelihood_opt)
%             d_opt=t;
%             likelihood_opt = log_lik;
%         end

        t = t+1;
        
    end
% % % % % % %     
% % % % % % %     %wts_hold(wts_hold<1e-20)= 1e-20;
% % % % % % %     % use bayes factor to select the best model
% % % % % % %   log_Bayes_fac = zeros(size(wts_hold,1)-1, len);
% % % % % % %   for model_num =1 : size(wts_hold,1)-1
% % % % % % % 
% % % % % % %       log_Bayes_fac(model_num,:) = cumsum(log(wts_hold(model_num,:)) - log(wts_hold(model_num+1,:)));
% % % % % % % 
% % % % % % %   end
% % % % % % % 
% % % % % % %   %calcukate relative probs of the various models (taking care of
% % % % % % %   %underflow/overflow)
% % % % % % %   log_Bayes_fac(log_Bayes_fac>700)=700;
% % % % % % % 
% % % % % % %   tmp1 = flipud(cumprod(flipud(exp(log_Bayes_fac)),1));
% % % % % % %   tmp2 = sum(tmp1,1)+1;
% % % % % % % 
% % % % % % %   tmp3 = 1./ tmp2;
% % % % % % % 
% % % % % % %   relative_probs = repmat(tmp3,size(wts_hold,1),1) .* [tmp1; ones(1, len)];
% % % % % % %   %relative_probs(isnan(relative_probs))=0;
% % % % % % %   %find best model as son as the max. prob converges to above 99% and converges
% % % % % % %   %===================================================================== 
% % % % % % %   tmp_convergence = double([relative_probs>0.9]);
% % % % % % %   %tmp_convergence_id = Inf(models_to_consider,1);
% % % % % % %   
% % % % % % %   thresh = 3;
% % % % % % %   
% % % % % % %   max_idx = seq_convergence_idx(tmp_convergence, thresh);
% % % % % % %   best_model = find(relative_probs(:, max_idx)==max(relative_probs(:, max_idx)));  
% % % % % % %     
% % % % % % %   dist(model_iter) = other_params.start(model_iter)+(best_model-1); 
% % % % % % %     
% % % % % % %    model_sum_wts_unnormalised(model_iter, :) = wts_hold(best_model,:); 
% % % % % % %  
% % % % % % %     

    

    %run the curr_model for best chosen d_opt
    %k = (sum(wts_hold,2)==0);
    wts_hold_copy = wts_hold;
    wts_hold(wts_hold==-1) = num_particles;
    %wts_hold(k,:)=0;
    
    
    log_lik_hold(1:size(wts_hold,1), model_iter) =  sum(log(wts_hold/num_particles),2);
    idx = find(log_lik_hold(:, model_iter) == max(log_lik_hold(:,model_iter)),1,'first');
    
    dist(model_iter) = other_params.start(model_iter)+(idx-1);
    est_dist_final(model_iter) = est_dist(idx,model_iter);
    
    %sum_wts_unnormalised_curr_model = store.(strcat('wts_',num2str(dist(model_iter)),'model_iter_',num2str(model_iter)));
    model_sum_wts_unnormalised(model_iter, :) = wts_hold_copy(idx,:);
    
%     init_state_tmp = posterior_states_mean(:,d_opt);
%     tmp_idx2 = repmat(tmp_idx',3,1);
%     init_state_tmp = init_state_tmp(tmp_idx2(:));
% 
%     [~, ~, ~, ~, ~, ~, ~, ~, ~, sum_wts_unnormalised_curr_model,~] = oneD_Particle_filter_occlusion_v1_vectorised(init_state_tmp, obj_idx_occlude_model(tmp_idx,d_opt:end,model_iter), measurements_model(tmp_idx,d_opt:end,model_iter), temporal_params, measurement_params, num_particles, global_vars,states, wts_sample(d_opt,:));
%     %[~, ~, num_failures, ~, ~, ~, ~, ~, ~, a, log_lik] = oneD_Particle_filter_occlusion_v1_vectorised(init_state_tmp, obj_idx_occlude_model(tmp_idx,t:end,model_iter), measurements_model(tmp_idx,t:end,model_iter), temporal_params, measurement_params, num_particles, global_vars, 1,wts_sample(t,:));
%         
    
    %model_sum_wts_unnormalised(model_iter, :) = [sum_wts_unnormalised_prev_model(1:dist(model_iter)-1), sum_wts_unnormalised_curr_model];

end


% Calculate Bayes factors and decide best model (of data association)
model_sum_wts_unnormalised_copy = model_sum_wts_unnormalised;
model_sum_wts_unnormalised(model_sum_wts_unnormalised==-1)=1;

 log_Bayes_fac = zeros(models_to_consider-1, len);
  for model_num =1 : models_to_consider-1

      log_Bayes_fac(model_num,:) = cumsum(log(model_sum_wts_unnormalised(model_num,:)) - log(model_sum_wts_unnormalised(model_num+1,:)));
        
      %correct for NaNs due to +inf = -inf
      id = find(isnan(log_Bayes_fac(model_num,:)),1,'first');
      if ~isempty(id)
         replacer =  log_Bayes_fac(model_num,id-1);
         log_Bayes_fac(model_num, isnan(log_Bayes_fac(model_num,:)) ) = replacer;
          
      end
      
  end

  %calcukate relative probs of the various models (taking care of
  %underflow/overflow)
  log_Bayes_fac(log_Bayes_fac>100)=100;

  %tmp0 = exp(log_Bayes_fac);
  
  tmp1 = flipud(cumprod(flipud(exp(log_Bayes_fac)),1));
  %tmp1(isnan(tmp1))=0;
  tmp2 = sum(tmp1,1)+1;

  tmp3 = 1./ (tmp2 + 1e-100); % samll const added to prevent division by zero

  relative_probs = repmat(tmp3,models_to_consider,1) .* [tmp1; ones(1, len)];
  %relative_probs(isnan(relative_probs))=0;
  %find best model as son as the max. prob crosses say 95%
  
%   tmp_convergence = double([relative_probs>0.99]);  
%   thresh = 3;
%   
  %max_idx = seq_convergence_idx(tmp_convergence, thresh);
  tmp_convergence = double([relative_probs>0.99]);
  thresh = 3;
  
%   if (len<50)  %otherwise takes too long to run
%     max_idx = seq_convergence_idx(tmp_convergence, thresh);
%   else
    max_idx = len;
%   end
  
  
% %   max_values = max(relative_probs,[], 1);
% %   max_idx = find(max_values>0.99,1,'first');
  best_model = find(relative_probs(:, max_idx)==max(relative_probs(:, max_idx)));
  
  dist2 = dist(relative_probs(:,max_idx) == max(relative_probs(:, max_idx)));
  weights = model_sum_wts_unnormalised_copy(best_model, :);
  
  
  log_lik_hold(isnan(log_lik_hold(:, best_model)),best_model) =min(log_lik_hold(~isinf(log_lik_hold)))-100;
  k= sum(~isinf(log_lik_hold(:,best_model))); 
  tmp_log_lik = log_lik_hold(1:k,best_model);
  max_tmp_log_lik = max(tmp_log_lik);
  tmp_log_lik = tmp_log_lik - max_tmp_log_lik;
  
  prob_dist = exp(tmp_log_lik); 
  prob_dist = prob_dist/sum(prob_dist);
% %   plot(est_dist(1:numel(prob_dist),best_model),prob_dist ,'o-'); drawnow;

  posterior_mean = posterior_mean_hold(:,dist2 - other_params.start(best_model)+1 , best_model);
 
  est_dist1 = est_dist_final(best_model);
  

  
  relative_probs(best_model, max_idx) =-1;
  best_model2 = find(relative_probs(:, max_idx)==max(relative_probs(:, max_idx)));
  
  est_dist2 = est_dist_final(best_model2);
  
  
  
           