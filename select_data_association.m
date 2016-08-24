function [best_model, weights, posterior_mean, best_model2] = select_data_association(init_state_model, obj_idx_occlude_model, measurements_model, temporal_params, measurement_params, num_particles, global_vars, states)

%This is again like another Bayesian model selection
%Here number of cars remain same but data association chnages maintaining
%"order" of vehicles.

len = size(measurements_model,2);
models_to_consider =size(init_state_model, 2); 
model_sum_wts_unnormalised = zeros(models_to_consider, len);
posterior_mean_hold = zeros( size(init_state_model,1), models_to_consider);

for model_iter = 1: models_to_consider
    
    last_occlusion_id = find(sum(obj_idx_occlude_model(:,:,model_iter),1)==0, 1,'last');
    if isempty(last_occlusion_id)
        last_occlusion_id =0;
    end
    if (model_iter == models_to_consider) && (sum(obj_idx_occlude_model(model_iter,last_occlusion_id+1:end,model_iter),2)==0)
        continue;
        %model_sum_wts_unnormalised(model_iter, :) =0;
    end

    [posterior_mean, ~, ~, ~, ~, ~, ~, ~, ~, sum_wts_unnormalised, ~] = oneD_Particle_filter_occlusion_v1_vectorised(init_state_model(:, model_iter), obj_idx_occlude_model(:,:,model_iter), measurements_model(:,:,model_iter), temporal_params, measurement_params, num_particles, global_vars, states);

    model_sum_wts_unnormalised(model_iter, :) = sum_wts_unnormalised;
    posterior_mean_hold(:,model_iter) = posterior_mean(:,end);
    
    %Note if model selected is such that it assumes that there is a last
    %car and it is "always" occluded ... then we diacrad this model as we can
    %never have any idnciation of this through the likelihood
    

end




% set the 0 wts to 1 so that taking log makes them zero .. equivalent to 1
% prob. (as these dont have observations)
model_sum_wts_unnormalised_copy = model_sum_wts_unnormalised;
model_sum_wts_unnormalised(model_sum_wts_unnormalised==-1)=1;

% Calculate Bayes factors and decide best model (of data association)
 log_Bayes_fac = zeros(models_to_consider-1, len);
  for model_num =1 : models_to_consider-1

      log_Bayes_fac(model_num,:) = cumsum(log(model_sum_wts_unnormalised(model_num,:)) - log(model_sum_wts_unnormalised(model_num+1,:)));

  end

  %calcukate relative probs of the various models (taking care of
  %underflow/overflow)
  log_Bayes_fac(log_Bayes_fac>100)=100;

  tmp1 = flipud(cumprod(flipud(exp(log_Bayes_fac)),1));
  tmp2 = sum(tmp1,1)+1;

  tmp3 = 1./ (tmp2 + 1e-100);

  relative_probs = repmat(tmp3,models_to_consider,1) .* [tmp1; ones(1, len)];
  
  %find best model as son as the max. prob converges to above 99% and converges
  %===================================================================== 
  tmp_convergence = double([relative_probs>0.99]);
  %tmp_convergence_id = Inf(models_to_consider,1);
  
  thresh = 3;
  
%   if (len<50)  %otherwise takes too long to run
%     max_idx = seq_convergence_idx(tmp_convergence, thresh);
%   else
    max_idx = len;
%   end
% % %   %old fashioned way of finding the earliuest convergence
% % %   for ii = 1: models_to_consider
% % %       c=1;
% % %       
% % %       while(c<=(len-thresh+1))
% % %         start_seq = find(tmp_convergence(ii,c:end)==1,1,'first');
% % %         if isempty(start_seq)
% % %             break;
% % %         end
% % %         c = c+start_seq-1;
% % %         
% % %         end_seq = find(tmp_convergence(ii,c:end)==0,1,'first');
% % %         if isempty(end_seq)
% % %             if (len-(c-1))>=thresh
% % %                 tmp_convergence_id(ii) = c;
% % %             else
% % %                 c=Inf;
% % %                 break;
% % %             end
% % %         end
% % %         
% % %         if (end_seq-1)>=thresh
% % %             tmp_convergence_id(ii) = c;
% % %         else
% % %             c = c+end_seq-1;
% % %         end
% % %       
% % %       end
% % %       
% % %       
% % %   end
% % %   
% % %   convergence_idx = min(tmp_convergence_id);
% % %   if isinf(convergence_idx)
% % %       max_idx = len;
% % %   else
% % %       max_idx = convergence_idx;
% % %   end

  %===================================================================== 
  
  
  %max_values = max(relative_probs,[], 1);
  %max_idx = find(max_values>0.99,1,'first');
  best_model = find(relative_probs(:, max_idx)==max(relative_probs(:, max_idx)));
  weights = model_sum_wts_unnormalised_copy(best_model, :);
  
  posterior_mean = posterior_mean_hold(:,best_model);
  
  
  
  relative_probs(best_model, max_idx) =-1;
  best_model2 = find(relative_probs(:, max_idx)==max(relative_probs(:, max_idx)));
  
  

  
           