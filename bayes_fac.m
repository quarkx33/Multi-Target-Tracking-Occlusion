function [log_bayes_fac, fac_curr_period] = bayes_fac(r, temporal_params, measurement_params, measurements, modelA, modelB, global_vars, obj_idx_occlude)

%code to estimate bayes factor
N=size(modelA.particles,2);

%input ratio (in log terms)
r;

%simulate 'N' samples from Y(t)| Y(t-1)
%Y(t) is the complete state vector for all cars according to the model
% Y_curr_timeA = zeros(size(modelA.Y_prev_time));
% Y_curr_timeB = zeros(size(modelB.Y_prev_time));

% % for ii= 1:N
% %     %can vectorise the call below later
% %     tmp = oneD_temporal_model_v2_constrained(modelA.Y_prev_time(:,ii), temporal_params, 2, modelA.num_objects, global_vars);
% %     Y_curr_timeA(:,ii) = tmp(:,2);
% %     
% %     tmp = oneD_temporal_model_v2_constrained(modelB.Y_prev_time(:,ii), temporal_params, 2, modelB.num_objects, global_vars);
% %     Y_curr_timeB(:,ii) = tmp(:,2);
% %     
% %     
% % end

% %pick out the relevant Y's (according to the model selected)
% Y_modelA = Y_curr_timeA(1:3:end,:);
% Y_modelA = Y_modelA(modelA.idx,:);
% 
% 
% %pick out the relevant Y's (according to the model selected)
% Y_modelB = Y_curr_timeB(1:3:end,:);
% Y_modelB = Y_modelB(modelB.idx,:);

%pick out the relevant Y's (according to the model selected)
Y_modelA = modelA.particles(1:3:end,:);
%Y_modelA = Y_modelA(modelA.idx,:);

%pick out the relevant Y's (according to the model selected)
Y_modelB = modelB.particles(1:3:end,:);
%Y_modelB = Y_modelB(modelB.idx,:);






f_modelA = zeros(N,1);
f_modelB = zeros(N,1);

for ii =1:N
    %f1 stores the numerator in the exp( term) of the normal distr
    f_modelA(ii) = -0.5/(measurement_params.noise_sigma^2)* sum((measurements - Y_modelA(:,ii)).^2);
    f_modelB(ii) = -0.5/(measurement_params.noise_sigma^2)* sum((measurements - Y_modelB(obj_idx_occlude,ii)).^2);
    
end


%we calculate bayes factor for model B over model A
%log of bayes factor is calculated

%find max exponent of both models
f_max_modelA = max(f_modelA);
f_max_modelB = max(f_modelB);


%find terms of model A and model B after dividing by max. in both cases
f_modelA = f_modelA - f_max_modelA;
f_modelB = f_modelB - f_max_modelB;




%log of bayes factor for latest time point
fac_curr_period = (f_max_modelB - f_max_modelA) + log(sum(exp(f_modelB))) -log(sum(exp(f_modelA)));

log_bayes_fac = r + fac_curr_period;





