%% Particle filter implementation (Adapted to apply constraints using KKT)

function [posterior_states, posterior_cov, num_failures, posterior_states1, posterior_cov1, posterior_states_resamp, posterior_cov_resamp, all_particles ] = oneD_Particle_filter_occlusion_v1(init_state, obj_idx_occlude, measurements, temporal_params, measurement_params, num_particles, global_vars, states)

%obj_idx_occlude holds a binary flag if object k is occluded at time t
% time across columns, object occlusion indicator across rows. If 
% no object is occluded at time k, then kth column is all zero

dim = size(init_state,1);
[num_objects, T] = size(measurements);

num_failures = 0;

%Initiliaze variables for solving convex optimisation (using quadratic
%programming)
options = optimset('LargeScale', 'off', 'TolX', 1e-10, 'TolFun', 1e-10, 'TolCon', 1e-10, 'GradObj', 'off', ...
             'MaxIter', 100, 'Display', 'off');
eps = 1;

A = eye(dim);


A_trunc = zeros(num_objects,dim);
for ii = 1: num_objects
   A_trunc(ii,(ii-1)*3+1)=1;
   
end

sigma_tmp = cov(measurements');
sigma = zeros(dim);
sigma([1:3:end],[1:3:end]) = sigma_tmp;

%H = A' *pinv(sigma) * A;
H = A' * A;


Aeq = zeros(num_objects-1,dim);
for ii = 1: num_objects-1
   Aeq(ii,(ii-1)*3+1)=-1;
   Aeq(ii, ii*3+1)=1;
   
end
b = -repmat(global_vars.safe_dist,num_objects-1,1);


%Initialise weights and particles
posterior_states = zeros(dim,T);
posterior_cov = zeros(dim, dim, T);

posterior_states1 = zeros(dim,T);
posterior_cov1 = zeros(dim, dim, T);

posterior_states_resamp = zeros(dim,T-1);
posterior_cov_resamp = zeros(dim, dim, T-1);


% For first time point .. initialize tracker to actual value of state    
particles = repmat(init_state,1, num_particles)+ temporal_params.noise_sigma * randn(dim,num_particles);
particles_new = zeros(size(particles));
wts = repmat(1/num_particles,1,num_particles);
posterior_states(:,1) =  init_state;
posterior_cov(:,:,1) = 0*eye(dim);

posterior_states1(:,1) =  posterior_states(:,1);
posterior_cov1(:,:,1) = posterior_cov(:,:,1);

% % posterior_states_resamp(:,2) =  posterior_states(:,1);
% % posterior_cov_resamp(:,:,2) = posterior_cov(:,:,1);


%Stores all the particles to pass back to main calling function
all_particles = zeros(size(init_state,1),num_particles,T);
all_particles(:,:,1) = particles;

for t = 2:T
   %t; 

    try
    particles = datasample(particles, num_particles, 2,'Replace', true, 'Weights', wts);
    catch
        %possible issue with wts becoming NaN.. due to issues in the
        %particles
        
%         %reset the measurment part of the particles
%         particles_new = particles;
%         particles_new(1:3:end,:) = repmat(measurements(:,t), 1, num_particles) + 0.001* randn(num_objects,num_particles);
%         wts = repmat(1/num_particles,1,num_particles);
%         pause;
        
    end
    
    
       %%====== Calculate the mean and cov. just after the resampling ---should be equal to 
       %%====== posterior_states and posterior_cov at prev time step
       %%===
    %Estimate posterior state vector (mean) 
    % All particles have equal wts now (since done resampling)
    % tmp_wts1 = repmat(wts, dim, 1);
     posterior_states_resamp(:, t-1) = mean(particles,2);
     
     %Estimate the posterior covariance vector
      %tmp = sqrt(tmp_wts1) .* (particles_new - repmat(posterior_states1(:, t),1, num_particles));
      posterior_cov_resamp(:,:,t-1) =  cov(particles', 1);%tmp * tmp';
      
    % ========= end of code for Calculating the mean and cov. just after resamploing======
    
    

    for jj = 1: num_particles
        % Pass through temporal update model   
        %maybe can vectorise the statement below===
        tmp = oneD_temporal_model_v2_constrained(particles(:,jj), temporal_params, 2, num_objects, global_vars);
        particles_new(:,jj) = tmp(:,2);
        
% % %         %Check if particles satisfy constraints else solve KKT equations -
% % %         %if using constrained temporal equations, particles will always satisfy
% % %         %constraints
% % %         x = particles_new(1:3:end,jj);
% % %         if any (diff(flipud(x))<=0)           
% % %             particles_new(:,jj) = quadprog(H, - particles_new(:,jj)'*H, Aeq , b,[],[],[],[],[],options);
% % %         end
    
    end
    


    %Check there are no NaNs in particle values for x-position and fix if necessary
    x = particles_new(1:3:end,:);
    tmp_measurements = repmat(measurements(:,t),1,num_particles); 
    x(isnan(x))= tmp_measurements(isnan(x));
    particles_new(1:3:end,:) = x;
    
    %%====== Calculate the mean and cov. before the measurement update step======
    %Estimate posterior state vector (mean) 
    % All particles have equal wts now (since done resampling)
    % tmp_wts1 = repmat(wts, dim, 1);
     posterior_states1(:, t) = mean(particles_new,2);
     
     %Estimate the posterior covariance vector
      %tmp = sqrt(tmp_wts1) .* (particles_new - repmat(posterior_states1(:, t),1, num_particles));
      posterior_cov1(:,:,t) =  cov(particles_new', 1);%tmp * tmp';
      
    % ========= end of code for Calculating the mean and cov. before the measurement update step======
      
    
    % Set weight for particles according to measurements made
   
    % assume given the state vectors, measurements are conditionally indep.
    %Only take measurements which are observed (i.e. according to obj_idx_occlude) 
    tmp1 = repmat(measurements(obj_idx_occlude(:,t) ,t),1,num_particles) - A_trunc(obj_idx_occlude(:,t),:)* particles_new - measurement_params.mean - measurement_params.noise_mean;
    tmp1 = -(0.5/(measurement_params.noise_sigma)^2)*(tmp1.^2);
    wts_new = exp(sum(tmp1,1));   

    

    if sum(wts_new)==0
       %fprintf(1,'tracker failure\n');
       %all particles are too far away from measurements so all weights are
       %zero. This can happen if measurements violate constraints. If this
       %is true, adjust measurements. If measurements ok then choose new particles which are all initialised at
       %previous measurements with vel and accelartion equal to last estimated figures
       %for this time point
       
      num_failures = num_failures +1;
 
      
        x = measurements(:,t);
        if any (diff(flipud(x))<=0)
            x = [measurements(:,t)';zeros(2,num_objects)];
            revised_measurements = quadprog(H, - x(:)'*H, Aeq , b,[],[],[],[],[],options);
            revised_measurements = revised_measurements(1:3:end);
            tmp1 = repmat(revised_measurements(obj_idx_occlude(:,t)),1,num_particles) - A_trunc(obj_idx_occlude(:,t),:)* particles_new - measurement_params.mean - measurement_params.noise_mean;
            tmp1 = -(0.5/(measurement_params.noise_sigma)^2)*(tmp1.^2);
            wts_new = exp(sum(tmp1,1));
        end
        
        if sum(wts_new)==0 % sum_wts is still zero 
           %issue is not with erroneous measurements alone (if at all). Particles simply too far away.
           %So make new particles initialised at true state values
           
% % % %             x = [measurements(:,t)';zeros(2,num_objects)];
% % % %             revised_measurements = quadprog(H, - x(:)'*H, Aeq , b,[],[],[],[],[],options);
% % % %             revised_measurements = revised_measurements(1:3:end);
% % % %     
% % % %                 
% % % %             %for missing measurements keep old particle values, if they are valid
% % % %             %else simply interpolate between  of neighbouring particles
% % % %             %(extrapolate for particles at edge)
% % % %             
% % % %             tmp1 = NaN(num_objects, num_particles); %Nan denotes missing values
% % % %             tmp1(obj_idx_occlude(:,t),:) =  repmat(revised_measurements(obj_idx_occlude(:,t)), 1, num_particles) + 0.001* randn(sum(obj_idx_occlude(:,t)),num_particles);
% % % %             
% % % %             non_missing_idx = find(obj_idx_occlude(:,t));
% % % %             missing_idx = find(~obj_idx_occlude(:,t));
% % % %             
% % % %             tmp2 = interp1(non_missing_idx,revised_measurements(obj_idx_occlude(:,t)),missing_idx, 'linear', 'extrap');
% % % %             %fprintf(1,'ss\n');
% % % %             
% % % %             tmp1(isnan(tmp1)) = repmat(tmp2(:),1,num_particles);
% % % %             particles_new(1:3:end,:) = tmp1;
            
            %Initilise particles at t-1 (time point before tracker failure)
            %and pass htrough temporal model. This gives a bit of diversity
            %to the particles
            
            for jj = 1: num_particles
                tmp = oneD_temporal_model_v2_constrained(states(:,t-1), temporal_params, 2, num_objects, global_vars);
                particles_new(:,jj) = tmp(:,2);

            end
            
            wts_new = repmat(1/num_particles,1,num_particles);
            posterior_cov1(:,:,t) =  cov(particles_new', 1);
        end

    end
    particles = particles_new;
    
    all_particles(:,:,t) = particles;
    
    % Normalise weights
     wts = wts_new / sum(wts_new);
     
     
     
     
% % % %        %%====== Calculate the mean and cov. without the resampling weights ======
% % % %     %Estimate posterior state vector (mean) 
% % % %     % All particles have equal wts now (since done resampling)
% % % %     % tmp_wts1 = repmat(wts, dim, 1);
% % % %      posterior_states1(:, t) = mean(particles_new,2);
% % % %      
% % % %      %Estimate the posterior covariance vector
% % % %       %tmp = sqrt(tmp_wts1) .* (particles_new - repmat(posterior_states1(:, t),1, num_particles));
% % % %       posterior_cov1(:,:,t) =  cov(particles_new', 1);%tmp * tmp';
% % % %       
% % % %     % ========= end of code for Calculating the mean and cov. before the measurement update step======
% % % %    
     
     
     %Estimate posterior state vector (mean)
     tmp_wts = repmat(wts, dim, 1);
     posterior_states(:, t) = sum(tmp_wts .* particles,2);
     
     %Estimate the posterior covariance vector
      tmp = sqrt(tmp_wts) .* (particles - repmat(posterior_states(:, t),1, num_particles));
      posterior_cov(:,:,t) = tmp * tmp';
     
% % %       if (rcond(posterior_cov(:,:,t))<0.1)
% % %           %t
% % %           %regularise to make sure matrix is non-singular
% % %           posterior_cov(:,:,t) =  posterior_cov(:,:,t) + eps*eye(dim);
% % % 
% % %       end
    
    
end




