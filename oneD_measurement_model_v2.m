%% Temporal model implementation (just measures x co-ordinates)

function obs = oneD_measurement_model_v2(states, measurement_params)


T = size(states,2);

noise = measurement_params.noise_mean +  measurement_params.noise_sigma * randn(size(states,1)/3, T);
obs = measurement_params.mean + states(1:3:end,:) + noise; % picking out only the x-coordinates of psoition

%Reset any measurements to NA(-1s) for which states is invalid
idx = isinf(states(1:3:end,:));

obs(idx) = -Inf; % set to _inf to denote value not used





