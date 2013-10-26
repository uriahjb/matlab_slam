%{
    Script for testing the correctness of particle resampling  
%}


%% Particle Filter Configuration
particles.count = 50;
%particles.variance = [1.5, 1.0];
particles.variance = [0.5, 0.5];
particles.state = zeros(particles.count,3);
particles.lidar_hits = {};
particles.hit_cost = zeros(particles.count,1);
particles.cost = zeros(particles.count,1);
particles.Neff = [];

%% Give each particle a unique position that we can use to evaluate if the
%  distribution is correct
particles.state(:,1) = linspace(0,10, particles.count);

%% Generate set of particle costs from distribution
x = linspace( -5, 5, particles.count );
particles.cost = normpdf( x, 0.0, 1.0 );

% Plot distribution
figure(1)
plot( 1:particles.count, particles.cost );

%% Resampling Algorithm
[sorted_pc, inds_pc] = sort(particles.cost);
parents_nrmd = sorted_pc/sum(sorted_pc);
parents_nrmd = parents_nrmd';

%{
num2resample = int16(particles.count*resample_percent);    
parents = sorted_pc((end-num2resample+1):end);
parents_nrmd = exp(parents)/sum(exp(parents));
% Get the indices of the new state of each particle. 
% RESAMPLING STILL HAS ERRORS, WRONG SIZES FOR NUM2RESAMPL AND
% PARENTS_NRMD
%}

idx_counter = [];

% For accurate resampling some additional tricky logic is required
for i = 1:100
    idx = sum(bsxfun(@ge, rand(1,particles.count), cumsum(parents_nrmd) ))' + 1;
    parent_idx = inds_pc( idx );
    idx_counter = [idx_counter parent_idx];
end

unq = unique( idx_counter );
n = hist( idx_counter, unq );
% Rescale to the number of particles
vs = particles.count*n/sum(n);
cnt_vs = ceil(vs);
cnt = sum(cnt_vs);
% If ceil produces an over estimate than remove values in order of
% likelyhood
while cnt >= particles.count        
    [~, min_i] = min( vs(cnt_vs ~= 0) );        
    cnt_vs(min_i) = cnt_vs(min_i) - 1;
    cnt = sum(cnt_vs);
end


figure(2)
hist( idx_counter );


%parent_idx = inds_pc(int16(idx+1) + particles.count-num2resample);
% Update particle values 


%{
particles.state(children_idx,:) = particles.state(parent_idx,:);
particles.hit_cost(children_idx) = particles.hit_cost(parent_idx);
particles.cost(children_idx) = particles.cost(parent_idx);
particles.lidar_hits(children_idx) = particles.lidar_hits(parent_idx);
disp(['Resample: ' num2str(ind)]);
%}