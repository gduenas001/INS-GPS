function lidar_update(obj, z, params)

obj.XX_update(params.ind_yaw)= pi_to_pi( obj.XX_update(params.ind_yaw) ); 


if all(obj.association == 0)
    obj.n_k= 0;
%     obj.Y_k= [];
%     obj.L_k= [];
%     obj.gamma_k= [];
%     obj.q_k= 0;
%     obj.H_k= [];
    obj.number_of_associated_LMs= 0;
    return;
end

% Eliminate the non-associated features
z(obj.association == 0, :)= [];

obj.association_no_zeros = obj.association(obj.association ~= 0);

% number of associated features
obj.n_k= length(obj.association_no_zeros) * params.m_F;

%obj.H_k = obj.return_lidar_H(obj.XX_update, obj.association_no_zeros, params);

R= kron( params.R_lidar, eye( obj.n_k / params.m_F ) );

obj.V_k = R;

zVector= z';
zVector= zVector(:);

obj.z_k = zVector;

obj.weight_vector= zeros(size(obj.XX_particles_update,1),1);

%obj.h_k=inf*ones( obj.n_k , 1 );
%Build Jacobian H
%spsi= sin(obj.XX_update(params.ind_yaw));
%cpsi= cos(obj.XX_update(params.ind_yaw));
%zHat= zeros(obj.n_k,1);
%for i= 1:length(obj.association_no_zeros)
%    % Indexes
%    indz= 2*i + (-1:0);
%
%    dx= obj.landmark_map(obj.association_no_zeros(i), 1) - obj.XX_update(1);
%    dy= obj.landmark_map(obj.association_no_zeros(i), 2) - obj.XX_update(2);
%
%    % Predicted measurement
%    zHat(indz)= [dx*cpsi + dy*spsi;
%                -dx*spsi + dy*cpsi];
%end
%obj.h_k = zHat;

obj.h_k_i=inf*ones( obj.n_k , size(obj.XX_particles_update,1) );

obj.H_k_particles = inf*ones(obj.n_k,params.m*size(obj.XX_particles_update,1));

for j=1:size(obj.XX_particles_update,1)
    obj.H_k_particles(:,params.m*(j-1)+1:params.m*j) = obj.return_lidar_H(obj.XX_particles_update(j,:), obj.association_no_zeros, params);
    %Build Jacobian H
    spsi= sin(obj.XX_particles_update(j,params.ind_yaw));
    cpsi= cos(obj.XX_particles_update(j,params.ind_yaw));
    zHat= zeros(obj.n_k,1);
    for i= 1:length(obj.association_no_zeros)
        % Indexes
        indz= 2*i + (-1:0);

        dx= obj.landmark_map(obj.association_no_zeros(i), 1) - obj.XX_particles_update(j,1);
        dy= obj.landmark_map(obj.association_no_zeros(i), 2) - obj.XX_particles_update(j,2);

        % Predicted measurement
        zHat(indz)= [dx*cpsi + dy*spsi;
                    -dx*spsi + dy*cpsi];
                
    end
    obj.h_k_i(:, j) = zHat;
    obj.weight_vector(j)=mvnpdf(zVector,zHat,R);
end

obj.weight_vector = obj.weight_vector/sum(obj.weight_vector);

XX_particles_new=ones(obj.number_of_particles,params.m);
particles_indices_new= zeros(obj.number_of_particles,1);

for j=1:obj.number_of_particles
    ind = find( mnrnd(1,obj.weight_vector) );
    if (j==1)
        particles_indices_new(j) = 1;
    else
        % check if the new particle is repeated
        check_matrix= ismember(XX_particles_new,obj.XX_particles_update(ind,:));
        check_vector=check_matrix(:,1)&check_matrix(:,2)&check_matrix(:,3);
        if check_vector'*check_vector >0
            particles_indices_new(j) = particles_indices_new(find(check_vector,1));
        else
            particles_indices_new(j) = max(particles_indices_new)+1;
        end
    end
    XX_particles_new(j,:)= obj.XX_particles_update(ind,:);
end

obj.XX_particles_update= XX_particles_new;
obj.particles_indices_update = particles_indices_new;

obj.XX_update= mean(obj.XX_particles_update)';
%[~, indices_of_unique_particles, ~] = unique(obj.particles_indices_update);
%obj.SX_update = cov(obj.XX_particles_update(indices_of_unique_particles,:));
obj.SX_update = cov(obj.XX_particles_update);

obj.number_of_associated_LMs= length(obj.association_no_zeros);
end