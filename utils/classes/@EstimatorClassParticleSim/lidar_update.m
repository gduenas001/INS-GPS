function lidar_update(obj, z, params)

obj.XX(params.ind_yaw)= pi_to_pi( obj.XX(params.ind_yaw) ); 


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

R= kron( params.R_lidar, eye( obj.n_k / params.m_F ) );

zVector= z';
zVector= zVector(:);

obj.weight_vector= zeros(size(obj.XX_particles,1),1);

for j=1:size(obj.XX_particles,1)
    %Build Jacobian H
    spsi= sin(obj.XX_particles(j,params.ind_yaw));
    cpsi= cos(obj.XX_particles(j,params.ind_yaw));
    zHat= zeros(obj.n_k,1);
    for i= 1:length(obj.association_no_zeros)
        % Indexes
        indz= 2*i + (-1:0);

        dx= obj.landmark_map(obj.association_no_zeros(i), 1) - obj.XX_particles(j,1);
        dy= obj.landmark_map(obj.association_no_zeros(i), 2) - obj.XX_particles(j,2);

        % Predicted measurement
        zHat(indz)= [dx*cpsi + dy*spsi;
                    -dx*spsi + dy*cpsi];
    end
    obj.weight_vector(j)=mvnpdf(zVector,zHat,R);
end

obj.weight_vector = obj.weight_vector/sum(obj.weight_vector);

XX_particles_new=ones(obj.number_of_particles,params.m);

for j=1:obj.number_of_particles
    ind = find( mnrnd(1,obj.weight_vector) );
    XX_particles_new(j,:)= obj.XX_particles(ind,:);
end

obj.XX_particles= XX_particles_new;

obj.XX= mean(obj.XX_particles)';
obj.PX= cov(obj.XX_particles);

obj.number_of_associated_LMs= length(obj.association_no_zeros);
end