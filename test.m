
% M= obj.M_M;
% f= rand(size(M,1),1);
% 
% [U,D,V]= svd(M);
% 
% U'*M*pinv(M)*U
% all( all( abs(U'*M*pinv(M)*U - eye(size(M))) < 1e-7 ) )

% all( all( abs(U'*U - eye(size(U))) < 1e-7 ) )

% all( all( abs( M*pinv(M) - eye(size(M)) ) < 1e-5 ) )
% 
% D_full= D(1:rank(M),1:rank(M));
% U_full= U(:,1:rank(M));
% U_0= U(:,rank(M)+1:end);

% all( all( abs(M*pinv(M) - eye(size(M))) < 1e-7 ) )
% all( all( abs(pinv(M)*M - eye(size(M))) < 1e-7 ) )

% all( all( abs(U_full*U_full' + U_0*U_0' - eye(size(U,1))) < 1e-7 ) )

% all( all( abs(U_full'*U_full - eye(size(U_full'*U_full))) < 1e-7 ) )

% [Q,R]= qr(M);


% all( all( abs( Q'*Q - eye(size(Q)) ) < 1e-5 ) )

% rank(R);

% all(all(M - M' < 1e-5))


% f'*M*f
% 
% sqrt_M= U_full*sqrt(D_full)*U_full';
% norm( (sqrt_M*f) )^2;
% norm( sqrt(D_full)* U_full' * f )^2


%%

I= 1e-6;
n= 14;
P= 1e-4;

for r= 1:n
    if  (P*n)^r  / factorial(r)  < I
        n_max= r - 1;
        break
    end
end

n_max

n_H= 0;
for i= 1:n_max
    n_H= n_H + nchoosek(n, i);
end

n_H
























