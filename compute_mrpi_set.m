function Fs = compute_mrpi_set(Ak, W, epsilon)
% Computes an invariant approximation of the minimal robust positively
% invariant set for
% x^{+} = Ax + w with w \in W
% according to Algorithm 1 in 'Invariant approximations of
% the minimal robust positively invariant set' by Rakovic et al.
% Requires a matrix A, a Polytope W, and a tolerance 'epsilon'.
[nx,~] = size(Ak);
s = 0;
alpha = 1000;
Ms = 1000;
E = eye(nx);
it = 0;


disp('-------------------------------------------------')
disp('Calculating Invariant Set ...')
tic
while(alpha > epsilon/(epsilon + Ms))
    s = s+1;
    alpha = max(W.support(Ak^s*(W.A)')./W.b);
    mss = zeros(2*nx,1);
    for i = 1:s
        mss = mss+W.support([Ak^i, -Ak^i]);
    end
    Ms = max(mss);
    it = it+1;
end

Fs = W;
for i =1:s-1
    Fs = Fs+Ak^i*W;
end
Fs = (1/(1-alpha))*Fs;

toc
end