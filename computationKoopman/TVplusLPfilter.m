function [f, derivative] = TVplusLPfilter(data, dt)
% dt = 0.0333;
f = data;

% Denoising
%  [temp1, temp2, ~] = lpftvd(f, 2, 0.02, 10, 1000);
 [temp1, temp2, ~] = lpftvd(f, 2, 0.1, 10, 1000);



temp1(end-1:end) = []; temp1(1:2) = []; temp2(end-1:end) = []; temp2(1:2) = [];
f(end-1:end,:) = []; f(1:2,:) = [];


derivative = gradient(temp1+temp2)/dt;


end

%% TOTAL VARIATION DENOISING
function [x, f, cost] = lpftvd(y, d, fc, lam, Nit)
% [x, f, cost] = lpftvd(y, d, fc, lam, Nit)
% Simultaneous low-pass filtering (LPF) and total variation denoising (TVD).
%
% INPUT
%   y - noisy data
%   d - degree of filter is 2d (use d = 1, 2, or 3)
%   fc - cut-off frequency (normalized frequency, 0 < fc < 0.5)
%   lam - regularization parameter (proportional to noise sigma)
%   Nit - number of iterations
%
% OUTPUT
%   x - TV component
%   f - LPF component
%   cost - cost function history
%
% The algorithm is based on majorization-minimization and fast
% solvers for banded linear systems.

% Ivan Selesnick,  NYU-Poly, 2012

y = y(:);                                     % convert to column
cost = zeros(1, Nit);                         % cost function history
N = length(y);

[A, B, B1] = ABfilt(d, fc, N);                % banded filter matrices [sparse]
H = @(x) A\(B*x);                             % H : high-pass filter
S = @(x) [0; cumsum(x)];                      % S : cumulative sum 
Id = @(x) x(d+1:N-d);
AAT = A*A';                                   % A*A' : banded matrix [sparse]
b = (1/lam) * B1'*(AAT\(B*y));                % b : (1/lam) * S * H'*H*y
u = -diff(y);                                 % initialization
for k = 1:Nit
    Lam = spdiags(abs(u), 0, N-1, N-1);       % Lam : diag(abs(u)) [sparse]
    F = lam*AAT + B1*Lam*B1';                 % F : banded matrix [sparse]
    u = Lam * (b - (B1'*(F\(B1*(Lam*b)))));   % update
    cost(k) = 0.5*sum(abs(H(y-S(u))).^2) + lam * sum(abs(u));
end

x = S(u);                                     % x : tv (step) component
bn = nan + zeros(d, 1);                       % bn : nan's to extend f to length N
f = y - x - [bn; H(y-x); bn];                 % f : low-pass component

end

function [A, B, B1, D, a, b, b1] = ABfilt(deg, fc, N)
% [A, B, B1] = ABfilt(d, fc, N)
%
% Banded matrices for zero-phase high-pass recursive filter.
% The matrices are created as 'sparse' structures.
%
% INPUT
%   d  : degree of filter is 2d
%   fc : cut-off frequency (normalized frequency, 0 < fc < 0.5)
%   N  : length of signal
%
% OUTPUT
%   A, B, B1 : banded filter matrices
%       with B = B1*D where D is the first-order difference matrix
%
% Use [A, B, B1, D, a, b, b1] = ABfilt(...) to return
% filter coefficient vectors a, b, b1.

% Ivan Selesnick,  NYU-Poly, 2012

b1 = 1;
for i = 1:2*deg-1
    b1 = conv(b1, [-1 1]);
end
b1 = b1 * (-1)^deg;
b = conv(b1, [-1 1]);

omc = 2*pi*fc;
t = ((1-cos(omc))/(1+cos(omc)))^deg;

a = 1;
for i = 1:deg
    a = conv(a, [1 2 1]);
end
a = b + t*a;

A = spdiags( a(ones(N-2*deg,1), :), -deg:deg, N-2*deg, N-2*deg);    % A: Symmetric banded matrix
B1 = spdiags(b1(ones(N,1), :), 0:2*deg-1, N-2*deg, N-1);            % B1: banded matrix
e = ones(N, 1);
D = spdiags([-e e] , 0:1, N-1, N);
B = B1 * D;                                                         % B: banded matrix

% verify that B = B1*D
x = rand(N,1);
err = B1*diff(x) - B*x;
if max(abs(err)) > 1e-10
    disp('Error in ABfilt (B1*D not equal to B)')
end

end