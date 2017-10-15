function [Phi, Gamma, Lambda] = predictionmatrices(A, B, C, N)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

n = size(A,1);
Atilde = [A; zeros((N-1)*n, n)];
k = [zeros(n, N*n); -kron(eye(N-1), A) zeros((N-1)*n,n)] + eye(N*n);

Phi = k\Atilde;
Gamma = k\kron(eye(N), B);
Lambda = kron(eye(N), C);

end

