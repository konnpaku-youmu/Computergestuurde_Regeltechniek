function [L, P_k_1] = KF_update(A, B, C, Q, R, P_k)
    P_k_1 = A*P_k*A' + B*Q*B' - A*P_k*C'*inv(C*P_k*C' + R)*C*P_k*A';
    L = A*P_k*C'*inv(C*P_k*C' + R);
end
