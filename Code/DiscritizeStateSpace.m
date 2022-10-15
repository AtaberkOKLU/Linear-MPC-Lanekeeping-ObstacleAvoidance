function [A, B, C, D] = DiscritizeStateSpace(A_c, B_c, C_c, D_c, T_sampling)
    A = expm(A_c*T_sampling);
    B_func = @(x) expm(A_c*x);
    B = integral(B_func, 0, T_sampling, 'ArrayValued', true)*B_c;
    C = C_c;
    D = D_c;
end