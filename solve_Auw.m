function A_star=solve_Auw(A , u , w)
    A_star = A - (A*u-w)*(u'*u)^(-1)*u';
end