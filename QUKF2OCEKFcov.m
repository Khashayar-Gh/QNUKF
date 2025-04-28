function P_o = QUKF2OCEKFcov(P_ukf)
    p_q = P_ukf(1:3 , 1:3);
    p_p = P_ukf(4:6 , 4:6);
    p_v = P_ukf(7:9 , 7:9);
    p_bw = P_ukf(10:12 , 10:12);
    p_ba = P_ukf(13:15 , 13:15);
    % x = [q , b_w , v  , b_a, p] OCEKF
    P_o = blkdiag(p_q , p_bw , p_v , p_ba , p_p);
%     q = x_o(1:4 , :);
%     p = x_o(14:16 , :); 
%     v = x_o(8:10 , :);    
%     bw = x_o(5:7);
%     ba = x_o(11:13);
%     x_q = [q ; p; v ; bw ; ba];
end