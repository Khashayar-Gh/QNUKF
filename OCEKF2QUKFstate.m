function x_q = OCEKF2QUKFstate(x_o)
    q = x_o(1:4 , :);
    p = x_o(14:16 , :); 
    v = x_o(8:10 , :);    
    bw = x_o(5:7);
    ba = x_o(11:13);
    x_q = [q ; p; v ; bw ; ba];
end