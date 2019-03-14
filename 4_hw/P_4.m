A = [ 6  4  1;...
     -5 -4  0;...
     -4 -3 -1];
 
 B = [1 -1 -1]';
 
 alpha = charpoly(A);
 
 C = ctrb(A,B);
 
 temp = [1 alpha(2), alpha(3);...
         0   1        alpha(2);...
         0   0           1];
     
 T = C*temp
 
 