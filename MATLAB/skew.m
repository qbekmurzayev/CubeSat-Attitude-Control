function S = skew(v)
% skew matrix, skew(v)*u = cross(v,u)

S = [ 0    -v(3)  v(2);
      v(3)  0    -v(1);
     -v(2)  v(1)  0   ];

end
