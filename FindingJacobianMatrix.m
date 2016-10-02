syms x y z
F=[x*x x*y x*y + y*z; sin(x) cos(y) tan(z);0 -sec(z) x+ y*sin(z)];
jacobian(F(1,:),[x y z])
jacobian(F(2,:),[x y z])
jacobian(F(3,:),[x y z])