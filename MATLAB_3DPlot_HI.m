x = linspace(-3,3,50);
y = linspace(-5,5,50);
[X Y]=meshgrid(x,y);
Z = exp(-X.^2-Y.^2/2).*cos(4*X) + exp(-3*((X+0.5).^2+Y.^2/2));
Z(Z>0.001)=0.001;
Z(Z<-0.001)=-0.001;
surf(X,Y,Z);
colormap(flipud(cool))
view([1 -1.5 2])