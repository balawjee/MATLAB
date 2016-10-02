syms x y
f1(x) = sin(x^2);
df1 = diff(f1,x)

f2(x,y)=x*sin(x*y);
diff(f2, x, x, x, y)