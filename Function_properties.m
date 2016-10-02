%asymptotes, critical point & inflection points
syms x
num = 3*x^2 + 6*x -1;
denom = x^2 + x - 3;
f = num/denom

%horizontal asymptote of function f
limit(f, inf)

%roots of quadratic equation
roots=solve(denom);

%differentiate f
f1 = diff(f)

%simplify expression
f1 = simplify(f1)

%display result in more presentable format
pretty(f1)

%critical points
crit_pts = solve(f1)
