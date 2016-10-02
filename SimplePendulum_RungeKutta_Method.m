% 2nd order Runge Kutta solution
initial_displacement=10;
npoints = 2500; %Discretize time into 250 intervals
dt = 0.04; % time step in seconds
v = zeros(npoints,1); % initializes v, a vector of dimension npoints X 1,to being all zeros
y = zeros(npoints,1); % initializes y, a vector of dimension npoints X 1,to being all zeros
time = zeros(npoints,1); % this initializes the vector time to being all zeros
y(1)=initial_displacement; % need some initial displacement
v = zeros(npoints,1); % initializes v, a vector of dimension npoints X 1,to being all zeros
y = zeros(npoints,1); % initializes y, a vector of dimension npoints X 1,to being all zeros
v_dash = zeros(npoints,1); % initializes y, a vector of dimension npoints X 1,to being all zeros
y_dash = zeros(npoints,1); % initializes y, a vector of dimension npoints X 1,to being all zeros
time = zeros(npoints,1); % this initializes the vector time to being all zeros
y(1)=10; % need some initial displacement
for step = 1:npoints-1 % loop over the timesteps
 v_dash=v(step)-0.5*y(step)*dt;
 y_dash=y(step)+0.5*v(step)*dt;

 v(step+1) = v(step)-y_dash*dt;
 y(step+1) = y(step)+v_dash*dt;
 time(step+1) = time(step)+dt;
end; 

plot(time,y,'g' );
axis([0 100 -20 20]);
xlabel('Time');
ylabel('Displacement');
legend ('Runge-Kutta method');
