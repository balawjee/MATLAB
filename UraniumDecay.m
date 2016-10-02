%
% 1D radioactive decay
% by Kevin Berwick,
% based on 'Computational Physics' book by N Giordano and H Nakanishi
% Section 1.2 p2
% Solve the Equation dN/dt = -N/tau
N_uranium_initial = 1000; %initial number of uranium atoms
npoints = 100; %Discretize time into 100 intervals
dt = 1e7; % time step in years
tau=4.4e9; % mean lifetime of 238 U
N_uranium = zeros(npoints,1); % initializes N_uranium, a vector of dimension npoints X 1,to being
all zeros
time = zeros(npoints,1); % this initializes the vector time to being all zeros
N_uranium(1) = N_uranium_initial; % the initial condition, first entry in the vector N_uranium is
time(1) = 0; %Initialise time
for step=1:npoints-1 % loop over the timesteps and calculate the numerical solution
N_uranium(step+1) = N_uranium(step) - (N_uranium(step)/tau)*dt;
time(step+1) = time(step) + dt;
end
% For comparison , calculate analytical solution
t=0:1e8:10e9;
N_analytical=N_uranium_initial*exp(-t/tau);
% Plot both numerical and analytical solution
plot(time,N_uranium,'r',t,N_analytical,'b'); %plots the numerical solution in red and the analytical

xlabel('Time in years')
ylabel('Number of atoms')