%generate 1000 random numbers between 0 and 1
gaussian_data=randn(10000,1);
plot(gaussian_data);
%create histogram with 100 bins
hist(gaussian_data,100);

%fit data to a probability distribution. Since array contains random
%numbers, the distribution will be Gaussian
histfit(gaussian_data);
