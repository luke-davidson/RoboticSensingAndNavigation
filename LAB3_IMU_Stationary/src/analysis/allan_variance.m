% Luke Davidson
% EECE 5554 - Lab 3
% 5 hr data analyzer

clc;
clear all;
close all;

%% Inertial Sensor Noise Analysis Using Allan Variance
% This example shows how to use the Allan variance to determine noise
% parameters of a MEMS gyroscope. These parameters can be used to model the 
% gyroscope in simulation. The gyroscope measurement is modeled as:
%
% $$\Omega(t) = \Omega_{Ideal}(t) + Bias_N(t) + Bias_B(t) + Bias_K(t)$$
%
% The three noise parameters _N_ (angle random walk), _K_ (rate random
% walk), and _B_ (bias instability) are estimated using data logged from a
% stationary gyroscope.

% Copyright 2018-2020 The MathWorks, Inc.


%% Background
% Allan variance was originally developed by David W. Allan to measure the 
% frequency stability of precision oscillators. It can also be used to 
% identify various noise sources present in stationary gyroscope 
% measurements. Consider _L_ samples of data from a gyroscope with a sample 
% time of $\tau_{0}$. Form data clusters of durations $\tau_{0}$, 
% $2\tau_{0}$, ..., $m\tau_{0}, (m < (L-1)/2)$ and obtain the averages of 
% the sum of the data points contained in each cluster over the length of 
% the cluster. The Allan variance is defined as the two-sample variance of 
% the data cluster averages as a function of cluster time. This example 
% uses the overlapping Allan variance estimator. This means that the 
% calculated clusters are overlapping. The estimator performs better than 
% non-overlapping estimators for larger values of _L_.

%% Allan Variance Calculation
% The Allan variance is calculated as follows:
% 
% Log _L_ stationary gyroscope samples with a sample period $\tau_{0}$. Let 
% $\Omega$ be the logged samples. 

% Load logged data from one axis of a three-axis gyroscope. This recording
% was done over a six hour period with a 100 Hz sampling rate. 
% load('LoggedSingleAxisGyroscope', 'omega', 'Fs')
Fs = 40;

data = readtable('~/Downloads/imu_data.csv');
orientation_x = data{:,6};
orientation_y = data{:,7};
orientation_z = data{:,8};
orientation_w = data{:,9};
linacc_x = data{:,13};
linacc_y = data{:,14};
linacc_z = data{:,15};
gyro_x = data{:,16};
gyro_y = data{:,17};
gyro_z = data{:,18};
magnetic_field_x = data{:,10};
magnetic_field_y = data{:,11};
magnetic_field_z = data{:,12};

omega = linacc_z;
string = 'Linear Acceleration - Z';

t0 = 1/Fs;
%%
% 
% For each sample, calculate the output angle $\theta$:
%
% $$\theta(t) = \int^{t}\Omega(t')dt'$$
%
% For discrete samples, the cumulative sum multiplied by $\tau_{0}$ can be
% used.
theta = cumsum(omega, 1)*t0;
%%
%
% Next, calculate the Allan variance:
%
% $$\sigma^2(\tau) =
% \frac{1}{2\tau^2}<(\theta_{k+2m}-2\theta_{k+m}+\theta_{k})^2>$$
%
% where $\tau = m\tau_{0}$ and $<>$ is the ensemble average. 
%
% The ensemble average can be expanded to:
% 
% $$\sigma^2(\tau) =
% \frac{1}{2\tau^2(L-2m)}\sum_{k=1}^{L-2m}(\theta_{k+2m} - 2\theta_{k+m}
% + \theta_{k})^2$$

maxNumM = 100;
L = size(theta, 1);
maxM = 2.^floor(log2(L/2)); 
m = logspace(log10(1), log10(maxM), maxNumM).';
m = ceil(m); % m must be an integer.
m = unique(m); % Remove duplicates.

tau = m*t0;

avar = zeros(numel(m), 1);
for i = 1:numel(m)
    mi = m(i);
    avar(i,:) = sum( ...
        (theta(1+2*mi:L) - 2*theta(1+mi:L-mi) + theta(1:L-2*mi)).^2, 1);
end
avar = avar ./ (2*tau.^2 .* (L - 2*m));
%%
%
% Finally, the Allan deviation $\sigma(t) = \sqrt{\sigma^2(t)}$ is used to 
% determine the gyroscope noise parameters. 
adev = sqrt(avar);

figure
loglog(tau, adev)
title1 = append('Allan Deviation - ',string);
title(title1);
xlabel('\tau');
ylabel('\sigma(\tau)')
grid on
axis equal

%%
% The Allan variance can also be calculated using the |allanvar| function.

[avarFromFunc, tauFromFunc] = allanvar(omega, m, Fs);
adevFromFunc = sqrt(avarFromFunc);

figure
loglog(tau, adev, tauFromFunc, adevFromFunc);
title2 = append('Allan Deviations - ',string);
title(title2)
xlabel('\tau')
ylabel('\sigma(\tau)')
legend('Manual Calculation', 'allanvar Function')
grid on
axis equal


%% Noise Parameter Identification
% To obtain the noise parameters for the gyroscope, use the following 
% relationship between the Allan variance and the two-sided power spectral
% density (PSD) of the noise parameters in the original data set $\Omega$. 
% The relationship is:
%%
% 
% $$\sigma^2(\tau) = 4\int_{0}^{\infty}S_\Omega(f)
% \frac{sin^4(\pi f\tau)}{(\pi f\tau)^2}df$$
% 
% From the above equation, the Allan variance is proportional to the total
% noise power of the gyroscope when passed through a filter with a transfer
% function of $sin^4(x)/(x)^2$. This transfer function arises from the 
% operations done to create and operate on the clusters. 
%
% Using this transfer function interpretation, the filter bandpass depends 
% on $\tau$. This means that different noise parameters can be identified 
% by changing the filter bandpass, or varying $\tau$.

%% Angle Random Walk
% The angle random walk is characterized by the white noise spectrum of the
% gyroscope output. The PSD is represented by: 
%%
%
% $$S_\Omega(f) = N^2$$
%
% where
%
% _N_ = angle random walk coefficient
%
% Substituting into the original PSD equation and performing integration 
% yields:
%%
%
% $$\sigma^2(\tau) = \frac{N^2}{\tau}$$
%
% The above equation is a line with a slope of -1/2 when plotted on a
% log-log plot of $\sigma(\tau)$ versus $\tau$. The value of _N_ can be
% read directly off of this line at $\tau = 1$. The units of _N_ are
% $(rad/s)/\sqrt{Hz}$.

% Find the index where the slope of the log-scaled Allan deviation is equal
% to the slope specified.
slope = -0.5;
logtau = log10(tau);
logadev = log10(adev);
dlogadev = diff(logadev) ./ diff(logtau);
[~, i] = min(abs(dlogadev - slope));

% Find the y-intercept of the line.
b = logadev(i) - slope*logtau(i);

% Determine the angle random walk coefficient from the line.
logN = slope*log(1) + b;
N = 10^logN

% Plot the results.
tauN = 1;
lineN = N ./ sqrt(tau);
figure
loglog(tau, adev, tau, lineN, '--', tauN, N, 'o')
title3 = append('Allan Deviation with Angle Random Walk - ',string);
title(title3)
xlabel('\tau')
ylabel('\sigma(\tau)')
legend('\sigma', '\sigma_N')
text(tauN, N, 'N')
grid on
axis equal

%% Rate Random Walk
% The rate random walk is characterized by the red noise (Brownian noise) 
% spectrum of the gyroscope output. The PSD is represented by:
%%
%
% $$S_\Omega(f) = (\frac{K}{2\pi})^2\frac{1}{f^2}$$
%
% where
%
% _K_ = rate random walk coefficient
%
% Substituting into the original PSD equation and performing integration 
% yields:
%%
%
% $$\sigma^2(\tau) = \frac{K^2\tau}{3}$$
%
% The above equation is a line with a slope of 1/2 when plotted on a
% log-log plot of $\sigma(\tau)$ versus $\tau$. The value of _K_ can be
% read directly off of this line at $\tau = 3$. The units of _K_ are
% $(rad/s)\sqrt{Hz}$.

% Find the index where the slope of the log-scaled Allan deviation is equal
% to the slope specified.
slope = 0.5;
logtau = log10(tau);
logadev = log10(adev);
dlogadev = diff(logadev) ./ diff(logtau);
[~, i] = min(abs(dlogadev - slope));

% Find the y-intercept of the line.
b = logadev(i) - slope*logtau(i);

% Determine the rate random walk coefficient from the line.
logK = slope*log10(3) + b;
K = 10^logK

% Plot the results.
tauK = 3;
lineK = K .* sqrt(tau/3);
figure
loglog(tau, adev, tau, lineK, '--', tauK, K, 'o')
title4 = append('Allan Deviation with Rate Random Walk - ', string);
title(title4)
xlabel('\tau')
ylabel('\sigma(\tau)')
legend('\sigma', '\sigma_K')
text(tauK, K, 'K')
grid on
axis equal

%% Bias Instability
% The bias instability is characterized by the pink noise (flicker noise) 
% spectrum of the gyroscope output. The PSD is represented by:
%%
%
% $$S_{\Omega}(f) = \left\{\begin{array}{lr}(\frac{B^2}{2\pi})\frac{1}{f} 
% & : f \leq f_0\\0 & : f > f_0\end{array}\right.$$
%
% where
%
% _B_ = bias instability coefficient
%
% $f_0$ = cut-off frequency
%
% Substituting into the original PSD equation and performing integration 
% yields:
%%
%
% $$\sigma^2(\tau) = \frac{2B^2}{\pi}[\ln{2} + \\
% -\frac{sin^3x}{2x^2}(sinx + 4xcosx) + Ci(2x) - Ci(4x)]$$
%
% where
%
% $x = \pi f_0\tau$
%
% _Ci_ = cosine-integral function
%
% When $\tau$ is much longer than the inverse of the cutoff frequency, the 
% PSD equation is:
%%
%
% $$\sigma^2(\tau) = \frac{2B^2}{\pi}\ln{2}$$
%
% The above equation is a line with a slope of 0 when plotted on a log-log
% plot of $\sigma(\tau)$ versus $\tau$. The value of _B_ can be read
% directly off of this line with a scaling of $\sqrt{\frac{2\ln{2}}{\pi}}
% \approx 0.664$. The units of _B_ are $rad/s$.

% Find the index where the slope of the log-scaled Allan deviation is equal
% to the slope specified.
slope = 0;
logtau = log10(tau);
logadev = log10(adev);
dlogadev = diff(logadev) ./ diff(logtau);
[~, i] = min(abs(dlogadev - slope));

% Find the y-intercept of the line.
b = logadev(i) - slope*logtau(i);

% Determine the bias instability coefficient from the line.
scfB = sqrt(2*log(2)/pi);
logB = b - log10(scfB);
B = 10^logB

% Plot the results.
tauB = tau(i);
lineB = B * scfB * ones(size(tau));
figure
loglog(tau, adev, tau, lineB, '--', tauB, scfB*B, 'o')
title5 = append('Allan Deviation with Bias Instability - ', string);
title(title5)
xlabel('\tau')
ylabel('\sigma(\tau)')
legend('\sigma', '\sigma_B')
text(tauB, scfB*B, '0.664B')
grid on
axis equal

%% 
%
% Now that all the noise parameters have been calculated, plot the Allan
% deviation with all of the lines used for quantifying the parameters.
tauParams = [tauN, tauK, tauB];
params = [N, K, scfB*B];
figure
loglog(tau, adev, tau, [lineN, lineK, lineB], '--', ...
    tauParams, params, 'o')
title6 = append('Allan Deviation with Noise Parameters - ', string);
title(title6)
xlabel('\tau')
ylabel('\sigma(\tau)')
legend('$\sigma (rad/s)$', '$\sigma_N ((rad/s)/\sqrt{Hz})$', ...
    '$\sigma_K ((rad/s)\sqrt{Hz})$', '$\sigma_B (rad/s)$', 'Interpreter', 'latex')
text(tauParams, params, {'N', 'K', '0.664B'})
grid on
axis equal

%% Gyroscope Simulation
% Use the |imuSensor| object to simulate gyroscope measurements with the
% noise parameters identified above.

% Simulating the gyroscope measurements takes some time. To avoid this, the
% measurements were generated and saved to a MAT-file. By default, this
% example uses the MAT-file. To generate the measurements instead, change
% this logical variable to true.
generateSimulatedData = false;

if generateSimulatedData
    % Set the gyroscope parameters to the noise parameters determined 
    % above. 
    gyro = gyroparams('NoiseDensity', N, 'RandomWalk', K, ...
        'BiasInstability', B);
    omegaSim = helperAllanVarianceExample(L, Fs, gyro);
else
    load('SimulatedSingleAxisGyroscope', 'omegaSim')
end

%%
% 
% Calculate the simulated Allan deviation and compare it to the logged 
% data.
%

[avarSim, tauSim] = allanvar(omegaSim, 'octave', Fs);
adevSim = sqrt(avarSim);
adevSim = mean(adevSim, 2); % Use the mean of the simulations.

figure
loglog(tau, adev, tauSim, adevSim, '--')
title7 = append('Allan Deviation of HW and Simulation - ', string);
title(title7)
xlabel('\tau');
ylabel('\sigma(\tau)')
legend('HW', 'SIM')
grid on
axis equal

%%
% The plot shows that the gyroscope model created from the |imuSensor| 
% generates measurements with similar Allan deviation to the logged data. 
% The model measurements contain slightly less noise since the quantization
% and temperature-related parameters are not set using |gyroparams|. 
% The gyroscope model can be used to generate measurements using movements 
% that are not easily captured with hardware. 

%% References
% * IEEE Std. 647-2006 IEEE Standard Specification Format Guide and Test 
% Procedure for Single-Axis Laser Gyros
