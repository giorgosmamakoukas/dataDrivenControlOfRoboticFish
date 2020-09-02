clc; close all; clear; rng(1);

% parameters
ts = 0.005; % time between measurements (related to discrete Koopman)

% 1. pre-process experimental data
interpolateExperimentalData(ts);

% 2. train Koopman operator
trainKoopman(ts);

% 3. compare prediction and experiments
comparePredictionAndExperimentalData;