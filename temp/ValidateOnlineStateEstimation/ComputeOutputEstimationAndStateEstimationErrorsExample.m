%% Compute Residuals and State Estimation Errors
%%
% This example shows how to estimate the states of a discrete-time Van der
% Pol oscillator and compute state
% estimation errors and residuals for validating the estimation.
% The residuals are the output estimation errors, that is, they are the 
% difference between the measured and estimated outputs.
%
% In the Simulink(TM) model, the Van der Pol Oscillator block implements the
% oscillator with nonlinearity parameter, mu, equal to 
% 1. The oscillator has two states.
% A noisy measurement of the first state |x1| is available.
%
% The model uses the Unscented Kalman Filter block to estimate the states
% of the oscillator. Since the block requires discrete-time inputs, the 
% Rate Transition block samples |x1| to give the discretized output
% measurement |yMeasured[k]| at time step |k|.
% The Unscented Kalman Filter block outputs the estimated
% state values |xhat[k|k]| at time step |k|, using |yMeasured| until time |k|.
% The filter block uses the previously
% written and saved state transition and measurement functions,
% |vdpStateFcn.m| and |vdpMeasurementFcn.m|. For information about these
% functions, see <docid:ident_ug.bvf58s3>.

model = fullfile(matlabroot,'examples','ident','vdpStateEstimationModel');
open_system('vdpStateEstimationModel')
%%
% To validate the state estimation, the model computes the residuals in
% the Generate Residual block. In addition,
% since the true state values are known, the model also computes the state 
% estimation errors.
%%
% To compute the residuals, the Generate Residual block first computes the
% estimated output |yPredicted[k|k-1]| using the estimated states
% and state transition and measurement functions.
% Here, |yPredicted[k|k-1]| is the estimated output at time step |k|, 
% predicted using output measurements until time step |k-1|.
% The block then computes the residual at time step |k| as 
% |yMeasured[k]| - |yPredicted[k|k-1]|.
%
open_system('vdpStateEstimationModel/Generate Residual')
%%
% Examine the residuals and state 
% estimation errors, and ensure that they have a small magnitude, 
% zero mean, and low autocorrelation.
%
% In this example, the Unscented Kalman Filter block outputs |xhat[k|k]|
% because the *Use the current measurements to improve state estimates*
% parameter of the block is selected. If you clear this parameter,
% the block instead outputs |xhat[k|k-1]|, the predicted state value at 
% time step |k|, using |yMeasured| until time |k-1|.
% In this case, compute |yPredicted[k|k-1] = MeasurementFcn(xhat[k|k-1])|,
% where |MeasurementFcn| is the measurement function for your system.