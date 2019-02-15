function T = helperGenerateFeatureTable(Ensemble, CandidateFeatures, Names)
%helperGenerateFeatureTable Extract features from residues.
% This function is only in support of
% CentrifugalPumpFaultDiagnosisUsingResidualAnalysisExample. It may change
% in a future release.

%  Copyright 2017 The MathWorks, Inc.

[N,m] = size(Ensemble);
nf = length(Names);
F = zeros(N,m*nf);
ColNames = cell(1,m*nf);
for j = 1:nf
   fcn = CandidateFeatures{j};
   F(:,(j-1)*m+(1:m)) = cellfun(@(x)fcn(x),Ensemble,'uni',1);
   ColNames((j-1)*m+(1:m)) = strseq(Names{j},1:m);
end
M1 = max(F);
M2 = min(F);
Range = M1-M2;
F = F./Range;
T = array2table(F,'VariableNames',ColNames);
