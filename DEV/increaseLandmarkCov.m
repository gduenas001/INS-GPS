
function [PX]= increaseLandmarkCov(PX,minPXLM)

if length(PX) == 15, return, end

PXLM= diag( PX(16:end,16:end) );

minPXLM= minPXLM * ones(length(PXLM),1);

newDiagLM= max(PXLM,minPXLM);

diffDiagLM= PXLM - newDiagLM;

PX(16:end,16:end)= PX(16:end,16:end) - diag( diffDiagLM );






