function increase_landmarks_cov(obj, minPXLM)

if length(obj.PX) == 15, return, end

PXLM= diag( obj.PX(16:end,16:end) );

minPXLM= minPXLM * ones(length(PXLM),1);

newDiagLM= max(PXLM,minPXLM);

diffDiagLM= PXLM - newDiagLM;

obj.PX(16:end,16:end)= obj.PX(16:end,16:end) - diag( diffDiagLM );

end