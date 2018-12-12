
function counter= storeData(time,counter)

global DATA XX PX


% Store data
DATA.update.time(counter)= time;
DATA.update.PX(:,counter)= diag(PX(1:15,1:15));
DATA.update.XX(:,counter)= XX(1:15);


% Increase counter
counter= counter + 1;



