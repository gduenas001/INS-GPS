
function counter= storeData(XX,PX,time,counter)

global DATA


% Store data
DATA.update.time(counter)= time;
DATA.update.PX(:,counter)= diag(PX(1:15,1:15));
DATA.update.XX(:,counter)= XX(1:15);
DATA.update.LM{counter}= XX(16:end);

% Increase counter
counter= counter + 1;



