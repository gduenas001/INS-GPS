dummy=[];
for (i = 1:32)
    if (isempty(FG.gps_R{i})==0)
        dummy=[dummy,sqrt(FG.gps_R{i})];
    end
end