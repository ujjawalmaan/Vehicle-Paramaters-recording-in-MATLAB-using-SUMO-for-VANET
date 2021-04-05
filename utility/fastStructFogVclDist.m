function [dist,connect] = fastStructFogVclDist(x,vhclLoc,trnsmsnRange)
 % fast solution to find the fog node distance to vehicle node and in the
 % transmission range of vehicle

if ~isempty(x.location)
    dist=norm(x.location-vhclLoc);
    if dist<trnsmsnRange || dist ==trnsmsnRange
        connect = 1;
    else
        connect=0;
    end
else
    dist=inf;
    connect=inf;
    
end