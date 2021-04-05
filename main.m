%%
% this is the first part of the work. This place the FOG nodes in the SUMO
% map and estimates th position by Kalman and PGRP. The results are saved
% in .mat file and passed to other part of the work.

% the work is written in two code files since the DQN required toolbox is
% in 2019a and this part of the code is written with MATLAB 2018a. The next
% part is written in comaptible version. 
%%
clear
close all
clc
% specigy the map to be used
map='Rtk_Pnp';
crntdir=pwd;  % save the current directory path
addpath([crntdir,'\utility'])

FogTr=200;
simTime=1500;              % simulation time
trnsmsnRange = 100; % vehicle transmision range
% create directory to store the results
d=num2str(clock);
d(isspace(d))=[];
if ~exist('Results','dir')
    mkdir('Results')
end
dirname=['Results/',d,'_',map,'_',num2str(trnsmsnRange)];
mkdir(dirname)
%%
import traci.constants
% Get the filename of the example scenario
[scenarioPath,~,~] = fileparts(which(mfilename));
% scenarioPath = [scenarioPath '\inter_palmas'];
cd([scenarioPath,'\',map]);
%% SUMO interfacing initialiation
disp('1. SUMO starting and collecting vehicle information...')
traci.start('sumo-gui -c ./Configuration.sumocfg --start');
traci.gui.setZoom('View #0',350)
traci.gui.setSchema('View #0','real world')
mapBoundary = traci.gui.getBoundary('View #0');
LaneIDlist = traci.lane.getIDList;  % get the lane ids on the map
% screenshot save
traci.gui.screenshot('View #0', 'Network ScreenShot.bmp')
%% FOG nodes placement
% road has two lane on both sides, so each lane segment has two types of co-ordinates for these
% two lanes: -ve coord and +ve coord. We remove the -ve coord as we are
% placing the FOG nodes on one side of the road.
cnt=1;laneShape=[];
for ii=1:numel(LaneIDlist)   % get the lanes ID and their length
    temp = strsplit(LaneIDlist{ii},'-');
    if size(temp)==1
        Singlelane(cnt) = LaneIDlist(ii);  % lane IDs
        SinglaneLength(cnt) = traci.lane.getLength(LaneIDlist{ii}); % lane length
        laneShape=[laneShape,(traci.lane.getShape(LaneIDlist{ii}))]; % lane co-ordinates
        cnt=cnt+1;
    end
end
% plot the single line road map in MATLAB
figure
for ii=1:numel(Singlelane)
    laneShape1=cell2mat([traci.lane.getShape(Singlelane{ii})]);
    tt(:,1) = laneShape1(1:2:end);
    tt(:,2) = laneShape1(2:2:end);
    plot(tt(:,1),tt(:,2))
    clear tt
    hold on
end
%save the image
saveimage_publish([crntdir,'/',dirname,'/','City Map'])

%% FOG nodes placement on road
distt=0;cnt=1;fogcnt=1;flag=0;laneindx=1;   % initialize the variables
for ii=1:numel(SinglaneLength)   % loop for all lane segements
    if flag==1       % update loop index if flag==1
        ii=laneindx+1;
        laneindex=ii;
    end
    if SinglaneLength(ii)<FogTr   % place the FOG node at the start of the lane if lane length is less than the FOG transmission range
        distt = SinglaneLength(ii);
        while distt<FogTr    % loop to check if next lanes and current lane lengths are less than FogTr
            if (ii+cnt)==numel(SinglaneLength) || (ii+cnt)>numel(SinglaneLength) % check to stop at matrix bounds violation
                lanecord=cell2mat([traci.lane.getShape(Singlelane{ii})]);
                FoG(fogcnt).location = lanecord(1:2);  % if condition is violated, place the FOG at current lane's start point
                FoG(fogcnt).laneID = ii; % store the lane ID too, This ID is cusotm made
                FoG(fogcnt).ActuallaneID = Singlelane{ii}; % actual lane ID
                FoG(fogcnt).ID = fogcnt;
                break
            end
            distt=distt+SinglaneLength(ii+cnt);% get the next lane length and add into previous lanes' length
            cnt=cnt+1;
            
        end
        if cnt>2  % this check is used to skip the lanes whose length are added with previosu lanes because their segment is less than FogTr
            flag=1;
        else
            flag=0;
        end
        laneindx = ii+cnt-2;  % the lane number of last lane which is added till dist<FogTr
        lanecord=cell2mat([traci.lane.getShape(Singlelane{ii})]);
        FoG(fogcnt).location = lanecord(1:2);  % place the node at the start lane's initial coord
        FoG(fogcnt).laneID = ii; % store the lane ID too, This ID is cusotm made
        FoG(fogcnt).ActuallaneID = Singlelane{ii}; % actual lane ID
        FoG(fogcnt).ID = fogcnt;
        fogcnt=fogcnt+1;cnt=1;
    else      % if lane length >FogTr
        corod=[];
        Fogno = round(SinglaneLength(ii)/FogTr);  % calculate the numebr of FOG nodes require
        lanecord=cell2mat([traci.lane.getShape(Singlelane{ii})]);   % lane segments
        corod(:,1) = lanecord(1:2:end);corod(:,2) = lanecord(2:2:end);
        
        dist=0;
        for kk=1:size(corod,1)-1 % divide the complete lane into segments and check the positon for FOG placemrent
            dist = [dist+norm(corod(kk,:)-corod(kk+1,:))];
            if dist>FogTr
                ang = getAngle(corod(kk,:),corod(kk+1,:)); %
                if (corod(kk,1)< corod(kk+1,1))&& (corod(kk,2)<corod(kk+1,2))
                    FoG(fogcnt).location(1) = corod(kk,1)+FogTr*cosd(ang);
                    FoG(fogcnt).location(2) = corod(kk,2)+FogTr*sind(ang);
                    %             elseif ang >90 && ang<180
                elseif (corod(kk,1)> corod(kk+1,1))&& (corod(kk,2)<corod(kk+1,2))
                    FoG(fogcnt).location(1) = corod(kk,1)+FogTr*cosd(ang);
                    FoG(fogcnt).location(2) = corod(kk,2)+FogTr*sind(ang);
                    %             elseif ang>180 && ang<270 || ang ==180
                elseif (corod(kk,1)> corod(kk+1,1))&& (corod(kk,2)>corod(kk+1,2))
                    if kk==2
                        if ang<90
                            ang=ang+180;
                        elseif ang>90 && ang<270
                            ang=ang+90;
                        elseif ang>270 && ang<360
                            ang=ang+180;
                        end
                    end
  
                    FoG(fogcnt).location(1) = corod(kk,1)+FogTr*cosd(ang);
                    FoG(fogcnt).location(2) = corod(kk,2)+FogTr*sind(ang);
                else
                    FoG(fogcnt).location(1) = corod(kk,1)+FogTr*cosd(ang);
                    FoG(fogcnt).location(2) = corod(kk,2)+FogTr*sind(ang);
                end
                
                FoG(fogcnt).laneID = ii; % store the lane ID too, This ID is cusotm made
                FoG(fogcnt).ActuallaneID = Singlelane{ii}; % actual lane ID
                FoG(fogcnt).ID = fogcnt;
                fogcnt=fogcnt+1;
                dist = 0;
            end
        end


        laneindx=laneindx+1;
    end
end
tem = FoG;
%% check which FOG nodes are in circular transmission range with each other
for ii=1:numel(FoG)
    for jj=1:numel(FoG)
        distfog(ii,jj) = norm(FoG(ii).location-FoG(jj).location);
         if distfog(ii,jj)<FogTr
             FoGinRange(ii,jj) = 1;
         end
    end
end
%% those FOG nodes which are in the transmission range of others and from different lane are removed
FoG=tem;
for ii = 1:numel(FoG)
    if ~isempty(FoG(ii).location)
    nearbyFogs = find(FoGinRange(ii,:));
    

    for kk=2:numel(nearbyFogs)
        if FoG(nearbyFogs(kk-1)).laneID~=FoG(nearbyFogs(kk)).laneID
            FoG(nearbyFogs(kk)).location=[];
             
        end
      
    end

    end
end
fogloc = vertcat(FoG.location);
plot(fogloc(:,1),fogloc(:,2),'^')
% save the image
saveimage_publish([crntdir,'/',dirname,'/','Map with FOG nodes'])
%% recording of vehicle parameters at each simiulation time step
initialVhclID=[];
initialPos=[];
for i = 1: simTime   % simulation time
    traci.simulation.step();
    vehicleID=traci.vehicle.getIDList();  % get vehicle's ID on the road
    
    if ~isempty(vehicleID)
        for vhcl=1:numel(vehicleID) % loop for all vehicles at any instant on the road
            detected{i,vhcl}.Location = traci.vehicle.getPosition(vehicleID{vhcl}); % measure vehcile position
            detected{i,vhcl}.speedY = traci.vehicle.getSpeed(vehicleID{vhcl}); % speed
            %             detected{i,vhcl}.speedX = traci.vehicle.getLateralSpeed(vehicleID{vhcl});
            detected{i,vhcl}.SpeedDeviation = traci.vehicle.getSpeedDeviation(vehicleID{vhcl}); % speed change
            detected{i,vhcl}.vehicleID = str2double(vehicleID{vhcl}); % each vehicle ID
            detected{i,vhcl}.angle = traci.vehicle.getAngle(vehicleID{vhcl}); % angle of the vehicle
            % check the fog node in the vicinity of the vehicle
            [dist_temp,connect_temp]=arrayfun(@(x) fastStructFogVclDist(x,detected{i,vhcl}.Location,...
                                                 trnsmsnRange), FoG,'UniformOutput',false);
            connect_fogVhcl = cellfun(@(x) x~=inf&&x~=0,connect_temp);
            tempIndx = find(connect_fogVhcl);
            dist_fogVhcl = cell2mat(dist_temp(tempIndx));
            [~,ind] = min(dist_fogVhcl);
            detected{i,vhcl}.FogID = tempIndx(ind);  % vehicle connected to FOG ID
            % store the vehicle's ID appearing very first time only with
            % their positions
            if ~ismember(str2double(vehicleID{vhcl}),initialVhclID)
                initialVhclID=[initialVhclID,str2double(vehicleID{vhcl})];
                initialPos=[initialPos;detected{i,vhcl}.Location];
            end
        end
        
    end
    
end
traci.close()
cd(crntdir)
disp('Finished.')
%% VANET processing
disp('2. Vehicle Data Processing...')
for ii=1:size(detected,1)
    if exist('distance','var')
        clear distance
    end
    if exist('inrange','var')
        clear inrange
    end
    % check for the empty cells at each simulaiton time
    emptyCells = cellfun(@isempty,detected(ii,:));
    % calculate the distance and inrange neighbors
    for rows=1:numel(detected(ii,~emptyCells))
        cnt=1;
        for cols=1:numel(detected(ii,~emptyCells))
            if rows~=cols
                distance(rows,cols)=pdist([detected{ii,rows}.Location;detected{ii,cols}.Location]);
                if distance(rows,cols)<=trnsmsnRange
                    %                         inrange(rows,cols)=1;
                    detected{ii,rows}.neighbrID(cnt)=(detected{ii,cols}.vehicleID);
                    detected{ii,rows}.neighbrPos(cnt,:)=detected{ii,cols}.Location;
                    detected{ii,rows}.neighbrDistance(cnt)=distance(rows,cols);
                    detected{ii,rows}.neighbrspeedY(cnt)=detected{ii,cols}.speedY;
                    detected{ii,rows}.neighbrangle(cnt)=detected{ii,cols}.angle;
                    cnt=cnt+1;
                end
                
            end
        end
    end
    
end
disp('Finished.')
