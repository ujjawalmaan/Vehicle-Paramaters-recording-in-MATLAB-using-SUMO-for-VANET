clear
close all
clc
% specigy the map to be used
map='Panipat Map';
crntdir=pwd;  % save the current directory path
addpath([crntdir,'\nbr selection Parameters'])
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
% get junctions ID and position
junctionID = traci.junction.getIDList();
for ii=1:numel(junctionID)
    junctionPos(ii,:) = traci.junction.getPosition(junctionID{ii});
end

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
%% plot the single line road map in MATLAB
figure
for ii=1:numel(Singlelane) 
    laneShape1=cell2mat([traci.lane.getShape(Singlelane{ii})]);
    tt(:,1) = laneShape1(1:2:end);
    tt(:,2) = laneShape1(2:2:end);
    plot(tt(:,1),tt(:,2))
    clear tt
    hold on
end

%% place the FOG nodes
 % place the fog nodes at the all junctions
% FoG.loc(1,:)=junctionPos(1,:);
for ii=1:size(junctionPos,1)
    for jj=1:size(junctionPos,1)
    dist(ii,jj) = norm(junctionPos(ii,:)-junctionPos(jj,:));
    if dist(ii,jj)==0
        FoG.loc(ii,:)=junctionPos(ii,:);
%         plot(FoG.loc(ii,1),FoG.loc(ii,2),'^')
%         hold on
    end
    if dist(ii,jj)<200
        inrange(ii,jj)=1;
    end
    end
end
%%
for ii=1:size(junctionPos,1)
    index=find(inrange(ii,:));
    if numel(index)>1
        FoG.loc(index(2:end),:)=0;
    end
    FoG.loc(index(1),:) = junctionPos(ii,:);
     plot(FoG.loc(index(1),1),FoG.loc(index(1),2),'^')
     
        hold on
end
% plot(FoG.loc(:,1),FoG.loc(:,2),'^')
%%
tt=cell2mat(laneShape);