% the matrix has following columns for tasks requirements
% 1. latency in seconds
% 2. aplication/container size in kb
% 3. the offloading request size that will be sent to the orchestrator and then
% to the device where the task will eb offloaded
% 4. results of the offloaded task in kb
% 5. task length in MI(million instructions)

% NOTE: these values are copied from the PureEdgedsim Simulator

apps = [5, 25000, 150, 100, 60000, 1;...           %augmented reality
        5, 13000, 10000, 10000, 300000, 1;...    % E-health
        5, 9000, 50, 50, 15000, 1];                %Heavy Comp apps  