function [timeManagement,stationManagement,sinrManagement,Nreassign] = BRreassignment3GPPmode4(timeManagement,stationManagement,positionManagement,sinrManagement,simParams,phyParams,appParams,outParams)
% Sensing-based autonomous resource reselection algorithm (3GPP MODE 4)
% as from 3GPP TS 36.321 and TS 36.213
% Resources are allocated for a Resource Reselection Period (SPS)
% Sensing is performed in the last 1 second
% Map of the received power and selection of the best 20% transmission hypothesis
% Random selection of one of the M best candidates
% The selection is rescheduled after a random period, with random
% probability controlled by the input parameter 'probResKeep'

% Number of TTIs per beacon period
NbeaconsT = appParams.NbeaconsT;
% Number of possible beacon resources in one TTI
NbeaconsF = appParams.NbeaconsF;
% Number of beacons per beacon period
Nbeacons = NbeaconsT*NbeaconsF;

% Calculate current T within the NbeaconsT
currentT = mod(timeManagement.elapsedTime_subframes-1,NbeaconsT)+1; 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% FIRST PART is checking the stations that need reselection

% LTE vehicles that are active
activeIdsLTE = stationManagement.activeIDsLTE;

% 1: check if a reselection is commanded by the PHY layer - i.e., in the case 
% the resource is not available in the interval T1-T2
% 2: check if (a) the reselection counter goes to zero and (b) reselection is
% commanded depending on p_keep

%% 1 - check reallocation commanded due to non available resource
% identify those vehicles having a new packet and no scheduled resource in
% the next T1-T2 interval
subframeNextResource = ceil(stationManagement.BRid/appParams.NbeaconsF);
subframesToNextAlloc = (subframeNextResource>currentT).*(subframeNextResource-currentT)+(subframeNextResource<=currentT).*(subframeNextResource+appParams.NbeaconsT-currentT);
scheduledID_PHY = activeIdsLTE(timeManagement.timeLastPacket(activeIdsLTE) > timeManagement.timeNow-phyParams.Tsf-1e-8 & (subframesToNextAlloc(activeIdsLTE) < simParams.subframeT1Mode4 | subframesToNextAlloc(activeIdsLTE)>simParams.subframeT2Mode4));
scheduledID_PHY(timeManagement.timeLastPacket(scheduledID_PHY)<0) = [];

% Following line for debug purposes - allows to remove PHY commanded reallocations
%scheduledID_PHY = [];

%% 2a - reselection counter to 0
% Evaluate which vehicles have the counter reaching zero

% LTE vehicles that have a resource allocated in this subframe
haveResourceThisTbeacon = -1*ones(length(subframeNextResource),1);
haveResourceThisTbeacon(activeIdsLTE) = (subframeNextResource(activeIdsLTE)==currentT);
% Update of next allocation for the vehicles that have a resource allocated
% in this subframe
% TODO to modify 'appParams.averageTbeacon' into something like
% 'LTE_allocation_period'

% timeManagement.timeOfResourceAllocationLTE is for possible future use
%timeManagement.timeOfResourceAllocationLTE(haveResourceThisTbeacon>0) = timeManagement.timeOfResourceAllocationLTE(haveResourceThisTbeacon>0) + appParams.averageTbeacon;

% Update resReselectionCounter
% Reduce the counter by one to all those that have a packet generated in
% this subframe
stationManagement.resReselectionCounterLTE(activeIdsLTE) = stationManagement.resReselectionCounterLTE(activeIdsLTE)-haveResourceThisTbeacon(activeIdsLTE);
% Among them, those that have reached 0 need to perform reselection
% Calculate IDs of vehicles which perform reselection
scheduledID_MAC = find (stationManagement.resReselectionCounterLTE==0);

% FOR DEBUG
% fid = fopen('temp.xls','a');
% for i=1:length(resReselectionCounter)
%     fprintf(fid,'%d\t',resReselectionCounter(i));
% end
% fprintf(fid,'\n');
% fclose(fid);

%% For the nodes with the counter reaching zero or with enforced reselction restart the reselection counter
% Calculate new resReselectionCounter for scheduledID
needReselectionCounterRestart = union(scheduledID_PHY,scheduledID_MAC);
stationManagement.resReselectionCounterLTE(needReselectionCounterRestart) = (simParams.minRandValueMode4-1) + randi((simParams.maxRandValueMode4-simParams.minRandValueMode4)+1,1,length(needReselectionCounterRestart));

%% 2b - p_keep check
% For the nodes with the counter reaching zero, check if reselection should be performed based on p_keep
% For those that have the counter reaching 0, a random variable should be drawn
% to define if the resource is kept or not, based on the input parameter probResKeep
if simParams.probResKeep>0
    keepRand = rand(1,length(scheduledID_MAC));
    % Update the vehicles which perform reselection
    scheduledID_MAC = scheduledID_MAC(keepRand >= simParams.probResKeep);
end
% else all vehicles with the counter reaching zero perform the reselection

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% SECOND PART is performing the reselection
% Merge the scheduled IDs
scheduledID = union(scheduledID_PHY,scheduledID_MAC);
Nscheduled = length(scheduledID);

% Reset number of successfully reassigned vehicles
Nreassign = 0;
for indexSensingV = 1:Nscheduled
    
    % Select the sensing matrix only for those vehicles that perform reallocation
    % and calculate the average of the measured power over the sensing window
    sensingMatrixScheduled = sum(stationManagement.sensingMatrixLTE(:,:,scheduledID(indexSensingV)),1)/length(stationManagement.sensingMatrixLTE(:,1,1));
    % "sensingMatrixScheduled" is a '1 x NbeaconIntervals' vector
    
    % With intrafrequency coexistence, any coexistence method
    % (simParams.coexMethod==1,2,3,6) might forbid LTE using some subframes
    if simParams.technology==4 && simParams.coexMethod~=0
        %if simParams.coexMethod==1 || simParams.coexMethod==2 || simParams.coexMethod==3 || simParams.coexMethod==6
        %MBest = ceil(Nbeacons * (sinrManagement.coex_NtsLTE(activeIdsLTE(indexSensingV))/simParams.coex_superframeSF) * simParams.ratioSelectedMode4);
        for block = 1:ceil(NbeaconsT/simParams.coex_superframeSF)
            sensingMatrixScheduled(...
                (block-1)*simParams.coex_superframeSF*NbeaconsF + ...
                ((((sinrManagement.coex_NtsLTE(activeIdsLTE(indexSensingV)))*NbeaconsF)+1):(simParams.coex_superframeSF*NbeaconsF))...
                ) = inf;
        end            
    end
    
    % Check T1 and T2 and in case set the subframes that are not acceptable to
    % Since the currentT can be at any point of beacon resource matrix,
    % the calculations depend on where T1 and T2 are placed
    % Note: phyParams.TsfGap is needed in the calculation because this
    % function is performed before the gap and not at the end of the
    % subframe
    timeStartingT = timeManagement.timeLastPacket(scheduledID(indexSensingV));
    startingT = mod(floor((timeStartingT+phyParams.TsfGap+1e-7)/phyParams.Tsf),NbeaconsT)+1; 
    % IF Both T1 and T2 are within this beacon period
    if (startingT+simParams.subframeT2Mode4+1)<=NbeaconsT
        sensingMatrixScheduled([1:((startingT+simParams.subframeT1Mode4-1)*NbeaconsF),((startingT+simParams.subframeT2Mode4)*NbeaconsF+1):Nbeacons]) = inf;
    % IF Both are beyond this beacon period
    elseif (startingT+simParams.subframeT1Mode4-1)>NbeaconsT
        sensingMatrixScheduled([1:((startingT+simParams.subframeT1Mode4-1-NbeaconsT)*NbeaconsF),((startingT+simParams.subframeT2Mode4-NbeaconsT)*NbeaconsF+1):Nbeacons]) = inf;
    % IF T1 within, T2 beyond
    else
        sensingMatrixScheduled(((startingT+simParams.subframeT2Mode4-NbeaconsT)*NbeaconsF+1):((startingT+simParams.subframeT1Mode4-1)*NbeaconsF)) = inf;
    end 
    
%     figure(1)
%     hold off
%     bar(isinf(sensingMatrixScheduled))
%     hold on

    % The best 20% (parameter that can be changed) is selected inside the pool as in TS 36.213
    % The pool of available resources is obtained as those that are not set
    % to infty
    nPossibleAllocations = sum(isfinite(sensingMatrixScheduled));
    MBest = ceil(nPossibleAllocations * simParams.ratioSelectedMode4);
    if MBest<=0
        error('Mbest must be a positive scalar (it is %d)',MBest);
    end

    % The knownUsedMatrix of the scheduled users is obtained
    knownUsedMatrixScheduled = stationManagement.knownUsedMatrixLTE(:,scheduledID(indexSensingV))';

    % Create random permutation of the column indexes of sensingMatrix in
    % order to avoid the ascending order on the indexes of cells with the
    % same value (sort effect) -> minimize the probability of choosing the same
    % resource
    rpMatrix = randperm(Nbeacons);

    % Build matrix made of random permutations of the column indexes
    % Permute sensing matrix
    sensingMatrixPerm = sensingMatrixScheduled(rpMatrix);
    knownUsedMatrixPerm = knownUsedMatrixScheduled(rpMatrix);

    % Now perform sorting and relocation taking into account the threshold on RSRP
    % Please note that the sensed power is on a per MHz resource basis,
    % whereas simParams.powerThresholdMode4 is on a resource element (15 kHz) basis, 
    % The cycle is stopped internally; a max of 100 is used to avoid
    % infinite loops in case of bugs
    powerThreshold = simParams.powerThresholdMode4;
    while powerThreshold < 100
        % If the number of acceptable BRs is lower than MBest,
        % powerThreshold is increased by 3 dB
        usableBRs = ((sensingMatrixPerm*0.015)<powerThreshold) | ((sensingMatrixPerm<inf) & (knownUsedMatrixPerm<1));
        if sum(usableBRs) < MBest
            powerThreshold = powerThreshold * 2;
        else
            break;
        end
    end        
    
    % To mark unacceptable RB as occupied, their power is set to Inf
    sensingMatrixPerm = sensingMatrixPerm + (1-usableBRs) * max(phyParams.P_ERP_MHz_LTE);
    
    % Sort sensingMatrix in ascending order
    [~, bestBRPerm] = sort(sensingMatrixPerm);

    % Reorder bestBRid matrix
    bestBR = rpMatrix(bestBRPerm);

    % Keep the best M canditates
    bestBR = bestBR(1:MBest);

    % Reassign, selecting a random BR among the bestBR
    BRindex = randi(MBest);
    BR = bestBR(BRindex);
	printDebugReallocation(timeManagement.timeNow,scheduledID(indexSensingV),positionManagement.XvehicleReal(stationManagement.activeIDs==scheduledID(indexSensingV)),'reall',BR,outParams);
    
    stationManagement.BRid(scheduledID(indexSensingV))=BR;
    Nreassign = Nreassign + 1;
    
    printDebugBRofMode4(timeManagement,scheduledID(indexSensingV),BR,outParams);
end

% % % Optional: if it is assumed that the reselction counter is sent in the SCI          
if simParams.lteSCIinEmptyResource
    % % Reduce the knownUsedMatrix by 1 (not a problem if it goes below 0) for
    % % those vehicles that have checked in this subframe if it is time to change
    % % allocation
    % % NOTE: the function repmat(V,n,m) creates n copies in the 1st dimension 
    % % and m copies in the 2nd dimension of the vector V
    stationManagement.knownUsedMatrixLTE = stationManagement.knownUsedMatrixLTE - repmat(haveResourceThisTbeacon',length(stationManagement.knownUsedMatrixLTE(:,1)),1);
    % IF this is used - the knownUsedMatrix should not be reset in the sensing
    % procedure
end

end

