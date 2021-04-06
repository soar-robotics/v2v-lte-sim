function [timeManagement,stationManagement,sinrManagement,Nreassign] = BRreassignment3GPPmode4(timeManagement,stationManagement,sinrManagement,simParams,phyParams,appParams,outParams)
% Sensing-based autonomous resource reselection algorithm (3GPP MODE 4)
% as from 3GPP TS 36.321 and TS 36.213
% Resources are allocated for a Resource Reselection Period (SPS)
% Sensing is performed in the last 1 second
% Map of the received power and selection of the best 20% transmission hypothesis
% Random selection of one of the M best candidates
% The selection is rescheduled after a random period, with random
% probability controlled by the input parameter 'probResKeep'

% ==============
% Copyright (C) Alessandro Bazzi, University of Bologna, and Alberto Zanella, CNR
% 
% All rights reserved.
% 
% Permission to use, copy, modify, and distribute this software for any 
% purpose without fee is hereby granted, provided that this entire notice 
% is included in all copies of any software which is or includes a copy or 
% modification of this software and in all copies of the supporting 
% documentation for such software.
% 
% THIS SOFTWARE IS BEING PROVIDED "AS IS", WITHOUT ANY EXPRESS OR IMPLIED 
% WARRANTY. IN PARTICULAR, NEITHER OF THE AUTHORS MAKES ANY REPRESENTATION 
% OR WARRANTY OF ANY KIND CONCERNING THE MERCHANTABILITY OF THIS SOFTWARE 
% OR ITS FITNESS FOR ANY PARTICULAR PURPOSE.
% 
% Project: LTEV2Vsim
% ==============

activeIdsLTE = stationManagement.activeIDsLTE;
subframeNextPacket = mod(ceil(timeManagement.timeNextPacket/phyParams.Tsf)-1,(appParams.NbeaconsT))+1;
NbeaconsT = appParams.NbeaconsT;
NbeaconsF = appParams.NbeaconsF;

% Calculate current T within the NbeaconsT
currentT = mod(timeManagement.elapsedTime_subframes-1,NbeaconsT)+1; 

% Number of beacons per beacon period
Nbeacons = NbeaconsT*NbeaconsF;

% The best 20% (modifiable by input) is selected as pool as in TS 36.213
% If T1==1 and T2==100, Mbest is the 20% of all beacon resources
% In the case T1>1 and/or T2<100, Mbest is the 20% of the consequent number
% of resources
MBest = ceil(Nbeacons * ((simParams.subframeT2Mode4-simParams.subframeT1Mode4+1)/100) * simParams.ratioSelectedMode4);

% Reset number of successfully reassigned vehicles
Nreassign = 0;

%% Update the sensing matrix
% The sensingMatrix is a 3D matrix with
% 1st D -> Number of values to be stored in the time domain, corresponding
%          to the standard duration of 1 second, of size ceil(1/Tbeacon)
% 2nd D -> BRid, of size Nbeacons
% 3rd D -> IDs of vehicles

% Array of BRids in the current subframe 
BRids_currentSF = ((currentT-1)*NbeaconsF+1):(currentT*NbeaconsF);

% A shift is performed to the estimations (1st dimension) corresponding 
% to the BRids in the current subframe for all vehicles
stationManagement.sensingMatrixLTE(:,BRids_currentSF,:) = circshift(stationManagement.sensingMatrixLTE(:,BRids_currentSF,:),1);

% The values in the first position of the 1st dimension (means last measurement) of
% the BRids in the current subframe are reset for all vehicles
%sensingMatrix(1,BRids_currentSF,:) = 0;
% These values will be hereafter filled with the latest measurements

% Update of the sensing matrix
if ~isempty(stationManagement.transmittingIDsLTE)   
    
    if isempty(sinrManagement.sensedPowerByLteNo11p)
                
        sensedPowerCurrentSF = sensedPowerLTE(stationManagement,sinrManagement,appParams,phyParams);
    else        
        sensedPowerCurrentSF = sinrManagement.sensedPowerByLteNo11p;
        
    end

    % If the received power measured on that resource is lower than
    % a threshold, it is assumed that no power is measured
    sensedPowerCurrentSF(sensedPowerCurrentSF<phyParams.Pnoise_MHz) = 0;

    stationManagement.sensingMatrixLTE(1,BRids_currentSF,stationManagement.activeIDsLTE) = sensedPowerCurrentSF;
    
end

% Cycle that updates per each vehicle and BR the knownUsedMatrix
%  knownUsedMatrix = zeros(appParams.Nbeacons,simValues.maxID);
if ~isempty(stationManagement.transmittingIDsLTE)     
    for i = 1:length(stationManagement.indexInActiveIDsOnlyLTE_OfTxLTE)
        idVtx = stationManagement.transmittingIDsLTE(i);
        indexVtxLte = stationManagement.indexInActiveIDsOnlyLTE_OfTxLTE(i);
        BRtx = stationManagement.BRid(idVtx);
        for indexNeighborsOfVtx = 1:length(stationManagement.neighborsIDLTE(indexVtxLte,:))
           idVrx = stationManagement.neighborsIDLTE(indexVtxLte,indexNeighborsOfVtx);
           if idVrx<=0
               break;
           end
           % IF the SCI is transmitted in this subframe AND if it is correctly
           % received AND the corresponding value of 'knownUsedMatrix' is lower
           % than what sent in the SCI (means the value is not updated)
           % THEN the corresponding value of 'knownUsedMatrix' is updated
           if stationManagement.correctSCImatrixLTE(i,indexNeighborsOfVtx) == 1 && ...
                  stationManagement.knownUsedMatrixLTE(BRtx,idVrx) < stationManagement.resReselectionCounterLTE(idVtx)
               stationManagement.knownUsedMatrixLTE(BRtx,idVrx) = stationManagement.resReselectionCounterLTE(idVtx);
           % NOTE: the SCI is here assumed to advertise the current value of the reselection
           % counter, which is an approximation of what in TS 36.213, Table 14.2.1-2
           end
        end
    end
end

%% Update the resReselectionCounter and evaluate which vehicles need reselection
% Calculate scheduledID
inTheLastSubframe = -1*ones(length(subframeNextPacket),1);
inTheLastSubframe(activeIdsLTE) = (subframeNextPacket(activeIdsLTE)==currentT);

% Update resReselectionCounter
% Reduce the counter by one to all those that have a packet generated in
% the last subframe
% Among them, those that have reached 0 need to be reset between min and max
stationManagement.resReselectionCounterLTE(activeIdsLTE) = stationManagement.resReselectionCounterLTE(activeIdsLTE)-inTheLastSubframe(activeIdsLTE);

% fid = fopen('temp.xls','a');
% for i=1:length(resReselectionCounter)
%     fprintf(fid,'%d\t',resReselectionCounter(i));
% end
% fprintf(fid,'\n');
% fclose(fid);

% Calculate IDs of vehicles which perform reselection
scheduledID = find (stationManagement.resReselectionCounterLTE==0);

% Calculate the number of vehicles which perform reselection
Nscheduled = length(scheduledID);

% Calculate new resReselectionCounter for scheduledID
stationManagement.resReselectionCounterLTE(scheduledID) = (simParams.minRandValueMode4-1) + randi((simParams.maxRandValueMode4-simParams.minRandValueMode4)+1,1,Nscheduled);

% For those that have the counter reaching 0, a random variable should be drawn
% to define if the resource is kept or not, based on the input parameter probResKeep
if simParams.probResKeep>0
    keepRand = rand(1,Nscheduled);
    scheduledID = scheduledID(keepRand >= simParams.probResKeep);
    % Update the number of vehicles which perform reselection
    Nscheduled = length(scheduledID);
end
% else all vehicles with the counter reaching zero perform the reselection

%% Perform the reselection
for indexSensingV = 1:Nscheduled
    
    % Select the sensing matrix only for those vehicles that perform reallocation
    % and calculate the average of the measured power over the sensing window
    sensingMatrixScheduled = sum(stationManagement.sensingMatrixLTE(:,:,scheduledID(indexSensingV)),1)/length(stationManagement.sensingMatrixLTE(:,1,1));
    % "sensingMatrixScheduled" is a '1 x NbeaconIntervals' vector
        
    % Check T1 and T2 and in case set the subframes that are not acceptable to
    % infinite sensed power
    if simParams.subframeT1Mode4>1 || simParams.subframeT2Mode4<100
        if NbeaconsT~=100
            error('This part is written for NbeaconsT=100. needs revision.');
        end
        % Since the currentT can be at any point of beacon resource matrix,
        % the calculations depend on where T1 and T2 are placed
        % IF Both T1 and T2 are within this beacon period
        if (currentT+simParams.subframeT2Mode4+1)<=NbeaconsT
            sensingMatrixScheduled([1:((currentT+simParams.subframeT1Mode4-1)*NbeaconsF),((currentT+simParams.subframeT2Mode4)*NbeaconsF+1):Nbeacons]) = inf;
        % IF Both are beyond this beacon period
        elseif (currentT+simParams.subframeT1Mode4-1)>NbeaconsT
            sensingMatrixScheduled([1:((currentT+simParams.subframeT1Mode4-1-NbeaconsT)*NbeaconsF),((currentT+simParams.subframeT2Mode4-NbeaconsT)*NbeaconsF+1):Nbeacons]) = inf;
        % IF T1 within, T2 beyond
        else
            sensingMatrixScheduled(((currentT+simParams.subframeT2Mode4-NbeaconsT)*NbeaconsF+1):((currentT+simParams.subframeT1Mode4-1)*NbeaconsF)) = inf;
        end 
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

    stationManagement.BRid(scheduledID(indexSensingV))=BR;
    Nreassign = Nreassign + 1;
    
    printDebugBRofMode4(timeManagement,scheduledID(indexSensingV),BR,outParams);
end

% Reduce the knownUsedMatrix by 1 (not a problem if it goes below 0) for
% those vehicles that have checked in this subframe if it is time to change
% allocation
% NOTE: the function repmat(V,n,m) creates n copies in the 1st dimension 
% and m copies in the 2nd dimension of the vector V
stationManagement.knownUsedMatrixLTE = stationManagement.knownUsedMatrixLTE - repmat(inTheLastSubframe',length(stationManagement.knownUsedMatrixLTE(:,1)),1);

% The channel busy ratio is calculated, if needed, every subframe for those
% nodes that have a packet generated in the subframe that just ended
if ~isempty(sinrManagement.cbrLTE)
    [timeManagement,stationManagement,sinrManagement] = cbrUpdateLTE(timeManagement,inTheLastSubframe,stationManagement,sinrManagement,appParams,simParams,phyParams,outParams);
end

end

