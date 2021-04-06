function [sinrManagement,stationManagement,timeManagement,outputValues] = ...
            mainLTEsubframeStarts(appParams,phyParams,timeManagement,sinrManagement,stationManagement,simParams,simValues,outParams,outputValues)
% an LTE subframe starts

% Compute the number of elapsed subframes (i.e., phyParams.Tsf)
timeManagement.elapsedTime_subframes = floor((timeManagement.timeNow+1e-7)/phyParams.Tsf) + 1;

% BR adopted in the time domain (i.e., TTI)
BRidT = ceil((stationManagement.BRid)/appParams.NbeaconsF);
BRidT(stationManagement.BRid<=0)=-1;

% Find IDs of vehicles that are currently transmitting
stationManagement.transmittingIDsLTE = find(BRidT == (mod((timeManagement.elapsedTime_subframes-1),appParams.NbeaconsT)+1));
% Remove those that do not have packets in the queue (occurs at the
% beginning of the simulation)
stationManagement.transmittingIDsLTE = stationManagement.transmittingIDsLTE.*(stationManagement.pckBuffer(stationManagement.transmittingIDsLTE)>0);
stationManagement.transmittingIDsLTE(stationManagement.transmittingIDsLTE==0) = [];
% The instant this packet was generated is saved - it is needed because a
% new packet might be generated during this subframe and thus overwrite the
% content of timeLastPacket - this was causing an inaccuracy in the KPIs
timeManagement.timeGeneratedPacketInTxLTE(stationManagement.transmittingIDsLTE) = timeManagement.timeLastPacket(stationManagement.transmittingIDsLTE);

if ~isempty(stationManagement.transmittingIDsLTE)     
    % Find index of vehicles that are currently transmitting
    stationManagement.indexInActiveIDsOnlyLTE_OfTxLTE = zeros(length(stationManagement.transmittingIDsLTE),1);
    stationManagement.indexInActiveIDs_OfTxLTE = zeros(length(stationManagement.transmittingIDsLTE),1);
    for ix = 1:length(stationManagement.transmittingIDsLTE)
        %A = find(stationManagement.activeIDsLTE == stationManagement.transmittingIDsLTE(ix));
        %if length(A)~=1
        %    error('X');
        %end
        stationManagement.indexInActiveIDsOnlyLTE_OfTxLTE(ix) = find(stationManagement.activeIDsLTE == stationManagement.transmittingIDsLTE(ix));
        stationManagement.indexInActiveIDs_OfTxLTE(ix) = find(stationManagement.activeIDs == stationManagement.transmittingIDsLTE(ix));
    end
end

% Initialization of the received power
[sinrManagement] = initLastPowerLTE(timeManagement,stationManagement,sinrManagement,simParams,appParams,phyParams);

% COEXISTENCE IN THE SAME BAND
if simParams.technology == 4      
    [timeManagement,stationManagement,sinrManagement,outputValues] = coexistenceAtLTEsubframeStart(timeManagement,sinrManagement,stationManagement,appParams,simParams,simValues,phyParams,outParams,outputValues);    
end
    
% Remove the packet from the queue
if ~isempty(stationManagement.transmittingIDsLTE)
    stationManagement.pckBuffer(stationManagement.transmittingIDsLTE) = stationManagement.pckBuffer(stationManagement.transmittingIDsLTE)-1;
end

