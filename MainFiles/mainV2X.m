function [simValues,outputValues,appParams,simParams,phyParams,outParams] = mainV2X(appParams,simParams,phyParams,outParams,simValues,outputValues,positionManagement)
% Core function where events are sorted and executed
        
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

%% Initialization
[appParams,simParams,phyParams,outParams,simValues,outputValues,...
    sinrManagement,timeManagement,positionManagement,stationManagement] = mainInit(appParams,simParams,phyParams,outParams,simValues,outputValues,positionManagement);

% The simulation starts at time '0'
timeManagement.timeNow = 0;

% The variable 'timeNextPrint' is used only for printing purposes
timeNextPrint = 0;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Simulation Cycle
% The simulation ends when the time exceeds the duration of the simulation
% (not really used, since a break inside the cycle will stop the simulation
% earlier)

% Start stopwatch
tic

fprintf('Simulation Time: ');
reverseStr = '';

while timeManagement.timeNow < simParams.simulationTime

    % The instant and node of the next event is obtained
    % indexEvent is the index of the vector IDvehicle
    % idEvent is the ID of the vehicle of the current event
    [timeEvent, indexEvent] = min(timeManagement.timeNextEvent(stationManagement.activeIDs));
    idEvent = stationManagement.activeIDs(indexEvent);

    % If the next LTE event is earlier than timeEvent, set the time to the
    % LTE event
    if timeEvent >= timeManagement.timeNextLTE
        timeEvent = timeManagement.timeNextLTE;
    end

    % If timeEvent is later than the next position update, set the time
    % to the position update
    if timeEvent >= timeManagement.timeNextPosUpdate
        timeEvent = timeManagement.timeNextPosUpdate;
    end
    
    % The time instant is updated
    % If the time instant exceeds or is equal to the duration of the
    % simulation, the simulation is ended
    timeManagement.timeNow = timeEvent;
    if round(timeManagement.timeNow*1e10)/1e10>=round(simParams.simulationTime*1e10)/1e10
        break;
    end

    %%
    % Print time to video
    while timeManagement.timeNow>timeNextPrint
        reverseStr = printUpdateToVideo(timeManagement.timeNow,simParams.simulationTime,reverseStr);
        timeNextPrint = timeNextPrint + appParams.averageTbeacon;        
    end
    %%
    
    %% Action
    % The action at timeManagement.timeNow depends on the selected event
 
    % POSITION UPDATE: positions of vehicles are updated
    if timeEvent==timeManagement.timeNextPosUpdate        
        % DEBUG EVENTS
        %printDebugEvents(timeEvent,'position update',-1);
            
        [appParams,simParams,phyParams,outParams,simValues,outputValues,timeManagement,positionManagement,sinrManagement,stationManagement] = ...
              mainPositionUpdate(appParams,simParams,phyParams,outParams,simValues,outputValues,timeManagement,positionManagement,sinrManagement,stationManagement);
        
        % DEBUG IMAGE
        % printDebugImage('position update',timeManagement,stationManagement,positionManagement,simParams,simValues);

        % CASE LTE
    elseif timeEvent == timeManagement.timeNextLTE

        if timeManagement.subframeLTEstarts
            % DEBUG EVENTS
            %printDebugEvents(timeEvent,'LTE subframe starts',-1);
            
            [sinrManagement,stationManagement,timeManagement] = ...
                mainLTEsubframeStarts(appParams,phyParams,timeManagement,sinrManagement,stationManagement,simParams,simValues);

            % DEBUG TX-RX
            %if isfield(stationManagement,'IDvehicleTXLTE') && ~isempty(stationManagement.transmittingIDsLTE)
            %    printDebugTxRx(timeManagement.timeNow,'LTE subframe starts',stationManagement,sinrManagement);
            %end

            % DEBUG TX
            printDebugTx(timeManagement.timeNow,true,-1,stationManagement,positionManagement,sinrManagement,outParams,phyParams);

            timeManagement.subframeLTEstarts = false;
            timeManagement.timeNextLTE = timeManagement.timeNextLTE + (phyParams.Tsf - phyParams.TsfGap);

            % DEBUG IMAGE
            %if isfield(stationManagement,'IDvehicleTXLTE') && ~isempty(stationManagement.transmittingIDsLTE)
            %    printDebugImage('LTE subframe starts',timeManagement,stationManagement,positionManagement,simParams,simValues);
            %end
        else
            % DEBUG EVENTS
            %printDebugEvents(timeEvent,'LTE subframe ends',-1);

            [phyParams,simValues,outputValues,sinrManagement,stationManagement,timeManagement] = ...
                mainLTEsubframeEnds(appParams,simParams,phyParams,outParams,simValues,outputValues,timeManagement,positionManagement,sinrManagement,stationManagement);

            % DEBUG TX-RX
            %if isfield(stationManagement,'IDvehicleTXLTE') && ~isempty(stationManagement.transmittingIDsLTE)
            %    printDebugTxRx(timeManagement.timeNow,'LTE subframe ends',stationManagement,sinrManagement);
            %end

            timeManagement.subframeLTEstarts = true;
            timeManagement.timeNextLTE = timeManagement.timeNextLTE + phyParams.TsfGap;

            % DEBUG IMAGE
            %if isfield(stationManagement,'IDvehicleTXLTE') && ~isempty(stationManagement.transmittingIDsLTE)
            %    printDebugImage('LTE subframe ends',timeManagement,stationManagement,positionManagement,simParams,simValues);
            %end
        end
     
        % CASE A: new packet is generated
    elseif timeEvent == timeManagement.timeNextPacket(idEvent)
        
        if stationManagement.vehicleState(idEvent)==100 % is LTE
            % DEBUG EVENTS
            %printDebugEvents(timeEvent,'New packet, LTE',idEvent);
            
            % DEBUG IMAGE
            %printDebugImage('New packet LTE',timeManagement,stationManagement,positionManagement,simParams,simValues);
        else % is not LTE
            % DEBUG EVENTS
            %printDebugEvents(timeEvent,'New packet, 11p',idEvent);
            
            % In the case of 11p, some processing is necessary
            [timeManagement,stationManagement,outputValues] = ...
                newPacketIn11p(timeEvent,idEvent,indexEvent,outParams,simParams,positionManagement,phyParams,timeManagement,stationManagement,sinrManagement,outputValues,appParams);
   
            % DEBUG TX-RX
            %printDebugTxRx(timeManagement.timeNow,'11p tx started',stationManagement,sinrManagement);
            %printDebugBackoff11p(timeManagement.timeNow,'11p backoff started',idEvent,stationManagement)

            % DEBUG IMAGE
            %printDebugImage('New packet 11p',timeManagement,stationManagement,positionManagement,simParams,simValues);
        end

        timeManagement.timeNextPacket(idEvent) = timeManagement.timeNow + timeManagement.beaconPeriod(idEvent);
        timeManagement.timeLastPacket(idEvent) = timeManagement.timeNow;
         
        % CASE B+C: either a backoff or a transmission concludes
    else % txrxevent-11p
        % A backoff ends
        if stationManagement.vehicleState(idEvent)==2 % END backoff
            % DEBUG EVENTS
            %printDebugEvents(timeEvent,'backoff concluded, tx start',idEvent);
            
            [timeManagement,stationManagement,sinrManagement] = ...
                endOfBackoff11p(idEvent,indexEvent,simParams,simValues,phyParams,timeManagement,stationManagement,sinrManagement,appParams);
 
            % DEBUG TX-RX
            %printDebugTxRx(timeManagement.timeNow,'11p tx started',stationManagement,sinrManagement);
            %printDebugBackoff11p(timeManagement.timeNow,'11p tx started',idEvent,stationManagement)
 
            % DEBUG TX
            printDebugTx(timeManagement.timeNow,true,idEvent,stationManagement,positionManagement,sinrManagement,outParams,phyParams);
            
            % DEBUG IMAGE
            %printDebugImage('11p TX starts',timeManagement,stationManagement,positionManagement,simParams,simValues);
 
            % A transmission ends
        elseif stationManagement.vehicleState(idEvent)==3 % END tx
            % DEBUG EVENTS
            %printDebugEvents(timeEvent,'Tx concluded',idEvent);
            
            [simValues,outputValues,timeManagement,stationManagement,sinrManagement] = ...
                endOfTransmission11p(idEvent,indexEvent,positionManagement,phyParams,outParams,simParams,simValues,outputValues,timeManagement,stationManagement,sinrManagement,appParams);
            
            % DEBUG IMAGE
            %printDebugImage('11p TX ends',timeManagement,stationManagement,positionManagement,simParams,simValues);

            % DEBUG TX-RX
            %printDebugTxRx(timeManagement.timeNow,'11p tx ended',stationManagement,sinrManagement);
            %printDebugBackoff11p(timeManagement.timeNow,'11p tx ended',idEvent,stationManagement)

        else
            fprintf('idEvent=%d, state=%d\n',idEvent,stationManagement.vehicleState(idEvent));
            error('Ends unknown event...')
        end
    end
    
    % The next event is selected as the minimum of all values in 'timeNextPacket'
    % and 'timeNextTxRx'
    timeManagement.timeNextEvent = min(timeManagement.timeNextPacket,timeManagement.timeNextTxRx11p);
    if min(timeManagement.timeNextEvent(stationManagement.activeIDs))<timeManagement.timeNow % error check
        fprintf('next=%f, now=%f\n',min(timeManagement.timeNextEvent(stationManagement.activeIDs)),timeManagement.timeNow);
        error('An event is schedule in the past...');
    end
    
end

% Print end of simulation
msg = sprintf('%.1f / %.1fs',simParams.simulationTime,simParams.simulationTime);
fprintf([reverseStr, msg]);

% Number of position updates
simValues.snapshots = positionManagement.NposUpdates;

% Stop stopwatch
outputValues.computationTime = toc;

end
