function sensedPower = sensedPowerLTE(stationManagement,sinrManagement,appParams,phyParams)
% This function calculates the power sensed by LTE nodes in a subframe
% The output does not include the interference from 11p nodes, if present

NbeaconsF = appParams.NbeaconsF;
sensedPower = zeros(NbeaconsF,length(stationManagement.activeIDsLTE));

if ~isempty(stationManagement.transmittingIDsLTE)

    P_RX_MHz_LTE = sinrManagement.P_RX_MHz(stationManagement.indexInActiveIDs_ofLTEnodes,stationManagement.indexInActiveIDs_ofLTEnodes);
    activeIDsLTE = stationManagement.activeIDsLTE;

    % Calculate the beacon resources used in the time and frequency domains
    % Find not assigned BRid
    idNOT = (stationManagement.BRid<=0);
    % Calculate BRidT = vector of BRid in the time domain
    %BRidT = ceil(BRid/NbeaconsF);
    %BRidT(idNOT) = -1;
    % Calculate BRidF = vector of BRid in the frequency domain
    BRidF = mod(stationManagement.BRid-1,NbeaconsF)+1;
    BRidF(idNOT) = -1;
    
    % Cycle that calculates per each vehicle the sensed power
    for indexSensingV = 1:length(activeIDsLTE)
        % Cycle per each resource (in the current subframe)
        for BRFi = 1:NbeaconsF           
            % Init the vector of received power in this beacon resource
            rxPsums_MHz = zeros(NbeaconsF,1);
            % Cycle over the vehicles transmitting in the same subframe
            for indexSensedV = (stationManagement.indexInActiveIDsOnlyLTE_OfTxLTE)'
                % Find which BRF is used by the interferer
                BRFsensedV = BRidF(activeIDsLTE(indexSensedV));
                
                % Separate all other vehicles to itself
                if activeIDsLTE(indexSensedV)~=activeIDsLTE(indexSensingV)
                    % If not itself, add the received power
                    rxPsums_MHz(BRFsensedV) = rxPsums_MHz(BRFsensedV) + P_RX_MHz_LTE(indexSensingV,indexSensedV);
                else 
                    % Including itself allows simulating full duplex devices
                    % If itself, add the Tx power multiplied by Ksi (set to inf 
                    %       if the devices are half duplex)
                    rxPsums_MHz(BRFsensedV) = rxPsums_MHz(BRFsensedV) + phyParams.Ksi*phyParams.P_ERP_MHz_LTE(activeIDsLTE(indexSensingV));
                end
            end
            % Find total received power using IBE
            % Including possible interference from 11p (note: the
            % interfering power is already calculated per BR)
            sensedPower(BRFi,indexSensingV) = phyParams.IBEmatrixData(BRFi,:)*rxPsums_MHz; 
    %         if IDvehicle(iV)==59
    %             fid = fopen('Temp.xls','a');
    %             fprintf(fid,'%d\t%d\t%d\t%e\t',elapsedTime_subframes,currentT,BRids_currentSF(BRFi),sensingMatrix(1,BRids_currentSF(BRFi)));
    %             for k = 1:length(intIndex)
    %                 fprintf(fid,'%d\t%.2f\t',IDvehicle(intIndex(k)),positionManagement.distanceReal(IDvehicle(iV),IDvehicle(intIndex(k))));
    %             end            
    %             fprintf(fid,'\n');
    %             fclose(fid);        
    %         end
        end
    end
end