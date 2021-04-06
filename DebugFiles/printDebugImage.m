function printDebugImage(stringTitle,timeManagement,stationManagement,positionManagement,simParams,simValues)
% Function to be used in Debug to print an image with vehicle positions+directions 
% and colors indicating technology and tx/rx

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

%return

strTime = sprintf('%s, %.6f s',stringTitle,timeManagement.timeNow);
figure1 = figure(100);
hold off
plot([-1 -1],[-1 -1]);
hold on
if simParams.roadWidth<=0 
    axis([0 simValues.Xmax -1 1]);
else
    axis([0 simValues.Xmax 0 (2*simParams.NLanes+1)*simParams.roadWidth]);
end
is11pIdleLeft = logical(simValues.direction.*(stationManagement.vehicleState==1));
plot(positionManagement.XvehicleReal(is11pIdleLeft),...
    positionManagement.YvehicleReal(is11pIdleLeft),'<k');
%title(strTime);
is11pIdleRight  = logical(~simValues.direction.*(stationManagement.vehicleState==1));
plot(positionManagement.XvehicleReal(is11pIdleRight),...
    positionManagement.YvehicleReal(is11pIdleRight),'>k');
is11pBackoffLeft = logical(simValues.direction.*(stationManagement.vehicleState==2));
plot(positionManagement.XvehicleReal(is11pBackoffLeft),...
    positionManagement.YvehicleReal(is11pBackoffLeft),'<c');
is11pBackoffRight = logical(~simValues.direction.*(stationManagement.vehicleState==2));
plot(positionManagement.XvehicleReal(is11pBackoffRight),...
    positionManagement.YvehicleReal(is11pBackoffRight),'>c');
is11pTxLeft = logical(simValues.direction.*(stationManagement.vehicleState==3));
plot(positionManagement.XvehicleReal(is11pTxLeft),...
    positionManagement.YvehicleReal(is11pTxLeft),'<b','MarkerSize',8);
is11pTxRight = logical(~simValues.direction.*(stationManagement.vehicleState==3));
plot(positionManagement.XvehicleReal(is11pTxRight),...
    positionManagement.YvehicleReal(is11pTxRight),'>b','MarkerSize',8);
is11pRXLeft = logical(simValues.direction.*(stationManagement.vehicleState==9));
plot(positionManagement.XvehicleReal(is11pRXLeft),...
    positionManagement.YvehicleReal(is11pRXLeft),'<g','MarkerSize',8);
is11pRXRight = logical(~simValues.direction.*(stationManagement.vehicleState==9));
plot(positionManagement.XvehicleReal(is11pRXRight),...
    positionManagement.YvehicleReal(is11pRXRight),'>g','MarkerSize',8);
isLTELeft = logical(simValues.direction.*(stationManagement.vehicleState==100));
plot(positionManagement.XvehicleReal(isLTELeft),...
    positionManagement.YvehicleReal(isLTELeft),'<r');
if isfield(stationManagement,'IDvehicleTXLTE') && ~isempty(stationManagement.transmittingIDsLTE)
    isLTEtxLeft = stationManagement.transmittingIDsLTE(logical(simValues.direction(stationManagement.transmittingIDsLTE)));
    plot(positionManagement.XvehicleReal(isLTEtxLeft),...
        positionManagement.YvehicleReal(isLTEtxLeft),'<m','MarkerSize',8);
end
isLTERigth = logical(~simValues.direction.*(stationManagement.vehicleState==100));
plot(positionManagement.XvehicleReal(isLTERigth),...
    positionManagement.YvehicleReal(isLTERigth),'>r');
if isfield(stationManagement,'IDvehicleTXLTE') && ~isempty(stationManagement.transmittingIDsLTE)
    isLTEtxRigth = stationManagement.transmittingIDsLTE(logical(~simValues.direction(stationManagement.transmittingIDsLTE)));
    plot(positionManagement.XvehicleReal(isLTEtxRigth),...
        positionManagement.YvehicleReal(isLTEtxRigth),'>m','MarkerSize',8);
end
movegui(figure1,'northwest');
pause(0.1)
end