
function PowerRx = PowerRX360_func(Tx, PhiRx, Dist)
PowerTx = Tx;
thetaTx = 0;
LightSpeedC=3e8;
BlueTooth=2.0e9;%hz       
Freq=BlueTooth;
Wavelength=LightSpeedC/Freq;
Distance=Dist;
TxAntennaGain=cosd(3.0*thetaTx)^2;%db
%RxAntennaGain=cosd(1.5*thetaRx)^2;%db
% if PhiRx <= -90
%     phi = PhiRx + 180;
%     if PhiRx >= 195
%          phi = -PhiRx+540;
%      end
%     if  150 <= phi && phi <= 210
%         RxAntennaGain = cosd(phi).^2; % when BW =50 degree
%         RxAntennaGain = cosd(phi)^2; % when BW =30 degree    
%     elseif 90 >= phi || 270 <= phi
%         RxAntennaGain = 0.000875;
%     else
%         RxAntennaGain = 0.00875 + cosd(phi).^4;
%     end
% 
%     PowerRx = (PowerTx * (Wavelength/(4*pi*Distance))^2 * TxAntennaGain) * RxAntennaGain;
% else
    phi = PhiRx - 180;

    if phi <= -195
         phi = phi+360;
    end
    if  -30 <= phi && phi <= 30
        %RxAntennaGain = cosd(phi).^2; % when BW =50 degree
        RxAntennaGain = cosd(phi)^2; % when BW =30 degree    
    elseif -90 >= phi || 90 <= phi
        RxAntennaGain = 0.000875;
    else
        RxAntennaGain = 0.00875 + cosd(phi).^4;
    end

    PowerRx = (PowerTx * (Wavelength/(4*pi*Distance))^2 * TxAntennaGain) * RxAntennaGain;
% end