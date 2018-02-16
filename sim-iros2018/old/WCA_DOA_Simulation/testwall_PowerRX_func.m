function PowerRx = testwall_PowerRX_func(Tx, PhiRx)
PowerTx = Tx;
thetaTx = 0;
LightSpeedC=3e8;
BlueTooth=2.0e9;%hz       
Freq=BlueTooth;
Wavelength=LightSpeedC/Freq;
TxAntennaGain=cosd(3.0*thetaTx)^2;%db
%RxAntennaGain=cosd(1.5*thetaRx)^2;%db
powerTX_watt_data = 10^(PowerTx/10);

phi = PhiRx - 90;  
%phi = PhiRx - 120; % try -120 to 120

if  -30 <= phi && phi <= 30
    %RxAntennaGain = cosd(phi).^2; % when BW =50 degree
    RxAntennaGain = cosd(phi)^2; % when BW =30 degree    
elseif -90 >= phi || 90 <= phi
    RxAntennaGain = 0.000875;
else
    RxAntennaGain = 0.00875 + cosd(phi).^4;
end

% PowerRx = (PowerTx * (Wavelength/(4*pi*Distance))^2 * TxAntennaGain) * RxAntennaGain;

PowerRx = powerTX_watt_data * RxAntennaGain;


