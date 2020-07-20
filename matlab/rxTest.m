

function rxTest
    clear global
    global Fs adcData adcData2

    Fs=1200000; % sample rate of ADC is 300 kHz
    loFreq = 150000;
    numValues = 2048;

    vna = LeanVNA;
    vna.openConnection();   
    vna.enterRawMode();
    
    vna.selectPath(3);
    vna.Si5351TxPower(1);
    vna.Si5351RxPower(1);
    vna.setFrequency(140E6);
    vna.setGain(3);
    pause()

    figure(1)
    clf
    
    j=1;  
    x=0;
    sinTable = vna.generateSinTable(Fs,numValues,loFreq); 
    adcData2 = zeros(3,numValues);
    while(1)
        vna.collectData(numValues);
        adcData = vna.readADC(numValues*3);
        adcData2(1,:) = adcData(1:numValues);
        adcData2(2,:) = adcData(1*numValues+1:2*numValues);
        adcData2(3,:) = adcData(2*numValues+1:3*numValues);
        
        adcData2(1:3,:) = kaiser(length(adcData2),5)'.*adcData2(1:3,:);
        amplitude = vna.calculateIFAmplitude(adcData2(1:3,:),sinTable);
        data(1:3,j)=abs(amplitude)/32768;

        x(j)=j;
        if(j>200)
            x1=x(j-200:j);
            data1=data(:,j-200:j);
            xmin=j-200;
            xmax=j;
        else
            x1=x;
            data1 = data;
            xmin=0;
            xmax=200;
        end
        figure(1)
        plot(x1,20*log10(data1(1,:)))
        hold on
        plot(x1,20*log10(data1(2,:)));
        plot(x1,20*log10(data1(3,:)));
        hold off
        axis([xmin xmax -110 10]);
        drawnow;
        j=j+1;
    end
end







