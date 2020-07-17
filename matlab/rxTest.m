

function rxTest
    clear global
    global Fs adcData adcData2

    Fs=300000; % sample rate of ADC is 300 kHz
    numValues = 1024;

    vna = LeanVNA;
    vna.openConnection();   
    vna.enterRawMode();
    
    vna.setGain(1);
    vna.selectPath(3);
    vna.setFrequency(151.4E7);

    j=1;
    x=0;
    loFreq = 12000;
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
        axis([xmin xmax -100 10]);
        drawnow;
        j=j+1;
    end
end







