

function testLeanVNA
    clear global
    global S21 S11

    numValues = 2048; % max 2048
    Fs=1200000; % sample rate of ADC is 300 kHz
    nAverages = 1;
    fStart = 200E6;
    fEnd = 300E6;
    nPoints = 30;

    if ~exist('transNorm','var')
        transNorm=ones(1,nPoints);
    end
    
    vna = LeanVNA;
    vna.openConnection();   
    vna.enterRawMode();
    vna.setGain(3);
        
    fig1=figure(1);
    clf
    fig2=figure(2);
    clf
    fig3=figure(3);
    clf
    switchDescription = ["reference" "reflection" "through"];
    
    S21 = zeros(1,nPoints);
    fIndex=1;
    vna.clearFifo();
    
    if fEnd < 140E6
        vna.adf4350Power(0);
    else
        vna.adf4350Power(1);        
    end
    
    vna.Si5351RxPower(1);
    vna.Si5351TxPower(1);

    
    for f = fStart:(fEnd-fStart)/(nPoints-1):fEnd
        vna.setFrequency(f)
        pause(0.05)
        if f >= 100000
            loFreq = 150000;
        else
            loFreq = 6000;
        end
        sinTable = vna.generateSinTable(Fs,numValues,loFreq);
       % vna.adjustRxGain(f)        
        tempS21 = zeros(1,nAverages);
        tempS11 = zeros(1,nAverages);
        for k = 1:nAverages
            adcVals2 = zeros(3,numValues);
            figure(fig1);
            
            vna.collectData(numValues);
            adcData = vna.readADC(numValues*3);
            adcData2(1,:) = adcData(1:numValues);
            adcData2(2,:) = adcData(1*numValues+1:2*numValues);
            adcData2(3,:) = adcData(2*numValues+1:3*numValues);

            % windows is not necessary if sample rate is in sync with IF
            adcData2(1:3,:) = kaiser(length(adcData2),5)'.*adcData2(1:3,:);
            amplitude = vna.calculateIFAmplitude(adcData2(1:3,:),sinTable);
            %amplitude =vna.calculateIFAmplitudeFFT(adcData2(1:3,:),Fs,loFreq);
            for i = 1:3

                subplot(2,3,i)
                vna.selectPath(i);
                pause(0.1)

                if abs(max(adcVals2(i,:))) > 30000
                    disp("clipping!")
                end
                plot(adcData2(i,:));
                title(switchDescription(i));
                ylim([-32700 32700])

                subplot(2,3,i+3)
                bar(abs(amplitude(i)));
                ylim([0 32000]);   

            end
            tempS21(k) = amplitude(3)/amplitude(1)/transNorm(fIndex)*vna.deviceS21Correction(f);
            tempS11(k) = amplitude(2)/amplitude(1);
            S11(fIndex) = sum(tempS11)/k;
            S21(fIndex) = sum(tempS21)/k;
            
            figure(fig2);
            smithplot(S11);

            figure(fig3);
            plot(fStart:(fEnd-fStart)/(nPoints-1):fEnd,20*log10(abs(S21)));
            ylim([-110 10]);
            ylabel('S21 (dB)')
            xlabel('f (Hz)')
        end
        fIndex = fIndex+1;
    end    
end

  