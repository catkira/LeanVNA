

function testLeanVNA
    clear global
    global Fs loFreq S21 S11

    numValues = 512; % max 2048
    Fs=300000; % sample rate of ADC is 300 kHz
    nAverages = 1;
    fStart = 1E9;
    fEnd = 1.5E9;
    nPoints = 20;

    if ~exist('transNorm','var')
        transNorm=ones(1,nPoints);
    end
    
    vna = LeanVNA;
    vna.openConnection();   
    vna.enterRawMode();
    vna.setGain(1);
        
    fig1=figure(1);
    fig2=figure(2);
    fig3=figure(3);
    switchDescription = ["reference" "reflection" "through"];
    
    S21 = zeros(1,nPoints);
    S11 = zeros(1,nPoints);
    fIndex=1;
    vna.setFrequency(fStart);
    vna.clearFifo();
    
    if fEnd < 140E6
        vna.adf4350Power(0);
    else
        vna.adf4350Power(1);        
    end
    
    for f = fStart:(fEnd-fStart)/(nPoints-1):fEnd
        vna.setFrequency(f)
        pause(0.05)
        if f >= 100000
            loFreq = 12000;
        else
            loFreq = 6000;
        end
        sinTable = vna.generateSinTable(Fs,numValues,loFreq);
        vna.adjustRxGain(f)
        
        tempS21 = zeros(1,nAverages);
        tempS11 = zeros(1,nAverages);
        for k = 1:nAverages
            adcVals2 = zeros(3,numValues);
            figure(fig1);
            
            vna.collectData(numValues);
            pause(0.01) % weird glitches with all bytes being 0 happen without this wait
            adcData = vna.readADC(numValues*3);
            adcData2(1,:) = adcData(1:numValues);
            adcData2(2,:) = adcData(1*numValues+1:2*numValues);
            adcData2(3,:) = adcData(2*numValues+1:3*numValues);

            adcData2(1:3,:) = kaiser(length(adcData2),5)'.*adcData2(1:3,:);
            amplitude = vna.calculateIFAmplitude(adcData2(1:3,:),sinTable);
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
                %bar(abs(amplitude(i)));
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
            ylim([-100 10]);
            ylabel('S21 (dB)')
            xlabel('f (Hz)')
        end
        fIndex = fIndex+1;
    end    
end

function a = calculateIFAmplitudeFFT(adcValues)
    global Fs loFreq
    n = length(adcValues);
    Y=fft(adcValues)/n;
    P2=Y;
    P1 = P2(1:n/2+1);
    P1(2:end-1) = 2*P1(2:end-1);            

    % take amplitude at intermediate frequency loFreq
    ifIndex = round(loFreq/(Fs/n))+1;
    a = P1(ifIndex) + P1(ifIndex+1); % use 2 fft bins 
end
  