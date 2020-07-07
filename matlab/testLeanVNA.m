

function testLeanVNA
    clear global
    global Fs numValues s sinTable loFreq S21

    numValues = 2048;
    Fs=300000; % sample rate of ADC is 300 kHz
    fStart = 20E6;
    fEnd = 1E9;
    nPoints = 100;

    if ~exist('transNorm','var')
        transNorm=ones(1,nPoints);
    end
    
    if ~isempty(instrfind) 
        fclose(instrfind); 
        delete(instrfind);
    end
    s = serialport('COM7',500000);
    configureTerminator(s,"CR")
    write(s,0,"uint8")
    write(s,0x0d,"uint8")
    data=read(s,1,"uint8");
    if data ~= '2'
        disp 'Error'
        return
    end
   
    enterRawMode();
    setGain(1);
    
    calculateBBGain();
        
    fig1=figure(1);
    fig2=figure(2);
    switchDescription = ["reference" "reflection" "through"];
    
    S21 = zeros(1,nPoints);
    fIndex=1;
    
    for f = fStart:(fEnd-fStart)/(nPoints-1):fEnd
        setFrequency(f)
        if f >= 100000
            loFreq = 12000;
        else
            loFreq = 6000;
        end
        sinTable = generateSinTable(Fs,numValues,loFreq);
        adcVals2 = zeros(3,numValues);
        ifAmplitude = zeros(3,1);
        figure(fig1);
        for i = 1:3

            subplot(2,3,i)
            selectPath(i);
            pause(0.1)
            clearFifo();
            flush(s);    

            adcVals2(i,:)=readADC(numValues);
            if abs(max(adcVals2(i,:))) > 30000
                disp("clipping!")
            end
            adcVals2(i,:) = kaiser(length(adcVals2),5)'.*adcVals2(i,:);
            plot(adcVals2(i,:));
            title(switchDescription(i));
            ylim([-32700 32700])

            ifAmplitude(i)=calculateIFAmplitude(adcVals2(i,:));
            subplot(2,3,i+3)
            bar(ifAmplitude(i));
            ylim([0 32000]);   
            
        end
        figure(fig2);
        S21(fIndex) = ifAmplitude(3)/ifAmplitude(1)/transNorm(fIndex);
        plot(fStart:(fEnd-fStart)/(nPoints-1):fEnd,20*log10(S21));
        ylim([-100 10]);
        ylabel('S21 (dB)')
        xlabel('f (Hz)')
        fIndex = fIndex+1;
    end    
    
    pause(1)
    frequency = uint64(100000000);
    write(s,[0x23 0x0 typecast(frequency, 'uint8')],'uint8')
end

function a = calculateIFAmplitudeFFT(adcValues)
    global Fs loFreq
    n = length(adcValues);
    Y=fft(adcValues)/n;
    P2=abs(Y);
    P1 = P2(1:n/2+1);
    P1(2:end-1) = 2*P1(2:end-1);            

    % take amplitude at intermediate frequency loFreq
    ifIndex = round(loFreq/(Fs/n))+1;
    a = P1(ifIndex) + P1(ifIndex+1); % use 2 fft bins 
end
                        
function a = calculateIFAmplitude(adcValues)
    global sinTable
    n = length(adcValues);
    P1 = sinTable(2,:)*adcValues' / n + i*sinTable(1,:)*adcValues' / n;
    a = abs(P1);
end

function sinTable = generateSinTable(Fs,n,lo)
    n_period = Fs/lo;
    sinTable=0;
    for i=1:n
        sinTable(1,i) = sin(i*2*pi/n_period);
        sinTable(2,i) = cos(i*2*pi/n_period);
    end
end

function data = readADC(n)
    global s
    write(s,[0x18 0x31 n/16],"uint8"); % read 255 values from adc, each values 2 bytes
    adcVals = zeros(1,n*2);
    adcVals = read(s,n*2,"uint8");

    data = zeros(1,n);
    for k = 1:n
        data(k)=adcVals((k-1)*2+2)*255 + adcVals((k-1)*2 + 1);
    end
    data = data*16 - 32768;    
end

function selectPath(i)
    global s
    write(s,[0x20 0x31 i-1],"uint8");
end

function setGain(i)
    global s
    write(s,[0x20 0x32 uint8(i)],"uint8")  % set gain
end

function clearFifo()
    global s
    write(s,[0x20 0x30 0],"uint8") %clear fifo
end

function enterRawMode
    global s
    write(s,[0x20 0x26 0x01],"uint8");
end

function setFrequency(f)
    global s
    frequency = uint64(f);
    disp("freq: " + int2str(f));
    write(s,[0x23 0x0 typecast(frequency, 'uint8')],'uint8');
end

function calculateBBGain 
end