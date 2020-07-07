

function rxTest
    clear global
    global Fs numValues s sinTable

    Fs=300000; % sample rate of ADC is 300 kHz
    numValues = 512;

    if ~isempty(instrfind) 
        fclose(instrfind); 
        delete(instrfind);
    end
    s = serialport('COM7',1000000);
    configureTerminator(s,"CR")
    write(s,0,"uint8")
    write(s,0x0d,"uint8")
    data=read(s,1,"uint8");
    if data ~= '2'
        disp 'Error'
        return
    end
   
    enterRawMode();
    setGain(3);
    selectPath(3);
    setFrequency(1E9);
    clearFifo();

    data=0;
    j=1;
    x=0;
    loFreq = 12000;
    sinTable = generateSinTable(Fs,numValues,loFreq);    
    while(1)
        adcData = readADC(numValues);
        clearFifo();
        adcData = kaiser(length(adcData),5)'.*adcData;
        amplitude = calculateIFAmplitude(adcData);      
        data(j)=abs(amplitude)/32768;
        x(j)=j;
        if(j>200)
            x1=x(j-200:j);
            data1=data(j-200:j);
            xmin=j-200;
            xmax=j;
        else
            x1=x;
            data1=data;
            xmin=0;
            xmax=200;
        end

        plot(x1,20*log10(data1));
        axis([xmin xmax -100 10]);
        drawnow;
        j=j+1;
    end
end

% values from main2.cpp of nanoVNA V2
function scale = deviceS21Correction(f)
    scale = 0.5;
    if f > 1900000000
        x = (f - 1900000000) / (4400000000 - 1900000000);
        scale = scale * (1 - 0.8*x*(2 - x));
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
                        
function a = calculateIFAmplitude(adcValues)
    global sinTable
    n = length(adcValues);
    a = sinTable(2,:)*adcValues' / n + i*sinTable(1,:)*adcValues' / n;
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
    write(s,[0x18 0x31 n/64],"uint8"); % read 255 values from adc, each values 2 bytes
    adcVals = zeros(1,n*2);
    %disp("waiting...")
    adcVals = read(s,n*2,"uint8");
    %disp("data received.")

    data = zeros(1,n);
    for k = 1:n
        data(k)=adcVals((k-1)*2+2)*255 + adcVals((k-1)*2 + 1);
    end
    data = data*16 - 32768;    
end

% values taken from main2.cpp of nanoVNA V2
function adjustRxGain(f)
    FREQUENCY_CHANGE_OVER = 140000000; % taken from common.hpp of nanoVNA V2
    if f > 2500000000
        setGain(2)
    elseif f > FREQUENCY_CHANGE_OVER
        setGain(1)
    else
        setGain(0)
    end
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