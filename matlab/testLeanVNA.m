

function testLeanVNA
    %fclose(instrfind);
    s = serialport('COM7',500000);
    configureTerminator(s,"CR")
    write(s,0,"uint8")
    write(s,0x0d,"uint8")
    data=read(s,1,"uint8");
    if data ~= '2'
        disp 'Error'
        return
    end
    
    write(s,[0x20 0x26 0x01],"uint8")
    
    write(s,[0x20 0x32 0x02],"uint8")  % set gain
    
    calculateBBGain();
    
    %frequency = uint64(1E9);
    %write(s,[0x23 0x0 typecast(frequency, 'uint8')],'uint8')
    
    fig1=figure(1);
    fig2=figure(2);
    switchDescription = ["reference" "reflection" "through"];
    
    fStart = 1E7;
    fEnd = 1E9;
    nPoints = 40;
    S21 = zeros(1,nPoints);
    fIndex=1;
    Fs=300000; % sample rate of ADC is 300 kHz
    numValues = 512;
    
    sinTable12000 = generateSinTable(Fs,numValues,12000);
    generateSinTable(Fs,numValues,12000);
    
    for f = fStart:(fEnd-fStart)/(nPoints-1):fEnd
        frequency = uint64(f);
        disp("freq: " + int2str(f));
        write(s,[0x23 0x0 typecast(frequency, 'uint8')],'uint8');
        if f >= 100000
            loFreq = 12000;
        else
            loFreq = 6000;
        end
        adcVals2 = zeros(3,numValues);
        ifAmplitude = zeros(3,1);
        figure(fig1);
        for i = 1:3

            subplot(2,3,i)
            write(s,[0x20 0x31 i-1],"uint8");
            pause(0.1)
            write(s,[0x20 0x30 0],"uint8") %clear fifo
            flush(s);    

            write(s,[0x18 0x31 numValues/16],"uint8"); % read 255 values from adc, each values 2 bytes
            adcVals = zeros(1,numValues*2);
            adcVals = read(s,numValues*2,"uint8");

            for k = 1:numValues
                adcVals2(i,k)=adcVals((k-1)*2+2)*255 + adcVals((k-1)*2 + 1);
            end
            adcVals2(i,:) = adcVals2(i,:)*16 - 32768;
            if abs(max(adcVals2(i,:))) > 30000
                disp("clipping!")
            end
            adcVals2(i,:) = kaiser(length(adcVals2),300)'.*adcVals2(i,:);
            plot(adcVals2(i,:));
            title(switchDescription(i));
            ylim([-32700 32700])

            %%
            % FFT
            %Y=fft(adcVals2(i,:)/numValues);
            %P2=abs(Y);
            %P1 = P2(1:numValues/2+1);
            %P1(2:end-1) = 2*P1(2:end-1);            
            
            % take amplitude at intermediate frequency 12.0 kHz
            
            %ifIndex = round(loFreq/(Fs/numValues));
            %ifAmplitude(i) = P1(ifIndex);

            %subplot(2,3,i+3)
            %P1(1)=0; % remove DC part
            %plot(0:(Fs/numValues):(Fs/2-Fs/numValues),P1(1:numValues/2))

            
            P1 = sinTable12000(2,:)*adcVals2(i,:)' / numValues^2 + i*sinTable12000(1,:)*adcVals2(i,:)' / numValues^2;
            ifAmplitude(i) = abs(P1);
            subplot(2,3,i+3)
            bar(abs(P1));
            ylim([0 2]);
            

            %%
            
        end
        figure(fig2);
        S21(fIndex) = ifAmplitude(3)/ifAmplitude(1);
        plot(fStart:(fEnd-fStart)/(nPoints-1):fEnd,20*log10(S21));
        ylim([-100 10]);
        fIndex = fIndex+1;
    end    
    
    pause(1)
    frequency = uint64(100000000);
    write(s,[0x23 0x0 typecast(frequency, 'uint8')],'uint8')
end


function sinTable = generateSinTable(Fs,n,lo)
    n_period = Fs/lo;
    sinTable=0;
    for i=1:n
        sinTable(1,i) = sin(i*2*pi/n_period);
        sinTable(2,i) = cos(i*2*pi/n_period);
    end
end

function calculateBBGain 
end