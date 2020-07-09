classdef LeanVNA  < handle
    properties
        s;
        lastFrequency=0;
    end
    methods
        function openConnection(obj)
            if ~isempty(instrfind) 
                fclose(instrfind); 
                delete(instrfind);
            end    
            obj.s=serialport('COM7',1000000);
            write(obj.s,0,"uint8")
            write(obj.s,0x0d,"uint8")
            data=read(obj.s,1,"uint8");
            if data ~= '2'
                disp 'Error'
                return
            end            
        end
        function enterRawMode(obj)
            write([obj.s],[0x20 0x26 0x01],"uint8");
        end
        function collectData(obj,i)
            write(obj.s,[0x21 0x33 typecast(uint16(i), 'uint8')],"uint8")  % samplesPerPhase
        end
        function adf4350Power(obj,i)
            write(obj.s,[0x20 0x35 uint8(i-1)],"uint8");
        end
        function selectPath(obj,i)
            write(obj.s,[0x20 0x31 uint8(i-1)],"uint8");
        end
        function setGain(obj,i)
            write(obj.s,[0x20 0x32 uint8(i)],"uint8")  % set gain
        end
        function clearFifo(obj)
            write(obj.s,[0x20 0x30 0],"uint8") %clear fifo
            flush(obj.s)
        end
        function setFrequency(obj,f)
            % there is a bug in adf4350_set(), it needs to be executed 2 times
            % after powerup (relevant for frequencies over 140 MHz)            
            frequency = uint64(f);
            disp("freq: " + int2str(f));
            write(obj.s,[0x23 0x0 typecast(frequency, 'uint8')],'uint8');
            if(obj.lastFrequency <= 140E6 & f > 140E6)
                write(obj.s,[0x23 0x0 typecast(frequency, 'uint8')],'uint8');
            end
            obj.lastFrequency=frequency;
        end    
        function a = calculateIFAmplitudeFFT(obj,adcValues,Fs,loFreq)
            n = length(adcValues);
            Y=zeros(3,n);
            Y(1,:)=fft(adcValues(1,:))/n;
            Y(2,:)=fft(adcValues(2,:))/n;
            Y(3,:)=fft(adcValues(3,:))/n;
            P2=Y;
            P1 = P2(:,1:n/2+1);
            P1(:,2:end-1) = 2*P1(:,2:end-1);

            % take amplitude at intermediate frequency loFreq
            ifIndex = round(loFreq/(Fs/n))+2; % +2 improves result
            a = P1(:,ifIndex); 
            % dont know why the complex conjugate has to be taken for the
            % result
            a = conj(a);
        end

        function a = calculateIFAmplitude(obj,adcValues,sinTable)
            n = length(adcValues);
            a = sinTable(2,:)*adcValues' / n + i*sinTable(1,:)*adcValues' / n;
        end

        function sinTable = generateSinTable(obj,Fs,n,lo)
            n_period = Fs/lo;
            sinTable=0;
            for i=1:n
                sinTable(1,i) = sin(i*2*pi/n_period);
                sinTable(2,i) = cos(i*2*pi/n_period);
            end
        end

        function data = readADC(obj,n)
            write(obj.s,[0x18 0x31 uint8(n/64)],"uint8"); % read 255 values from adc, each values 2 bytes
            adcVals = zeros(1,n*2);
            %disp("waiting...")
            adcVals = read(obj.s,n*2,"uint8");
            %disp("data received.")

            data = zeros(1,n);
            for k = 1:n
                data(k)=adcVals((k-1)*2+2)*255 + adcVals((k-1)*2 + 1);
            end
            data = data*16 - 32768;    
        end

        % values taken from main2.cpp of nanoVNA V2
        function adjustRxGain(obj,f)
            FREQUENCY_CHANGE_OVER = 140000000; % taken from common.hpp of nanoVNA V2
            if f > 2500000000
                obj.setGain(2)
            elseif f > FREQUENCY_CHANGE_OVER
                obj.setGain(1)
            else
                obj.setGain(0)
            end
        end     
        % values from main2.cpp of nanoVNA V2
        function scale = deviceS21Correction(obj,f)
            scale = 0.5;
            if f > 1900000000
                x = (f - 1900000000) / (4400000000 - 1900000000);
                scale = scale * (1 - 0.8*x*(2 - x));
            end
        end        
    end
end





