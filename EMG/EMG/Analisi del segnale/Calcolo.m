clc
clear all
close all
load('Data')



%%
signal=Data; %segnale
leng=length(signal); %lunghezza segnale
threshold=75;   %soglia minima per accettare il segnale(utile per evitare disturbi quando si è inattivi)

figure(1)
plot(signal)
title('EMG signal')
xlabel('Sample')
ylabel('Amplidtude')


%%
for i=1:leng
   signal_thresh(i)= signal(i);           %taglio la parte del segnale al di sotto della soglia 
   if(signal_thresh(i)<threshold)
     signal_thresh(i)=0;
   end
end

figure(2)
plot(signal_thresh)
title('EMG signal with threshold')
xlabel('Sample')
ylabel('Amplidtude')



%%
signal_windowed=zeros(1,leng);                                 %finestro il segnale in modo da considerare solo una contrazione alla volta
k=1;                                                           %Problema: Bisogna scegliere una soglia adeguata
signal_windowed(k,1)= signal_thresh(1);
for i=2:leng
   signal_windowed(k,i)= signal_thresh(i);
   if(not(signal_windowed(k,i-1)==0) && signal_windowed(k,i)==0)
       k=k+1;
   end
end


figure(3)
hold on
for j=1:k
    plot(signal_windowed(j,:))
end
hold off
title('EMG signal windowed')
xlabel('Sample')
ylabel('Amplidtude')


%%
                   %valutato il valore RMS di ogni finestra
for i=1:k
    RMS(i)=rms(signal_windowed(i,:));
    t_max=find(signal_windowed(i,:)==max(signal_windowed(i,:)));
    window(i)=t_max(end);
end

figure(4)
stem(window,RMS)
title('windowed EMG signal RMS')
xlabel('windows')
ylabel('Amplidtude')


%%
Fs=2000;                                       %studio in frequenza
for i=1:k
spectrum_sig(i,:)= fft(signal_windowed(1,:)) ;  
psd_sig_temp(i,:) = (1/(Fs*leng)) * abs(spectrum_sig(i,:)).^2;
psd_sig(i,:)=psd_sig_temp(i,1:leng/2+1);
f = Fs*(0:(leng/2))/leng;
end

figure(5)
hold on
for i=1:k
plot(f,(psd_sig(i,:)))
end
hold off                                         %RISULTA CHE TUTTE LE FINESTRE HANNO LA STESSA DENSITà SPETTRALE
title('windowed EMG signal PSD')
xlabel('Sample')
ylabel('Amplidtude')


%%
% Dimitrov’s Spectral Fatigue Index
fmin=0; fmax=500;                       %Fmin dovrebbe essere di 5hz, ma la maggio parte dell'energia del segnale si concentra in 0 hz; 
for j=2:length(f)                  
    if(f(j-1)<=fmin && f(j)>=fmin)       %cerco i limiti dell'integrale
        f1=j;
    end
    if(f(j-1)<fmax && f(j)>=fmax)
        f2=j;
    end
end

for i=1:k                                              %frequenza media
Fmean(i)=sum(f(f1:f2).*psd_sig(i,f1:f2))/sum(psd_sig(i,f1:f2));
end
Fmean

for i=1:k                                         
    peak(i)=max(psd_sig(i,:)); %picchi massimi
    mean_pow(i)=sum(psd_sig(i,:))/length(psd_sig(i,:));
end
peak
mean_pow


order=2;
for i=1:k
M_order(i)=sum(f(f1:f2).^order.*psd_sig(i,f1:f2));     %calcolo il momento spettrale di ordine generico
M1(i)=sum(f(f1:f2).^(-1).*psd_sig(i,f1:f2));           %calcolo il momento spettrale di ordine -1
DI(i)=M1(i)/M_order(i);                                %calcolo l'indice di fatica spettrale di dimitrov
end
DI
%RISULTA AVERE SEMPRE LO STESSO DI

%per il calcolo della fatica si prende il DI della i-esima ripetizione e lo
%si divide con quello della prima, moltiplicandola per 100;




%% Scarto medio assoluto
% for k=1:length(window)             
% for i=1:leng
%     deviation(k,i)=abs(signal_windowed(k,i)-mean(signal_windowed(k,:)))/leng;
% end
% end

% figure(6)
% hold on
% for k=1:length(window)             
% plot(deviation(k,:));
% end
% hold off
% title('windowed EMG signal deviation')
% xlabel('Sample')
% ylabel('Amplidtude')

 for k=1:length(window)
  deviation(k)=std(signal_windowed(k,:));
 end
 stem(window,deviation)
title('windowed EMG signal deviation')
xlabel('Window')
ylabel('Amplidtude')

         
for i=1:leng
    deviation_full(i)=abs(signal(i)-mean(signal))/leng;
end

figure(7)   
plot(deviation_full);
title('EMG signal deviation')
xlabel('Sample')
ylabel('Amplidtude')

for i=1:leng
    deviation_thresh(i)=abs(signal_thresh(i)-mean(signal_thresh))/leng;
end

figure(8)   
plot(deviation_thresh);
title('EMG signal deviation with threshold')
xlabel('Sample')
ylabel('Amplidtude')


%% Integral Absolute Value

for k=1:length(window)             

    IAV(k)=sum(abs(signal_windowed(k,:)))/leng;

end

figure(9)
stem(window,IAV);
title('EMG signal IAV')
xlabel('Sample')
ylabel('Amplidtude')


%% Log Detector
for i=1:length(window)
    
  log_dect(i)=sum(log(abs(signal_windowed(i,:))+0.0001))/leng;
  exp_dect(i)=exp(log_dect(i));
end

figure(10)
stem(window,exp_dect);
title('EMG signal Log Detector')
xlabel('Sample')
ylabel('Amplidtude')


%% Waveform length

for i=1:length(window)
    WL(i)=0;
    for j=1:leng-1
        WL(i)=WL(i)+ abs(signal_windowed(i,j+1)-signal_windowed(i,j));
    end
end

figure(11)
stem(window,WL);
title('EMG signal Waveform Length')
xlabel('Sample')
ylabel('Amplidtude')

%% Myopulse percentage rate
Myo_thresh=0;
for i=1:length(window)
    
    for k=1:length(signal)
        
        if(signal_windowed(i,k)>Myo_thresh)
            f(k)=1;
        else
            f(k)=0;
        end
    end
        MYOP(i)=sum(f)/leng;
end
        
figure(12)
stem(window,MYOP);
title('EMG signal Myopulse percentage rate')
xlabel('Sample')
ylabel('Amplidtude')     
%% threshold pari alla media. Caso prima contrazione
cutted_signal=signal(1:1200);
mean_sig=mean(cutted_signal);
percent=1; % 100 percent
thr_mean=zeros(1,length(cutted_signal));
for i=1:length(cutted_signal)
if(cutted_signal(i)>(mean_sig*percent)) 
    thr_mean(i)=cutted_signal(i);
end
end
figure(13)
hold on
plot(thr_mean)
plot(ones(1,length(cutted_signal))*mean_sig*percent)
hold off
title('Signal with Threshold=Mean Value')
xlabel('Sample')
ylabel('Amplitude')
legend('Signal','Mean Value')