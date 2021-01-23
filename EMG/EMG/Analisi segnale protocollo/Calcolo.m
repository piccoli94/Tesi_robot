
% for i=1:length(Tempo)
%     stringa_iniziale= string(Tempo(i));
%     ore=double(extractBefore(stringa_iniziale,':'))*3600*1000;
%     b=extractAfter(stringa_iniziale,':');
%     minuti=double(extractBefore(b,':'))*60*1000;
%     c=extractAfter(b,':');
%     secondi=double(extractBefore(c,'.'))*1000;
%     millesimi=double(extractAfter(c,'.'));
%     val_tot(i)=ore+minuti+secondi+millesimi;
%     val_tot(i)=val_tot(i)-val_tot(1);
%     i
% end
clear all
close all
load('Protocollo_Data.mat')

div1=3420;
div2=8400;

div_w=256;

sig1=Ampiezza(1:div1);
sig2=Ampiezza(div1+1:div2);
sig3=Ampiezza(div2+1:end);
%%
wind1=zeros(1,div_w);
k=1;
z=0;
for i=1:length(sig1)
    wind1(k,i-z)=sig1(i);
    if(i==div_w*k) 
        k=k+1;
        z=i;
    end
end

figure(2)
hold on
for i=1:k
    plot(wind1(i,:))
end
hold off

wind_full1=zeros(1,length(sig1));
j=1;
for i=1:length(sig1)
    wind_full1(j,i)=sig1(i);
    if (i==div_w*j)
        j=j+1;
    end
end
figure(3)
hold on
for i=1:j
    plot(wind_full1(i,:))
end
hold off

for i=1:k                        %rms
    RMS(i)=rms(wind1(i,:));
    t_max=find(wind_full1(i,:)==max(wind_full1(i,:)));
    window(i)=t_max(end);
end
figure(4)
stem(window,RMS)
title('windowed EMG signal RMS')
xlabel('windows')
ylabel('Amplidtude')

   
mean_full=mean(sig1)
for k=1:length(window)                  %media
  mv(k)=mean(wind1(k,:));
end
 figure(5)
 stem(window,mv)
title('EMG Mean Value')
xlabel('Window')
ylabel('Amplidtude')

for k=1:length(window)                  %deviazione standard
  deviation(k)=std(wind1(k,:));
end
 figure(6)
 stem(window,deviation)
 title('EMG Standard deviation')
xlabel('Window')
ylabel('Amplidtude')

 for k=1:length(window)                 %IAV            
    IAV(k)=sum(abs(wind1(k,:)))/div_w;
end

figure(7)
stem(window,IAV);
title('EMG IAV')
xlabel('Window')
ylabel('Amplidtude')


thre_mean=40;
for i=1:k                        % full signal with rms thresh
    if (mv(i)<thre_mean)
        wind_full1(i,:)= 0;
    end         
end
figure(8)
hold on
for i=1:k
    plot(wind_full1(i,:))
end
hold off
title('EMG with Mean=40 Threshold')

%%


close all
wind2=zeros(1,div_w);
k=1;
z=0;
for i=1:length(sig2)
    wind2(k,i-z)=sig2(i);
    if(i==div_w*k) 
        k=k+1;
        z=i;
    end
end

figure(2)
hold on
for i=1:k
    plot(wind2(i,:))
end
hold off

wind_full2=zeros(1,length(sig2));
j=1;
for i=1:length(sig2)
    wind_full2(j,i)=sig2(i);
    if (i==div_w*j)
        j=j+1;
    end
end
figure(3)
hold on
for i=1:j
    plot(wind_full2(i,:))
end
hold off

for i=1:k                        %rms
    RMS(i)=rms(wind2(i,:));
    t_max=find(wind_full2(i,:)==max(wind_full2(i,:)));
    window(i)=t_max(end);
end
figure(4)
stem(window,RMS)
title('windowed EMG signal RMS')
xlabel('windows')
ylabel('Amplidtude')


    
mean_full=mean(sig2)
for k=1:length(window)                  %media
  mv(k)=mean(wind2(k,:));
end
 figure(5)
 stem(window,mv)
title('EMG Mean Value')
xlabel('Window')
ylabel('Amplidtude')

for k=1:length(window)                  %deviazione standard
  deviation(k)=std(wind2(k,:));
end
 figure(6)
 stem(window,deviation)
 title('EMG Standard deviation')
xlabel('Window')
ylabel('Amplidtude')

 for k=1:length(window)                 %IAV            
    IAV(k)=sum(abs(wind2(k,:)))/div_w;
end

figure(7)
stem(window,IAV);
title('EMG IAV')
xlabel('Window')
ylabel('Amplidtude')


thre_mean=40;
for i=1:k                        % full signal with mean thresh
    if (mv(i)<thre_mean)
        wind_full2(i,:)= 0;
    end         
end
figure(8)
hold on
for i=1:k
    plot(wind_full2(i,:))
end
hold off
title('EMG with Mean=40 Threshold')

%%


close all
wind3=zeros(1,div_w);
k=1;
z=0;
for i=1:length(sig3)
    wind3(k,i-z)=sig3(i);
    if(i==div_w*k) 
        k=k+1;
        z=i;
    end
end

figure(2)
hold on
for i=1:k
    plot(wind3(i,:))
end
hold off

wind_full3=zeros(1,length(sig3));
j=1;
for i=1:length(sig3)
    wind_full3(j,i)=sig3(i);
    if (i==div_w*j)
        j=j+1;
    end
end
figure(3)
hold on
for i=1:j
    plot(wind_full3(i,:))
end
hold off

window=zeros(1,k);
RMS=zeros(1,k);
for i=1:k                        %rms
    RMS(i)=rms(wind3(i,:));
    t_max=find(wind_full3(i,:)==max(wind_full3(i,:)));
    window(i)=t_max(end);
end
figure(4)
stem(window,RMS)
title('windowed EMG signal RMS')
xlabel('windows')
ylabel('Amplidtude')


mv=zeros(1,k);   
mean_full=mean(sig3)
for k=1:length(window)                  %media
  mv(k)=mean(wind3(k,:));
end
 figure(5)
 stem(window,mv)
title('EMG Mean Value')
xlabel('Window')
ylabel('Amplidtude')

deviation=zeros(1,k);
for k=1:length(window)                  %deviazione standard
  deviation(k)=std(wind3(k,:));
end
 figure(6)
 stem(window,deviation)
 title('EMG Standard deviation')
xlabel('Window')
ylabel('Amplidtude')

IAV=zeros(1,k);
 for k=1:length(window)                 %IAV            
    IAV(k)=sum(abs(wind3(k,:)))/div_w;
end

figure(7)
stem(window,IAV);
title('EMG IAV')
xlabel('Window')
ylabel('Amplidtude')

thre_mean=40;
for i=1:k                        % full signal with mean thresh
    if (mv(i)<thre_mean)
        wind_full3(i,:)= 0;
    end         
end
figure(8)
hold on
for i=1:k
    plot(wind_full3(i,:))
end
hold off
title('EMG with Mean=40 Threshold')
