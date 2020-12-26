load('str_div')
for i=1:7
    istr=string(i);
    param{i}=append("ml",istr);
    param{i+7}=append("mm",istr);
    param{i+14}=append("Il1_",istr);
    param{i+21}=append("Il2_",istr);
    param{i+28}=append("Il3_",istr);
    param{i+35}=append("Im",istr);
end

for j=1:length(B_str_div)
    temp=B_str_div{j};
    for k=1:length(param)
        element{j,k} = strfind(temp,param{k});
    end 
end

res=zeros(111,42,49);
for j=1:length(B_str_div)
    for k=1:length(param)
        temp=element{j,k};
        for i=1:length(temp)
            if(not(isempty(temp{i})))
                res(i,k,j)=1;                
            end
        end
        
    end 
end

cont=zeros(111,length(B_str_div));
for j=1:length(B_str_div)
   for i=1:111 
        cont(j,i)=sum(res(i,:,j));
        end
    end 

