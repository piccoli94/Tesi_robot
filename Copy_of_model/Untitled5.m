
Y_str=string(Y);

len1=length(Y_str(1,:));
len2=length(Y_str(:,2));

k=1;
for i=1:len1
    for j=1:len2
        Y(k)="Y("+j+","+i+")="+Y_str(j,i);
        k=k+1;
    end
end

writematrix(Y,'Y_b2.txt','Delimiter',';')