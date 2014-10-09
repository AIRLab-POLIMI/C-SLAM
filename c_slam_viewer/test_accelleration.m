
global acc


for i = 1:length(acc)
    x = acc(i,3:21);
    ra = acc(i,31:33);
    sa = acc(i,34:36);
    z = acc(i,22:24);   
     
    tmp0 = x(4);
    tmp1 = x(5);
    tmp2 = x(6);
    tmp3 = x(7);
    err(i,1) = sa(1) + ra(1)*(19.6133*(-(tmp0*tmp2) + tmp1*tmp3) + x(14)) - z(1);
    err(i,2) = sa(1) + ra(2)*(19.6133*(tmp0*tmp1 + tmp2*tmp3) + x(15)) - z(2);
    err(i,3) = sa(1) + ra(3)*(9.80665*(pow(tmp0,2) - pow(tmp1,2) - pow(tmp2,2) + pow(tmp3,2)) + x(16)) - z(3);
    
    g(i,1) = ra(1)*(19.6133*(-(tmp0*tmp2) + tmp1*tmp3));
    g(i,2) = ra(2)*(19.6133*(tmp0*tmp1 + tmp2*tmp3));
    g(i,3) = ra(3)*(9.80665*(pow(tmp0,2) - pow(tmp1,2) - pow(tmp2,2) + pow(tmp3,2)));

end

plot(err)
plot(g)


%plot(acc(:,16:18))

%plot(acc(:,34:36))

plot(acc(:,22:24))

%%

plot(acc(:,6:9))
hold on
plot(x(4:7,:)')

%%
    
close all

qtest = zeros(min(size(x,2),size(acc,1)),4);
for i = 1:size(qtest,1)
    qtest(i,:) = quatprod(quatinv(x(4:7,i)),acc(i,6:9));
    if qtest(i,1) < 0
        qtest(i,1) = -qtest(i,1);
    end
end

plot(qtest)   