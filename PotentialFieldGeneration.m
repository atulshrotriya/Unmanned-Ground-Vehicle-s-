function PotentialFieldGeneration
%work area
wa=[0:0.1:13;0:0.1:13];
%locations of obstacles and target
T=[10,10];
obs=[3,4;8,5];
n=length(wa(1,:));
%gains
kT=3;
ko=[4,5];

for p=1:n
    for q=1:n
        VT(q,p)=kT*sqrt(((wa(1,p)-T(1,1))^2)+(wa(2,q)-T(1,2))^2);
        Vo1(q,p)=ko(1,1)/sqrt(((wa(1,p)-obs(1,1))^2)+(wa(2,q)-obs(1,2))^2);
        Vo2(q,p)=ko(1,2)/sqrt(((wa(1,p)-obs(2,1))^2)+(wa(2,q)-obs(2,2))^2);
        V=VT+Vo1+Vo2;
    end
end
figure
mesh(wa(1,:),wa(2,:),V)

figure
contour(wa(1,:),wa(2,:),V)
end