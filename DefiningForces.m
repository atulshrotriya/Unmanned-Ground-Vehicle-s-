function DefiningForces
%work area
wa=[0:0.1:13;0:0.1:13];
%locations of obstacles and target
T=[10,10];
obs=[3,4;8,5];
n=length(wa(1,:));
%gains
kT=3;
ko=[4,5];

for r=1:n
    for s=1:n
        FT(s,r)=kT;
        Fo1(s,r)=-ko(1,1)/(((wa(1,r)-obs(1,1))^2)+(wa(2,s)-obs(1,2))^2);
        Fo2(s,r)=-ko(1,2)/(((wa(1,r)-obs(2,1))^2)+(wa(2,s)-obs(2,2))^2);
        F=FT+Fo1+Fo2;
    end
end

for t=1:n
    for u=1:n
        FTx(u,t)=kT*(wa(1,t)-T(1,1))/(sqrt(((wa(1,t)-T(1,1))^2)+(wa(2,u)-T(1,2))^2));
        Fo1x(u,t)=-ko(1,1)*(wa(1,t)-obs(1,1))/((((wa(1,t)-obs(1,1))^2)+(wa(2,u)-obs(1,2))^2)^1.5);
        Fo2x(u,t)=-ko(1,2)*(wa(1,t)-obs(2,1))/((((wa(1,t)-obs(2,1))^2)+(wa(2,u)-obs(2,2))^2)^1.5);
        Fx=FTx+Fo1x+Fo2x;
    end
end

for a=1:n
    for b=1:n
        FTy(b,a)=kT*(wa(2,b)-T(1,2))/(sqrt(((wa(1,a)-T(1,1))^2)+(wa(2,b)-T(1,2))^2));
        Fo1y(b,a)=-ko(1,1)*(wa(2,b)-obs(1,2))/((((wa(1,a)-obs(1,1))^2)+(wa(2,b)-obs(1,2))^2)^1.5);
        Fo2y(b,a)=-ko(1,2)*(wa(2,b)-obs(2,2))/((((wa(1,a)-obs(2,1))^2)+(wa(2,b)-obs(2,2))^2)^1.5);
        Fy=FTy+Fo1y+Fo2y;
    end
end

figure(1)
mesh(wa(1,:),wa(2,:),F)
figure(2)
mesh(wa(1,:),wa(2,:),Fx)
figure(3)
mesh(wa(1,:),wa(2,:),Fy)

end