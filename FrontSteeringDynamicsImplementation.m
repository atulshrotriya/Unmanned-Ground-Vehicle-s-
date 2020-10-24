function FrontSteeringDynamicsImplementation
%work area
wa=[0:0.1:13;0:0.1:13];
%locations of obstacles and target
T=[10,10];
obs=[3,4;8,5];
n=length(wa(1,:));
%gains
kT=3;
ko=[4,5];
options=odeset('events',@StopSim);
Vp=4; kn=3; L=2;
init=[0;0;pi/4]; %initial robot condition
[t,pos]=ode45(@robot,[0 100],init,options);
figure
plot(pos(:,1),pos(:,2))
xlim([0 13]);
ylim([0 13]);

VT=kT.*sqrt(((pos(:,1)-T(1,1)).^2)+(pos(:,2)-T(1,2)).^2);
Vo1=ko(1,1)./sqrt(((pos(:,1)-obs(1,1)).^2)+(pos(:,2)-obs(1,2)).^2);
Vo2=ko(1,2)./sqrt(((pos(:,1)-obs(2,1)).^2)+(pos(:,2)-obs(2,2)).^2);
V=VT+Vo1+Vo2;
mesh(pos(:,1),pos(:,2),V)
    function dp=robot(t,pos)
        dp=zeros(3,1);        
        x=pos(1);y=pos(2);theta=pos(3);
        FTx=kT*(T(1,1)-x)/(sqrt(((T(1,1)-x)^2)+((T(1,2)-y)^2)));
        Fo1x=-ko(1,1)*(obs(1,1)-x)/((((obs(1,1)-x)^2)+(obs(1,2)-y)^2)^1.5);
        Fo2x=-ko(1,2)*(obs(2,1)-x)/((((obs(2,1)-x)^2)+(obs(2,2)-y)^2)^1.5);
        Fx=FTx+Fo1x+Fo2x;
        FTy=kT*(T(1,2)-y)/(sqrt(((T(1,1)-x)^2)+(T(1,2)-y)^2));
        Fo1y=-ko(1,1)*(obs(1,2)-y)/((((obs(1,1)-x)^2)+(obs(1,2)-y)^2)^1.5);
        Fo2y=-ko(1,2)*(obs(2,2)-y)/((((obs(2,1)-x)^2)+(obs(2,2)-y)^2)^1.5);
        Fy=FTy+Fo1y+Fo2y;
        thetades=atan2(Fy,Fx);
        fi=kn*(thetades-theta);
        dp(1)=Vp*cos(fi)*cos(theta);
        dp(2)=Vp*cos(fi)*sin(theta);
        dp(3)=(Vp/L)*sin(fi);
    end
    function [Val,Ister,Dir]=StopSim(t,pos)
        Val(1)=(pos(1)-T(1,1));
        Ister(1)=1;
        Dir(1)=0;
    end 
end