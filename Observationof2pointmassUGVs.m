function Observationof2pointmassUGVs
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
ktr=0.05; m=1;
init=[0;0;0;0;0.5;0;0;0];
[t,pa]=ode45(@robota,[0 10],init,options);
plot(pa(:,1),pa(:,3),pa(:,5),pa(:,7))
    function dpa=robota(t,pa)
        dpa=zeros(8,1);
        xa=pa(1);ya=pa(3); xb=pa(5); yb=pa(7);

        FTax=kT*(T(1,1)-xa)/(sqrt(((T(1,1)-xa)^2)+((T(1,2)-ya)^2)));
        Fo1ax=-ko(1,1)*(obs(1,1)-xa)/((((obs(1,1)-xa)^2)+(obs(1,2)-ya)^2)^1.5);
        Fo2ax=-ko(1,2)*(obs(2,1)-xa)/((((obs(2,1)-xa)^2)+(obs(2,2)-ya)^2)^1.5);
        Frbax=-ktr*(xb-xa)/(((xb-xa)^2+(yb-ya)^2)^1.5);
        Fxa=FTax+Fo1ax+Fo2ax+Frbax;
        
        FTay=kT*(T(1,2)-ya)/(sqrt(((T(1,1)-xa)^2)+(T(1,2)-ya)^2));
        Fo1ay=-ko(1,1)*(obs(1,2)-ya)/((((obs(1,1)-xa)^2)+(obs(1,2)-ya)^2)^1.5);
        Fo2ay=-ko(1,2)*(obs(2,2)-ya)/((((obs(2,1)-xa)^2)+(obs(2,2)-ya)^2)^1.5);
        Frbay=-ktr*(yb-ya)/(((xb-xa)^2+(yb-ya)^2)^1.5);
        Fya=FTay+Fo1ay+Fo2ay+Frbay;
        
        dpa(1)=pa(2);
        dpa(2)=Fxa/m;
        dpa(3)=pa(4);
        dpa(4)=Fya/m;
        
        FTbx=kT*(T(1,1)-xb)/(sqrt(((T(1,1)-xb)^2)+(T(1,2)-yb)^2));
        Fo1bx=-ko(1,1)*(obs(1,1)-xb)/((((obs(1,1)-xb)^2)+(obs(1,2)-yb)^2)^1.5);
        Fo2bx=-ko(1,2)*(obs(2,1)-xb)/((((obs(2,1)-xb)^2)+(obs(2,2)-yb)^2)^1.5);
        Frabx=-ktr*(xa-xb)/(((xa-xb)^2+(ya-yb)^2)^1.5);
        Fxb=FTax+Fo1ax+Fo2ax+Frabx;

        FTby=kT*(T(1,2)-ya)/(sqrt(((T(1,1)-xa)^2)+(T(1,2)-ya)^2));
        Fo1by=-ko(1,1)*(obs(1,2)-ya)/((((obs(1,1)-xa)^2)+(obs(1,2)-ya)^2)^1.5);
        Fo2by=-ko(1,2)*(obs(2,2)-ya)/((((obs(2,1)-xa)^2)+(obs(2,2)-ya)^2)^1.5);
        Fraby=-ktr*(ya-yb)/(((xa-xb)^2+(ya-yb)^2)^1.5);
        Fyb=FTay+Fo1ay+Fo2ay+Fraby;

        dpa(5)=pa(7);
        dpa(6)=Fxb/m;
        dpa(7)=pa(8);
        dpa(8)=Fyb/m;
    end
    function [Val,Ister,Dir]=StopSim(t,pa)
        Val(1)=pa(3)-T(1,1);
        Ister(1)=1;
        Dir(1)=0;
    end
end