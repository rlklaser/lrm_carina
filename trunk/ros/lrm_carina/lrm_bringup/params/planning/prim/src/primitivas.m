clc
close all
clear all

T=15;
dt=0.1;
xnext=[0,0,0];
speed=[0.3,0.4,0.5];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
steer= -pi/6;

for k=1:length(speed)
    xnext=[0,0,0];   
    for i=1:(T/dt)
        next= car(speed(k),steer,xnext);
        xhist(i,:)=next;
        xnext=next;
    end
    prim1(k,:,:) = xhist;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
steer= -pi/12;

for k=1:length(speed)    
    xnext=[0,0,0];
    for i=1:(T/dt)
        next= car(speed(k),steer,xnext);
        xhist(i,:)=next;
        xnext=next;
    end
    prim2(k,:,:) = xhist;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
steer= 0;

for k=1:length(speed)
    xnext=[0,0,0];
    for i=1:(T/dt)
        next= car(speed(k),steer,xnext);
        xhist(i,:)=next;
        xnext=next;
    end
    prim3(k,:,:) = xhist;
end
 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
steer= pi/12;

for k=1:length(speed)    
    xnext=[0,0,0];
    for i=1:(T/dt)
        next= car(speed(k),steer,xnext);
        xhist(i,:)=next;
        xnext=next;
    end
    prim4(k,:,:) = xhist;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
steer= pi/6;

for k=1:length(speed)
    xnext=[0,0,0];
    for i=1:(T/dt)
        next= car(speed(k),steer,xnext);
        xhist(i,:)=next;
        xnext=next;
    end
    prim5(k,:,:) = xhist;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
cor=['--m',':bs','-.r*'];
for i=1:3
    figure(1);
    plot(prim1(i,:,1),prim1(i,:,2),'*',prim2(i,:,1),prim2(i,:,2), ...
    's',prim3(i,:,1),prim3(i,:,2),prim4(i,:,1),prim4(i,:,2),'s',prim5(i,:,1),prim5(i,:,2),'*');
    
    hold on;
    %axis([0 3 -3 3]);
    pause;
end
