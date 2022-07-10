function drawdoubleintegrator(hax, x,xDes)
W = xDes/6;  % cart width
H = .5; % cart height
figure(3)
cla
plot(hax, [-(xDes+W/2) (xDes+W/2)],[0 0],'k','LineWidth',2)
hold on
%for i = 1:length(t)
%x = xTraj(1,i);
plot(hax, [-(xDes+W/2) (xDes+W/2)],[0 0],'k','LineWidth',2)
%hold on
rectangle('Position',[x-W/2,-H/2,W,H],'Curvature',.1)



paddingX=1.25;
% set(gca,'YTick',[])
% set(gca,'XTick',[])
xlim([0 xDes+(W/2)*paddingX]);
ylim([-2 2]);
set(gcf,'Position',[100 550 1000 400])
% box off

%dt_viz=0.1
drawnow
gif
%pause(dt_viz)
  
    
%end
hold off