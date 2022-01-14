%%MOCOM LAB1

clc
clear
close all

addpath('include');


%initialization of the transformation matrix from link i to link i+1 for
%qi=0

biTri(1,1,1) = 1; biTri(1,2,1) = 0; biTri(1,3,1) = 0; biTri(1,4,1) = 0;
biTri(2,1,1) = 0; biTri(2,2,1) = 1; biTri(2,3,1) = 0; biTri(2,4,1) = 0;
biTri(3,1,1) = 0; biTri(3,2,1) = 0; biTri(3,3,1) = 1; biTri(3,4,1) = 0.175;
biTri(4,1,1) = 0; biTri(4,2,1) = 0; biTri(4,3,1) = 0; biTri(4,4,1) = 1;

biTri(1,1,2) = -1; biTri(1,2,2) = 0;  biTri(1,3,2) = 0; biTri(1,4,2) = 0;
biTri(2,1,2) = 0;  biTri(2,2,2) = 0;  biTri(2,3,2) = 1; biTri(2,4,2) = 0;
biTri(3,1,2) = 0;  biTri(3,2,2) = 1;  biTri(3,3,2) = 0; biTri(3,4,2) = 0.108;
biTri(4,1,2) = 0;  biTri(4,2,2) = 0;  biTri(4,3,2) = 0; biTri(4,4,2) = 1;

biTri(1,1,3) = 0;  biTri(1,2,3) = 0; biTri(1,3,3) = 1;  biTri(1,4,3) = 0.105;
biTri(2,1,3) = -1; biTri(2,2,3) = 0; biTri(2,3,3) = 0;  biTri(2,4,3) = 0;
biTri(3,1,3) = 0;  biTri(3,2,3) = -1; biTri(3,3,3) = 0; biTri(3,4,3) = 0;
biTri(4,1,3) = 0;  biTri(4,2,3) = 0; biTri(4,3,3) = 0;  biTri(4,4,3) = 1;

biTri(1,1,4) = 0;  biTri(1,2,4) = 1;  biTri(1,3,4) = 0;  biTri(1,4,4) = -0.1455;
biTri(2,1,4) = 0;  biTri(2,2,4) = 0;  biTri(2,3,4) = -1; biTri(2,4,4) = 0;
biTri(3,1,4) = -1; biTri(3,2,4) = 0;  biTri(3,3,4) = 0;  biTri(3,4,4) = 0.3265;
biTri(4,1,4) = 0;  biTri(4,2,4) = 0;  biTri(4,3,4) = 0;  biTri(4,4,4) = 1;

biTri(1,1,5) = 0; biTri(1,2,5) = 0;  biTri(1,3,5) = 1; biTri(1,4,5) = 0.095;
biTri(2,1,5) = 0; biTri(2,2,5) = -1; biTri(2,3,5) = 0; biTri(2,4,5) = 0;
biTri(3,1,5) = 1; biTri(3,2,5) = 0;  biTri(3,3,5) = 0; biTri(3,4,5) = 0;
biTri(4,1,5) = 0; biTri(4,2,5) = 0;  biTri(4,3,5) = 0; biTri(4,4,5) = 1;

biTri(1,1,6) = 0; biTri(1,2,6) = 0;  biTri(1,3,6) = 1; biTri(1,4,6) = 0;
biTri(2,1,6) = 0; biTri(2,2,6) = -1; biTri(2,3,6) = 0; biTri(2,4,6) = 0;
biTri(3,1,6) = 1; biTri(3,2,6) = 0;  biTri(3,3,6) = 0; biTri(3,4,6) = 0.325;
biTri(4,1,6) = 0; biTri(4,2,6) = 0;  biTri(4,3,6) = 0; biTri(4,4,6) = 1;

biTri(1,1,7) = 0; biTri(1,2,7) = 0;  biTri(1,3,7) = 1; biTri(1,4,7) = 0.132;
biTri(2,1,7) = 0; biTri(2,2,7) = -1; biTri(2,3,7) = 0; biTri(2,4,7) = 0;
biTri(3,1,7) = 1; biTri(3,2,7) = 0;  biTri(3,3,7) = 0; biTri(3,4,7) = 0;
biTri(4,1,7) = 0; biTri(4,2,7) = 0;  biTri(4,3,7) = 0; biTri(4,4,7) = 1;

%number of links of the manipulator
nLinks = size(biTri,3);

% in our robot all joints are revolute joint
linkType = zeros(nLinks,1);

% initialize the matrix which will contain the basic vector with respect to
% base of each joint
bri= zeros(3,nLinks);

bTi = zeros(4,4, nLinks);

linkNumber_i=[1,2,3];
linkNumber_j=[4,5,6];

iTj=zeros(4,4,1);

% chosen q configuration
q=[0.5,1,0.5,1,0.5,1,0.5];

% here we get all the model matrices for the q configuration chosen
biTei = GetDirectGeometry(q,biTri, linkType);

% with this loop and thanks to the function GetTransformationWrtBase
% we get all the transformation matrix with respect to the base
for i = 1:nLinks
    
    bTi(:,:,i)=GetTransformationWrtBase(biTei,i);
    
end 

for i=1:length(linkNumber_i)
    
    iTj(:,:,i) = GetFrameWrtFrame(linkNumber_i(i),linkNumber_j(i),biTei);
    
end

% here we get the vector of the omogeneous matrix of each joint with
% respect to the base
for i = 1:nLinks
    
    bri(:,i)= GetBasicVectorWrtBase(biTei,i);
    
end 
%-------------------MOVE----------------------%

start_position=[0,0,0,0,0,0,0];
final_position=[3,2,1,2,2,1,0];

NumberOfSteps=65;

qSteps=[linspace(start_position(1),final_position(1),NumberOfSteps)',...
    linspace(start_position(2),final_position(2),NumberOfSteps)',...
    linspace(start_position(3),final_position(3),NumberOfSteps)',...
    linspace(start_position(4),final_position(4),NumberOfSteps)',...
    linspace(start_position(5),final_position(5),NumberOfSteps)',...
    linspace(start_position(6),final_position(6),NumberOfSteps)',...
    linspace(start_position(7),final_position(7),NumberOfSteps)'];

figure

hold on
xlabel('x')
ylabel('y')
zlabel('z')
axis equal
grid on
colorbar
az = 50;
el = 25;
view(az,el);

cindex = 1;
csize = NumberOfSteps;
cmap = colormap(turbo(csize));
color = cmap(mod(cindex,csize)+1,:);

% plot a red dot in the origin of the base frame
plot3(0,0,0,'red.', 'MarkerSize', 20)

brij=zeros(3,nLinks);

for i = 1:NumberOfSteps
    
        q = qSteps(i,1:nLinks)';
        
        biTei = GetDirectGeometry(q,biTri,linkType);

    
    % Evaluate all vectors from base to link i (base_to_i)
    for j = 1:nLinks
            
        brij(:,j) = GetBasicVectorWrtBase(biTei,j);
    end
    
    % Add zero to the base_to_i vectors 
    brij= [[0;0;0],brij];
    
    % plot the links exept for the zero link (plotted outside the loop)
    for j = 2:nLinks+1
        
       plot3(brij(1,j),brij(2,j),brij(3,j),'k.', 'MarkerSize', 10)
       
    end
    
    color=cmap(mod(cindex,csize)+1,:);
    cindex = cindex + 1;
    hold on
    line(brij(1,:),brij(2,:),brij(3,:),'LineWidth',1.5,'Color',color)
    
    getframe;
end





