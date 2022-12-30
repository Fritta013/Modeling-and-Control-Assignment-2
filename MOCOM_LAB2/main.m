%% Modelling and Control of Manipulator assignment 2: Manipulator Geometry and Direct kinematic
clc;
clear;
close all;
addpath('include');


%% 1.
% You will need to define all the model matrices, and fill the so called iTj matrices inside BuildTree() function 
% Be careful to define the z-axis coinciding with the joint rotation axis,
% and such that the positive rotation is the same as showed in the CAD model you received.
geom_model = BuildTree()

% Useful initizializations
numberOfLinks = 7;                     % number of manipulator's links.
linkType = 0; %0 because all joints are R, boolean that specifies two possible link types: Rotational, Prismatic.
bri= zeros(3,numberOfLinks);       % Basic vector of i-th link w.r.t. base
bTi = zeros(4,4,numberOfLinks);     % Trasformation matrix i-th link w.r.t. base
biTei=zeros(4,4); %Q1.2, Direct Geometry
iTj_q = zeros(4,4,numberOfLinks); % Trasformation matrix i-th link w.r.t. base USED IN Q1.2
iTj = zeros(4,4,1);
linksleng=zeros(1,7); %stores the lenghths of the links, size of array 1*7


% Initial joint configuration 
q = [1.3,1.3,1.3,1.3,1.3,1.3,1.3];

% Q1.1 and Q1.2
iTj_q = GetDirectGeometry(q, linkType, numberOfLinks);

%initialize linkNumber_i and linkNumber_j for Q1.3, we can take any random
%number between 1 and 7. However linkNumber_j should be higher than
%linkNumber_i
linkNumber_i= 3; %example
linkNumber_j= 6; %example

%Q1.3
biTei= zeros(4,4,numberOfLinks);  %vector of matrices containing the transformation matrices from link i to link i +1 for the current q
biTei=iTj_q; %because iTj_q contains all the transformation matrices from joint 0 to joint 7 with a set of q

%GetTransformationWrtBase
for i =1:numberOfLinks
   bTi= GetTransformationWrtBase(biTei, i);
end 

%GetFrameWrtFrame
%for this example we took linkNumber_i= 3 and linkNumber_j= 6
iTj_lenght= [(linkNumber_j - linkNumber_i)+1]
for i= 1:length(linkNumber_j) %the for loop is entered only once in this example because linkNumber_j is not an array
    iTj = zeros(4,4,iTj_lenght);
    iTj = GetFrameWrtFrame(linkNumber_i, linkNumber_j, biTei, bTi)
    %note: the size of iTj is (linkNumber_j -  linkNumber_i)
end


%GetBasicVectorWrtBase
for i = 1:numberOfLinks
    bri(:,i) = GetBasicVectorWrtBase(biTei ,i)
end



% Q1.4
%%1.4.1
% use plot3() and line() matlab functions. 
%Use linspace

qi = [1.3,1.3,1.3,1.3,1.3,1.3,1.3];%initial joints configuration 
qf = [2, 2, 2, 2, 2, 2, 2]; %final joints configuration
numberOfSteps =100;
qi_iTj=zeros(4,4,7); %initialize
qf_iTj=zeros(4,4,7); %initialize
%computing direct geometry from GetDirectGeometry()
qi_iTj = GetDirectGeometry(qi, linkType, numberOfLinks)
qf_iTj= GetDirectGeometry(qf, linkType, numberOfLinks)

% distance vectors from GetBasicVectorWrtBase()
qi_bri=zeros(3,7); %initialize
qf_bri=zeros(3,7); %initialize
for i = 1:numberOfLinks
    qi_bri(:,i) = GetBasicVectorWrtBase(qi_iTj ,i)
    qf_bri(:,i) = GetBasicVectorWrtBase(qf_iTj ,i)
end

for i = 1:numberOfSteps

%-------------------MOVE----------------------%
    
end
%use plot 3 for plotting the links positions
plot3(qi_bri(1,:), qi_bri(2,:), qi_bri(3,:),"red",'linewidth',2);
title('Mapping from qi(red) to qf(green)');
grid on 
hold on
plot3(qf_bri(1,:), qf_bri(2,:), qf_bri(3,:),"green",'linewidth',2);

%%1.4.2
%clear past figures 
clf reset
qi = [1.3, 0, 1.3, 1.7, 1.3, 0.8, 1.3];%initial joints configuration 
qf = [2, 0, 1.3, 1.7, 1.3, 0.8, 1.3]; %final joints configuration
numberOfSteps =100;
qi_iTj=zeros(4,4,7); %initialize
qf_iTj=zeros(4,4,7); %initialize
%computing direct geometry from GetDirectGeometry()
qi_iTj = GetDirectGeometry(qi, linkType, numberOfLinks)
qf_iTj= GetDirectGeometry(qf, linkType, numberOfLinks)

% distance vectors from GetBasicVectorWrtBase()
qi_bri=zeros(3,7); %initialize
qf_bri=zeros(3,7); %initialize
for i = 1:numberOfLinks
    qi_bri(:,i) = GetBasicVectorWrtBase(qi_iTj ,i)
    qf_bri(:,i) = GetBasicVectorWrtBase(qf_iTj ,i)
end

for i = 1:numberOfSteps

%-------------------MOVE----------------------%
    
end
%use plot 3 for plotting the links positions
plot3(qi_bri(1,:), qi_bri(2,:), qi_bri(3,:),"red",'linewidth',2);
title('Mapping qi(red)');
grid on 
hold on
plot3(qf_bri(1,:), qf_bri(2,:), qf_bri(3,:),"green",'linewidth',2);
title('Mapping qf(green)');

%%1.4.3
%clear past figures 
clf reset
qi = [1.3, 0.1, 0.1, 1, 0.2, 0.3, 1.3];%initial joints configuration 
qf = [2, 2, 2, 2, 2, 2, 2]; %final joints configuration
numberOfSteps =100;
qi_iTj=zeros(4,4,7); %initialize
qf_iTj=zeros(4,4,7); %initialize
%computing direct geometry from GetDirectGeometry()
qi_iTj = GetDirectGeometry(qi, linkType, numberOfLinks)
qf_iTj= GetDirectGeometry(qf, linkType, numberOfLinks)

% distance vectors from GetBasicVectorWrtBase()
qi_bri=zeros(3,7); %initialize
qf_bri=zeros(3,7); %initialize
for i = 1:numberOfLinks
    qi_bri(:,i) = GetBasicVectorWrtBase(qi_iTj ,i)
    qf_bri(:,i) = GetBasicVectorWrtBase(qf_iTj ,i)
end

for i = 1:numberOfSteps

%-------------------MOVE----------------------%
    
end
%use plot 3 for plotting the links positions
plot3(qi_bri(1,:), qi_bri(2,:), qi_bri(3,:),"red",'linewidth',2);
title('Mapping from qi(red) to qf(green)');
grid on 
hold on
plot3(qf_bri(1,:), qf_bri(2,:), qf_bri(3,:),"green",'linewidth',2);

%%1.5
%clear past figures 
clf reset
%defining the start configuration and the 3 changes
qstart= [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1];%initial joints configuration 
q1= [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.9]; %change 1
q2= [0.1, 0.1, 0.1, 0.5, 0.1, 0.1, 0.1]; %change 2
q3= [0.1, 0.1, 0.1, 0.1, 0.1, 0.7, 0.1]; %change 3
numberOfSteps =100;
qstart_iTj=zeros(4,4,7); %initialize
q1_iTj=zeros(4,4,7); %initialize
q2_iTj=zeros(4,4,7); %initialize
q3_iTj=zeros(4,4,7); %initialize
%computing direct geometry from GetDirectGeometry()
qstart_iTj = GetDirectGeometry(qstart, linkType, numberOfLinks)
q1_iTj= GetDirectGeometry(q1, linkType, numberOfLinks)
q2_iTj= GetDirectGeometry(q2, linkType, numberOfLinks)
q3_iTj= GetDirectGeometry(q3, linkType, numberOfLinks)

% distance vectors from GetBasicVectorWrtBase()
qstart_bri=zeros(3,7); %initialize
q1_bri=zeros(3,7); %initialize
q2_bri=zeros(3,7); %initialize
q3_bri=zeros(3,7); %initialize
for i = 1:numberOfLinks
    qstart_bri(:,i) = GetBasicVectorWrtBase(qstart_iTj ,i)
    q1_bri(:,i) = GetBasicVectorWrtBase(q1_iTj ,i)
    q2_bri(:,i) = GetBasicVectorWrtBase(q2_iTj ,i)
    q3_bri(:,i) = GetBasicVectorWrtBase(q3_iTj ,i)
end

for i = 1:numberOfSteps

%-------------------MOVE----------------------%
    
end
%use plot 3 for plotting the links positions
plot3(qstart_bri(1,:), qstart_bri(2,:), qstart_bri(3,:),"red",'linewidth',2);
title('Mapping qstart(red), q1(blue), q2(yellow), q3(green)');
grid on 
hold on
plot3(q1_bri(1,:), q1_bri(2,:), q1_bri(3,:),"blue",'linewidth',2);
grid on 
hold on
plot3(q2_bri(1,:), q2_bri(2,:), q2_bri(3,:),"yellow",'linewidth',2);
grid on 
hold on
plot3(q3_bri(1,:), q3_bri(2,:), q3_bri(3,:),"green",'linewidth',2);
grid on 
hold on
