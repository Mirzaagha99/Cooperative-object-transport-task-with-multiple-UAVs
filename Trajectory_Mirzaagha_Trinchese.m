%procedura per utilizzare il workspace salvato:
%da riga 7 clear a riga 63 end 2: ESEGUI
%metti il workspace salvato nel workspace
%da 189 takeoffenu a 205 put2: ESEGUI
%da 154 tempo a 170 timetake2: ESEGUI
%da 176 a 186: esegui
clear
hold off

takeoff = [0,0,0.5];
take = [17,10,14.5];
put = [10,15,16.5];

takeoff2 = [0 20 0.5];
take2 = put;
put2 = [10 3 12.5];

travel=[takeoff take; take put; put takeoff];
travel2=[takeoff2 take2; take2 put2;put2 takeoff2];

Scenario = uavScenario("UpdateRate",100,"ReferenceLocation",[0 0 0]);

InitialPosition=[0 0 0];
InitialOrientation=[0 0 0];

platUAV = uavPlatform("UAV",Scenario, ...
                      "ReferenceFrame","NED", ...
                      "InitialPosition",InitialPosition, ...
                      "InitialOrientation",eul2quat(InitialOrientation));

platUAV2 = uavPlatform("UAV2",Scenario, ...
                      "ReferenceFrame","NED", ...
                      "InitialPosition",InitialPosition, ...
                      "InitialOrientation",eul2quat(InitialOrientation));

updateMesh(platUAV,"quadrotor",{1.2},[0 0 1],eul2tform([0 0 pi]));
updateMesh(platUAV2,"quadrotor",{1.2},[0 0 1],eul2tform([0 0 pi]));

ObstaclePositions = [2 16;5 5;6 2;15 16; 10 10;10 3; 17 10; 10 15;17 4]; 
% Locations of the obstacles

ObstacleHeight = 4;                      % Height of the obstacles
ObstaclesWidth = 1.5;                       % Width of the obstacles

for i = 1:size(ObstaclePositions,1)
    addMesh(Scenario,"polygon", ...
        {[ObstaclePositions(i,1)-ObstaclesWidth*i/7 ...
        ObstaclePositions(i,2)-ObstaclesWidth*i/7; ...
        ObstaclePositions(i,1)+ObstaclesWidth*i/7 ...
        ObstaclePositions(i,2)-ObstaclesWidth*i/7; ...
        ObstaclePositions(i,1)+ObstaclesWidth*i/7 ...
        ObstaclePositions(i,2)+ObstaclesWidth*i/7; ...
        ObstaclePositions(i,1)-ObstaclesWidth*i/7 ...
        ObstaclePositions(i,2)+ObstaclesWidth*i/7], ...
        [1.3*i ObstacleHeight*i/2]},0.651*ones(1,3));
end

meshes_vector=[];

for i=1:size(Scenario.Meshes,2)
    meshes_vector=[meshes_vector ...
        collisionMesh(Scenario.Meshes{1,i}.Vertices)];
end

a=4000; %number of internal iterations (increased if iter is increased)

delta=5; %distance for the RRT algorithm
range_goal=3; %radius around the GOAL where we check for the q_new
check_line=5; %number of points to control in a segment connecting 2 nodes

TOTAL_COORD_PATH=[];
TOTAL_COORD_PATH2=[];

%construction of the seventh order polynomial
numpts_for_segment=1000; %number of points for each segment
tf=1; %final time
lin=linspace(0,1,numpts_for_segment); %time vector

des_vel_0=0; %desired velocity at time 0
des_vel_1=0; %desired velocity at time 1
des_acc_0=0; %desired acceleration at time 1
des_acc_1=0; %desired acceleration at time 1
des_jerk_0=0; %desired jerk at time 1
des_jerk_1=0; %desired jerk at time 1

A=[ 1 0 0 0 0 0 0 0;
    1 1 1 1 1 1 1 1;
    0 1 0 0 0 0 0 0;
    0 1 2 3 4 5 6 7;
    0 0 2 0 0 0 0 0;
    0 0 2 6 12 20 30 42;
    0 0 0 6 0 0 0 0;
    0 0 0 9 24 60 120 210];

b=[0 1 des_vel_0 des_vel_1 des_acc_0 des_acc_1 des_jerk_0 des_jerk_1]';

ai=A\b;

a0=ai(1);
a1=ai(2);
a2=ai(3);
a3=ai(4);
a4=ai(5);
a5=ai(6);
a6=ai(7);
a7=ai(8);

s_t=a7*lin.^7+a6*lin.^6+a5*lin.^5+a4*lin.^4+a3*lin.^3+a2*lin.^2+a1*lin+a0;

for GENERAL=1:3
    GENERAL

    COORD_PATH=[];
    PATH=[];

    start=travel(GENERAL,1:3);
    goal=travel(GENERAL,4:6);

    [NODELIST,PATH,COORD_PATH,found]=RRT(start,goal,a,...
        meshes_vector,delta,check_line,range_goal);
    TOTAL_COORD_PATH=[TOTAL_COORD_PATH;COORD_PATH];
end

PATH_TL=[];

for i=1:size(TOTAL_COORD_PATH,1)-1
    new_piece=s_t'*(TOTAL_COORD_PATH(i+1,:)-TOTAL_COORD_PATH(i,:))+...
        TOTAL_COORD_PATH(i,:);
    PATH_TL=[PATH_TL;new_piece];
end

for GENERAL=1:3
    GENERAL

    COORD_PATH2=[];
    PATH2=[];

    start2=travel2(GENERAL,1:3);
    goal2=travel2(GENERAL,4:6);

    [NODELIST2,PATH2,COORD_PATH2,found2]=RRT(start2,goal2,a,...
        meshes_vector,delta,check_line,range_goal);
    TOTAL_COORD_PATH2=[TOTAL_COORD_PATH2;COORD_PATH2];
end

PATH_TL2=[];

for i=1:size(TOTAL_COORD_PATH2,1)-1
    new_piece=s_t'*(TOTAL_COORD_PATH2(i+1,:)-TOTAL_COORD_PATH2(i,:))+...
        TOTAL_COORD_PATH2(i,:);
    PATH_TL2=[PATH_TL2;new_piece];
end

tempo=120;

timevec=linspace(0,tempo,size(PATH_TL,1));

RowIdx = find(ismember(PATH_TL,put,'rows'));
timeput1 = timevec(RowIdx(1));

RowIdx = find(ismember(PATH_TL,take,'rows'));
timetake1 = timevec(RowIdx(1));

timevec2=linspace(timeput1,tempo+timeput1,size(PATH_TL2,1));

RowIdx = find(ismember(PATH_TL2,put2,'rows'));
timeput2 = timevec2(RowIdx(1));

RowIdx = find(ismember(PATH_TL2,take2,'rows'));
timetake2 = timevec2(RowIdx(1));

%go NED for simulink
PATH_TL=[PATH_TL(:,2) PATH_TL(:,1) PATH_TL(:,3)*(-1)];
PATH_TL2=[PATH_TL2(:,2) PATH_TL2(:,1) PATH_TL2(:,3)*(-1)];

PATH_TL_dot=diff(PATH_TL)/tempo*size(PATH_TL,1);
PATH_TL_dot=[0 0 0;PATH_TL_dot];

PATH_TL_dot_dot=diff(PATH_TL_dot)/tempo*size(PATH_TL,1);
PATH_TL_dot_dot=[0 0 0;PATH_TL_dot_dot];

PATH_TL2_dot=diff(PATH_TL2)/tempo*size(PATH_TL2,1);
PATH_TL2_dot=[0 0 0;PATH_TL2_dot];

PATH_TL2_dot_dot=diff(PATH_TL2_dot)/tempo*size(PATH_TL2,1);
PATH_TL2_dot_dot=[0 0 0;PATH_TL2_dot_dot];


%save ENU for 3d plots in matlab
takeoffenu = takeoff;
takeenu=take;
putenu =put;

takeoff2enu = takeoff2;
take2enu = take2;
put2enu = put2;

%go NED for Simulink
takeoff = [takeoff(:,2) takeoff(:,1) -takeoff(:,3)];
take = [take(:,2) take(:,1) -take(:,3)];
put = [put(:,2) put(:,1) -put(:,3)];

takeoff2 = [takeoff2(:,2) takeoff2(:,1) -takeoff2(:,3)];
take2 = put;
put2 = [put2(:,2) put2(:,1) -put2(:,3)];

%{ 
TOTAL_COORD_PATHenu=[TOTAL_COORD_PATH(:,2) TOTAL_COORD_PATH(:,1) -TOTAL_COORD_PATH(:,3)];
TOTAL_COORD_PATH2enu=[TOTAL_COORD_PATH2(:,2) TOTAL_COORD_PATH2(:,1) -TOTAL_COORD_PATH2(:,3)];
%}

%save ENU for 3d plots in matlab

TOTAL_COORD_PATHenu=TOTAL_COORD_PATH;
TOTAL_COORD_PATH2enu=TOTAL_COORD_PATH2;

%go NED for 2d plots in matlab

TOTAL_COORD_PATH=[TOTAL_COORD_PATH(:,2) TOTAL_COORD_PATH(:,1) -TOTAL_COORD_PATH(:,3)]
TOTAL_COORD_PATH2=[TOTAL_COORD_PATH2(:,2) TOTAL_COORD_PATH2(:,1) -TOTAL_COORD_PATH2(:,3)]

hold off

figure(1)
title('reference in x, y and z for the UAV1 with respect to time NED')
grid on
%plot(timevec,PATH_S);
hold on
plot(timevec,PATH_TL);
hold on
scatter(linspace(0,tempo,size(TOTAL_COORD_PATH,1)),TOTAL_COORD_PATH,'*')
legend({'x','y','z','discrete x','discrete y','discrete z'},...
    'Location','northeast')

figure(2)
title('reference in x, y and z for the UAV2 with respect to time NED')
grid on
%plot(timevec2,PATH_S2);
hold on
plot(timevec2,PATH_TL2);
hold on
scatter(linspace(timeput1,timeput1+tempo,size(TOTAL_COORD_PATH2,1)),...
    TOTAL_COORD_PATH2,'*')
legend({'x','y','z','discrete x','discrete y','discrete z'},...
    'Location','northeast')

figure(3)
title('3D path for the UAV1')
hold on
axis equal
for i=1:size(Scenario.Meshes,2)
    show(meshes_vector(i));
end
plot3(TOTAL_COORD_PATHenu(:,1),TOTAL_COORD_PATHenu(:,2),TOTAL_COORD_PATHenu(:,3),...
    '.','Color','black','MarkerSize',15) %plot the nodes
plot3(takeoffenu(1),takeoffenu(2),takeoffenu(3),'o','Color','black','MarkerSize',20,...
    'MarkerFaceColor','g')
plot3(takeenu(1),takeenu(2),takeenu(3),'o','Color','black','MarkerSize',20,...
    'MarkerFaceColor','y')
plot3(putenu(1),putenu(2),putenu(3),'o','Color','black','MarkerSize',20,...
    'MarkerFaceColor','b')
for i=1:size(TOTAL_COORD_PATH,1)-1
    line([TOTAL_COORD_PATHenu(i,1) TOTAL_COORD_PATHenu(i+1,1)],...
        [TOTAL_COORD_PATHenu(i,2) TOTAL_COORD_PATHenu(i+1,2)],...
        [TOTAL_COORD_PATHenu(i,3) TOTAL_COORD_PATHenu(i+1,3)],...
        'LineWidth',1,'Color','red')
end

figure(4)
title('3D path for the UAV2')
hold on
axis equal
for i=1:size(Scenario.Meshes,2)
    show(meshes_vector(i));
end

plot3(TOTAL_COORD_PATH2enu(:,1),TOTAL_COORD_PATH2enu(:,2),TOTAL_COORD_PATH2enu(:,3),...
    '.','Color','black','MarkerSize',15) %plot the nodes
plot3(takeoff2enu(1),takeoff2enu(2),takeoff2enu(3),'o','Color','black',...
    'MarkerSize',20,'MarkerFaceColor','g')
plot3(take2enu(1),take2enu(2),take2enu(3),'o','Color','black','MarkerSize',20,...
    'MarkerFaceColor','y')
plot3(put2enu(1),put2enu(2),put2enu(3),'o','Color','black','MarkerSize',20,...
    'MarkerFaceColor','b')
for i=1:size(TOTAL_COORD_PATH2,1)-1
    line([TOTAL_COORD_PATH2enu(i,1) TOTAL_COORD_PATH2enu(i+1,1)],...
        [TOTAL_COORD_PATH2enu(i,2) TOTAL_COORD_PATH2enu(i+1,2)],...
        [TOTAL_COORD_PATH2enu(i,3) TOTAL_COORD_PATH2enu(i+1,3)],...
        'LineWidth',1,'Color','red')
end

figure(5)
hold on
axis equal
grid on
title('Environment')

for i=1:size(Scenario.Meshes,2)
    show(meshes_vector(i));
end

figure(6)
hold on
grid on
title('reference velocity in x, y and z for the UAV1 with respect to time NED')
plot(timevec,PATH_TL_dot);
legend({'dot x','dot y','dot z'},'Location','northeast')

figure(7)
hold on
grid on
title('reference acceleration in x, y and z for the UAV1 with respect to time NED')
plot(timevec,PATH_TL_dot_dot);
legend({'ddot x','ddot y','ddot z'},'Location','northeast')

figure(8)
hold on
grid on
title('reference velocity in x, y and z for the UAV2 with respect to time NED')
plot(timevec2,PATH_TL2_dot);
legend({'dot x','dot y','dot z'},'Location','northeast')

figure(9)
hold on
grid on
title('reference acceleration in x, y and z for the UAV2 with respect to time NED')
plot(timevec2,PATH_TL2_dot_dot);
legend({'ddot x','ddot y','ddot z'},'Location','northeast')


figure(10)
hold on
axis equal
title('SCENARIO AND INTERESTING POINTS')
plot3(takeoff2enu(1),takeoff2enu(2),takeoff2enu(3),'o','Color','black',...
    'MarkerSize',10,'MarkerFaceColor','g')
plot3(take2enu(1),take2enu(2),take2enu(3),'o','Color','black','MarkerSize',10,...
    'MarkerFaceColor','y')
plot3(put2enu(1),put2enu(2),put2enu(3),'o','Color','black','MarkerSize',10,...
    'MarkerFaceColor','b')
plot3(takeoffenu(1),takeoffenu(2),takeoffenu(3),'o','Color','black','MarkerSize',10,...
    'MarkerFaceColor','r')
plot3(takeenu(1),takeenu(2),takeenu(3),'o','Color','black','MarkerSize',10,...
    'MarkerFaceColor','m')
for i=1:size(Scenario.Meshes,2)
    show(meshes_vector(i));
end

figure(11)
hold on
title('s')
plot(s_t)

function[NODELIST,PATH,COORD_PATH,found]=RRT(start,goal,a,...
    meshes_vector,delta,check_line,range_goal)

    ADJ=[1]; %Adjacency matrix at the beginning (1x1 matrix)
    NODELIST=[start]; %list of nodes and their coordinates (X,Y)
    N=1; %number of nodes at the beginning (1)
    found=0; %flag: if a connection with the GOAL node is found
    iteration=1; %INDICE ITERAZIONE GRANDE

    while (iteration <a & found==0)

        iteration=iteration+1;

        x_rand=(rand*22); %22x22x22 is the size of the environment
        y_rand=(rand*22);
        z_rand=(rand*22);
    
        q_rand=[x_rand y_rand z_rand]; %a random point in the map
    
        best=10000; %BEST EUCLIDEAN DISTANCE FOUND AT THE BEGINNING
        %it must be large, it's a minimum searching algorithm
    
        %rearching q_near
        for j=1:N %search for the closer point to q_rand in the graph
            if(norm([NODELIST(j,:)-q_rand])<best) 
                 %norm: euclidean distance
                 best=norm([NODELIST(j,:)-q_rand]);
                 q_near_index=j;
            end
        end
           
        Dx=x_rand-NODELIST(q_near_index,1); %difference along x y and z
        Dy=y_rand-NODELIST(q_near_index,2);
        Dz=z_rand-NODELIST(q_near_index,3);


        %we choose the distance between q_near and q_new
        %leng<=delta
        if(norm([NODELIST(q_near_index,:)-q_rand])<delta) 
            leng=norm([NODELIST(q_near_index,:)-q_rand]);
        else
            leng=delta; 
        end

        %the unit vector is built
        Vx=Dx/norm([NODELIST(q_near_index,:)-q_rand]);
        Vy=Dy/norm([NODELIST(q_near_index,:)-q_rand]);
        Vz=Dz/norm([NODELIST(q_near_index,:)-q_rand]);

        vnorm=norm([Vx Vy Vz]); %Always one

        %we find q_new on the segment by multiplying the unit vector with
        %the chosen leng and we add the offset given by the coordinates of
        %q_near
        q_new=[NODELIST(q_near_index,1)+leng*Vx ...
            NODELIST(q_near_index,2)+leng*Vy ...
            NODELIST(q_near_index,3)+leng*Vz];


        %we check if the q_new is in collision
        collision_ext=controlloCollisione(q_new,meshes_vector,0.5);

        %we check if the q_new is in a good place
        if(q_new(1,1)>0 & q_new(1,1)<22 ...
            & q_new(1,2)>0 & q_new(1,2)<22 ...
            & q_new(1,3)>0 & q_new(1,3)<22 ...
            & not(collision_ext))
            
            ok=1;
            %we beck a certain number of point on the segment between q_new
            %and q_near by increasing an index that is multiplied by a
            %vector that is long fraction of the distance between the 
            % two points.
            for k=1:check_line
                coord_pt=[(NODELIST(q_near_index,1)+leng/check_line*k*Vx)...
                    (NODELIST(q_near_index,2)+leng/check_line*k*Vy)...
                    (NODELIST(q_near_index,3)+leng/check_line*k*Vz)];

                collision_int=controlloCollisione(coord_pt,meshes_vector,...
                    0.2);

                if(collision_int==1)
                    ok=0;
                end
            end

            %if the point is good we add it to the tree
            if ok==1

                %we enlarge the adjacency matrix by adding a row and a col
                ADJ=[ADJ zeros(N,1)];
                ADJ=[ADJ ; zeros(1,N+1)];
               
                %we put one where is needed
                ADJ(N+1,q_near_index)=1;
                ADJ(q_near_index,N+1)=1;
                ADJ(N+1,N+1)=1;
                N=N+1;

                %we also add the coordinates of the new point to NODELIST
                NODELIST=[NODELIST;q_new];

                %if the new node is inside a sphere of radius range_goal
                %that is placed on the goal but displaced 5 meters higher,
                %we make the last connection of the thee by connecting this
                %q_new to the goal node. We enlarge the ajacency matrix, we
                %add the goal node to NODELIST and we put 1 where is needed
                %in ADJ. 
                if(norm([q_new-[goal+[0 0 4]]])<range_goal)
                    found=1; %the flag is setted to 1

                    %the GOAL node is put inside the graph
                    ADJ=[ADJ zeros(N,1)];
                    ADJ=[ADJ ; zeros(1,N+1)];
                    ADJ(N+1,N)=1;
                    ADJ(N,N+1)=1;
                    ADJ(N+1,N+1)=1;
                    N=N+1;
                    NODELIST=[NODELIST;goal];                
                end
            end           
        end
    end

    COORD_PATH=[];
    PATH=[];

    if(found==1)        
        PATH=size(NODELIST,1);
        ADJ=ADJ-eye(size(ADJ,1));
        path_not_done=1;
    
        while (path_not_done)
            ROW=ADJ(PATH(size(PATH,2)) , 1:PATH(size(PATH,2))-1);
            PATH=[PATH find(ROW==1)];
            COORD_PATH=[COORD_PATH;NODELIST(find(ROW==1),:)];
            if PATH(size(PATH,2))==1
                path_not_done=0;
            end
        end
    end
    PATH=flip(PATH);
    COORD_PATH=flip(COORD_PATH);
    COORD_PATH=[COORD_PATH;goal];
end

function collision=controlloCollisione(punto,meshes_vector,dim)
    test_sphere_collision=collisionSphere(dim);
    test_sphere_collision.Pose=trvec2tform(punto);
    collision=0;
    for i=1:size(meshes_vector,2)
        coll = checkCollision(meshes_vector(i),test_sphere_collision);
        if coll==1
            collision=1;
        end
    end
end

