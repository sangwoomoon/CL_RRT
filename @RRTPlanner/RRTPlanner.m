classdef RRTPlanner < handle
    % PathPlanner 객체
    %   RRT 경로계획과 관련된 변수 및 함수가 포함됨
    
    properties
                                                       %  변수명                       형태                                           설명
                                                       % ===============================================================================================================
        p                                              % 랜덤 number (0~1)            변수                   0~1사이에 무작위로 선택된 값
        randomPoint                                    % 랜덤 샘플                     변수                  무작위로 추출한 sample
        diff                                           % 노드간 거리                   변수                  무작위로 추출한 sample과 기존의 노드간의 거리
        idx                                            % 최단노드 index                변수                  RandomPoint에서 최단거리에 있는 노드의 index 
        segmentLength                                  % RRT 확장길이                  변수                  RRT 확장길이
        cost                                           % 누적 cost                     변수                  시작점에서 현재 노드까지의 누적된 cost 
        new_point                                      % 새로 결정된 포인트             배열(1X2)             RandomPoint에서 segmentLength까지 확장된 후의 point
        new_node                                       % 새로 결정된 노드               변수(1X5)             [new_point, 0, cost, idx]
        tree                                           % RRT tree
        wpt                                            % RRT에 의해 선택된 waypoints
        goal                                           % RRT goal point
    end
    
    
    
    
     
    
    
    
    
   
    
    methods
        function setRRT(rrt,veh)                                            % RRT Parameter setting
            rrt.p = 0;                                                      % probability of goal point selection
            rrt.segmentLength = 15;                                         % RRT node extension length
            rrt.tree = [veh.Pos(1:2), 0, 0, 0];                             % RRT tree array
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % Initial Heading Consideration
            % initial vehicle heading
            rrt.tree(2,:)  = [veh.Pos(1) + rrt.segmentLength*cos(veh.Pos(3)), veh.Pos(2) + rrt.segmentLength*sin(veh.Pos(3)), 0, rrt.segmentLength, 1];
           %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            rrt.randomPoint = veh.Pos(1:2);
            rrt.diff = 0;                             
            rrt.idx  = 1;                                    
            rrt.cost = 0;                                           
            rrt.new_point  = veh.Pos(1:2);                              
            rrt.new_node   = [veh.Pos(1:2), 0, 0, 0]; 
            rrt.wpt = veh.Pos(1:2);
            rrt.goal = [200, 200];
        end
        


        
        
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  Draw Sample  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%          
        function DrawSample(rrt,env)                                        % draw sample
            rrt.p=rand;    
            if rrt.p < .2
                rrt.randomPoint = rrt.goal(1:2);
            else
                rrt.randomPoint = [...
                    (env.bound(2)-env.bound(1))*rand,...
                    (env.bound(4)-env.bound(3))*rand];      % rand is not equal to rrt.p !!
            end
       
        end
        
        
        
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  Node Selection  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%          
        function NodeSelection(rrt)                                                      % select node to extend
            % find leaf on node that is closest to randomPoint            
            rrt.diff = rrt.tree(:,1:2)-ones(size(rrt.tree,1),1)*rrt.randomPoint;
            [dist,rrt.idx] = min(diag(rrt.diff*rrt.diff'));
            
            if rrt.tree(rrt.idx,end) == 0
                 rrt.tree = rrt.tree;         
            else

                rrt.cost     = rrt.tree(rrt.idx,4) + rrt.segmentLength;     % 원래 cost에 15만큼을 더하므로 15의 배수가 됨 -> 15 만큼 떨어져가며 확장
                rrt.new_point = (rrt.randomPoint-rrt.tree(rrt.idx,1:2));
                rrt.new_point = rrt.tree(rrt.idx,1:2)+rrt.new_point/norm(rrt.new_point)*rrt.segmentLength;
                rrt.new_node = [rrt.new_point, 0, rrt.cost, rrt.idx];   

    %             dist = sqrt(dist);
    %             rrt.cost     = rrt.tree(rrt.idx,4) + dist;
    %             rrt.new_point = (rrt.randomPoint-rrt.tree(rrt.idx,1:2));
    %             rrt.new_point = rrt.tree(rrt.idx,1:2)+rrt.new_point/norm(rrt.new_point)*dist;
    %             rrt.new_node = [rrt.new_point, 0, rrt.cost, rrt.idx]; 
    
                rrt.tree = [rrt.tree; rrt.new_node]; 
            end

        end        
        
       
        
        
        
        
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  Waypoint Construction  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%         
        function findMinimumPath(rrt,end_node)                                               % construct waypoint list

           % find nodes that connect to end_node 
           connectingNodes = rrt.tree(end_node(end),:);

            % find minimum cost last node
            [tmp,idx] = min(connectingNodes(:,4));

            % construct lowest cost path
            rrt.wpt = [connectingNodes(idx,:); end_node];
            parent_node = connectingNodes(idx,end);

            if size(rrt.tree,1)==2
                rrt.wpt = [rrt.tree(1,:);end_node];
            else
                rrt.wpt = [rrt.tree(parent_node,:); rrt.wpt];
                while parent_node > 1,
                    parent_node = rrt.tree(parent_node,end);
                    rrt.wpt = [rrt.tree(parent_node,:); rrt.wpt];
                end
            end             
        end        
        
        
        
        
        
          
        
        
        
        
        
        
        
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  Tree Expansion  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%          
        function TreeExpansion(rrt,env,veh,ctrl,sim)                        % CL-RRT initial test
            
            while size(rrt.tree,1) < 30
                DrawSample(rrt,env) 
                NodeSelection(rrt) 
%                 TreeConstruction(rrt)  
                
                end_node = rrt.tree(end,:);
                
                if end_node(end)~=1                                        % 만약 1이면 그 자체로 끝나기 때문에 findMinimumPath을 할 필요가 없음
                    findMinimumPath(rrt,end_node);                         % Waypoint Selection
                    % setWptNum(env,rrt);
                    setWpt(env,rrt)
                    evaluateSim(sim, env, veh, ctrl)
                    PlotVehicleTrajectory(veh);  
                end
                
            end
        end    
        
  
        

        


    end
    
end

