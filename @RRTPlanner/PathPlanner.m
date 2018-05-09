classdef PathPlanner < handle
    % PathPlanner 객체
    %   RRT 경로계획과 관련된 변수 및 함수가 포함됨
    
    properties
                                                       %  변수명                       형태                                           설명
                                                       % ===============================================================================================================
        NEcorner                                       % Environment 크기(북동)        변수(1X2)             작업공간의 크기 : 북동쪽 방향 
        SWcorner                                       % Environment 크기(남서)        변수(1X2)             작업공간의 크기 : 남서쪽 방향 
        RandomPoint                                    % 랜덤 샘플                     변수                  무작위로 추출한 sample
        EuclideanDistance                              % 노드간 거리                   변수                  무작위로 추출한 sample과 기존의 노드간의 거리
        idx                                            % 최단노드 index                변수                  RandomPoint에서 최단거리에 있는 노드의 index 
        segmentLength                                  % RRT 확장길이                  변수                  RRT 확장길이
        cost                                           % 누적 cost                     변수                  시작점에서 현재 노드까지의 누적된 cost 
        new_point                                      % 새로 결정된 포인트             배열(1X2)             RandomPoint에서 segmentLength까지 확장된 후의 point
        new_node                                       % 새로 결정된 노드               변수(1X5)             [new_point, 0, cost, idx]

    end
    
    
    
    
    
    
    methods
        function setLookAheadDistance(ctrl,veh)                                                         % 추종거리 및 추종점 설정
            ctrl.refLfw = veh.ratioLfw*veh.length;
            ctrl.posLfw = veh.hisPos(1,1:2);
        end
        
        
        function setPID(ctrl,p,i,d)                                                                     % 추력입력값 관련 PID 제어이득 설정
            ctrl.Kp = p;
            ctrl.Ki = i;
            ctrl.Kd = d;
        end
        
        
        
        
        
        
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  Draw Sample  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%          
        function DrawSample(ctrl,env,veh,sim)                                                      % 조향제어
            p=rand;    
            if p < .2
                randomPoint = end_node(1:2);
            else
                randomPoint = [...
                    (env.bound(2)-env.bound(1))*rand,...
                    (env.bound(4)-env.bound(3))*rand]; 
            end
       
        end
        
        
        
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  Node Selection  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%          
        function NearestNeighbor(ctrl,env,veh,sim)                                                      % 조향제어
            % find leaf on node that is closest to randomPoint
            EuclideanDistance = tree(:,1:2)-ones(size(tree,1),1)*randomPoint;
            [dist,idx] = min(diag(tmp*tmp'));
            cost     = tree(idx,4) + segmentLength;
            new_point = (randomPoint-tree(idx,1:2));
            new_point = tree(idx,1:2)+new_point/norm(new_point)*segmentLength;
            new_node = [new_point, 0, cost, idx];       
        end        
        
        

    end
    
end

