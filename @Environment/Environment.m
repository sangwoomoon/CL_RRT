classdef Environment < handle
    %ENVIRONMENT 객체
    %   임무를 수행하는 주변환경의 정보와 관련 함수가 포함됨
    
    properties
                               %  변수명                  형태                                           설명
                               % ===============================================================================================================
        nWaypt                 % 경로점 개수               변수                
        Waypt                  % 경로점 위치          배열({nWaypt}X2)                       [x좌표(meter), y좌표(meter)] 
        distBtwWaypt           % 경로점 사이 거리     배열({nWaypt-1}X1)       meter단위, 최종 경로점까지 진행하기위해 가야할 거리를 계산하기 위하여 사용
        
        bound                  % 필드 크기            배열(4X1)             [x축 최소값(meter),x축 최대값(meter),y축 최소값(meter),y축 최대값(meter)]
    end
    
    methods
        
        function setBound(env,input)                                                % 필드 크기 지정 : input을 받아서 필드크기를 할당
            env.bound = input;
        end
        
        function setWptNum(env,input)                                               % 경로점 개수 지정 : input을 받아서 경로점 개수를 할당
            env.nWaypt = input;
        end
        
        function dispField(env)                                                     % 필드 표시 : 할당된 필드크기를 사용하여 필드 자체만을 표시
            figure(1)
            axis([env.bound(1),env.bound(2),env.bound(3),env.bound(4)]);
            xlabel('X-direction (meter)');
            ylabel('Y-direction (meter)');
            axis equal, grid on,  hold on;
%             axis tight,
        end
        
        
        
        
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  RRT Algorithm  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
        
%         function setWpt(env)                                                        % 경로점 지정 : 마우스 클릭을 통한 경로점 할당 : RRT 알고리듬 추가될 지점
%             for iter=1:env.nWaypt
%                 %경로점을 지정하기 위하여 ginput 함수를 사용하여
%                 % 그래프 상에서 사용자가 임의대로 할당해 준다.
%                 env.Waypt(iter,1:2)=ginput(1);
%                 plot(env.Waypt(iter,1),env.Waypt(iter,2),'ro');
%             end
% 
%             %할당된 경로점 사이의 거리를 구한다.
%             for iter=1:env.nWaypt-1
%                 env.distBtwWaypt(iter) = norm(env.Waypt(iter+1,1:2)-env.Waypt(iter,1:2));
%             end
%         end



%         function setWpt(env)                                                        % 경로점 지정 : 마우스 클릭을 통한 경로점 할당 : RRT 알고리듬 추가될 지점
%             env.Waypt = [0,0; 50 40;100 40;150 0;180 0];
%             env.nWaypt = size(env.Waypt,1);
%             plot(env.Waypt(:,1),env.Waypt(:,2),'r-o', 'LineWidth',3);
% 
%             %할당된 경로점 사이의 거리를 구한다.
%             for iter=1:env.nWaypt-1
%                 env.distBtwWaypt(iter) = norm(env.Waypt(iter+1,1:2)-env.Waypt(iter,1:2));
%             end
%         end
        
           
        
        
        
        function setWpt(env,rrt)                                                        % 경로점 지정 : 마우스 클릭을 통한 경로점 할당 : RRT 알고리듬 추가될 지점
            env.Waypt = rrt.wpt(:,1:2);
            plot(env.Waypt(:,1),env.Waypt(:,2),'r-o', 'LineWidth',1);
            
            env.Waypt = rrt.wpt(2:end,1:2);
            env.nWaypt = size(env.Waypt,1);

            %할당된 경로점 사이의 거리를 구한다.
            for iter=1:env.nWaypt-1
                env.distBtwWaypt(iter) = norm(env.Waypt(iter+1,1:2)-env.Waypt(iter,1:2));
            end
        end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
        


    end
    
end

