classdef Control < handle
    %CONTROL 객체
    %   주행로봇의 종방향 및 횡방향 제어와 관련된 변수 및 함수가 포함됨
    
    properties
                                                       %  변수명                  형태                                           설명
                                                       % ===============================================================================================================
        Kp                                             % 추력입력 P-gain          변수             추력 입력값을 만들기 위한 속도에 대한 P-gain. Kp * ({추종속도}-{현재속도}) 
        Ki                                             % 추력입력 I-gain          변수             추력 입력값을 만들기 위한 속도에 대한 I-gain. Ki * int({추종속도}-{속도} fr. t=0 to t=current) 
        Kd                                             % 추력입력 D-gain          변수             추력 입력값을 만들기 위한 속도에 대한 D-gain. Kd * d/dt({추종속도}-{현재속도}) 
        
        sumVehicleSpeed                                % 추력입력 산출변수         변수             m/s, 추력 입력값의 제어이득 중 I-gain을 구하기 위하여 속도오차를 더할 때 사용하는 변수
        
        lfw                                            % 추종거리                 변수             meter, 현재위치 대비 추종점이 있어야 할 상대거리를 나타내며, 속도에 대한 종속변수
        refLfw                                         % 뒤차축대비추종선시작거리   변수            meter, 뒤 차축으로부터 추종선(look-ahead line)이 시작하는 거리 
        posLfw                                         % 추종점위치              배열(2X1)             [x좌표(meter), y좌표(meter)], 추종점(look-ahead point) 위치
        lfwMin                                         % 최소추종거리              변수            meter, 추종거리는 주행로봇의 속도에 대한 함수로 표현되는데, 이 때 최소추종 거리
        lfwMax                                         % 최대추종거리              변수            meter, 추종거리는 주행로봇의 속도에 대한 함수로 표현되는데, 이 때 최대추종 거리
        lfwSlope                                       % 추종거리변화율            변수            second, 추종거리의 변화를 속도에 대한 함수로 나타냄, {추종거리} = {추종거리변화율}X{추종속도}
        
        refVel                                         % 추종속도                  변수            m/s, 주행로봇이 추종해야 할 속도
        refCoastVel                                    % 추종주행속도              변수            m/s, 주행로봇이 가속/감속하여 일정하게 주행하는 상태에서의 속도
        refSteer                                       % 추종조향각도              변수            radian, 주행로봇이 추종해야 할 조향각도
        refSteerEta                                    % 추종선각도                변수            radian, 경로선과 추종선 사이의 각도
        
        hisLfw                                         % 추종거리저장        배열(1X{iterSim})     모의실험 수행과정에 따른 추종거리 저장 변수
        hisPosLfw                                      % 추종점위치저장      배열(1X{iterSim})     모의실험 수행과정에 따른 추종점위치 저장 변수
        hisRefVel                                      % 추종속도저장        배열(1X{iterSim})     모의실험 수행과정에 따른 추종속도 저장 변수
        hisRefCoastVel                                 % 추종주행속도저장     배열(1X{iterSim})     모의실험 수행과정에 따른 추종주행속도 저장 변수
        hisRefSteer                                    % 추종조향각도저장     배열(1X{iterSim})     모의실험 수행과정에 따른 추종조향각도 저장 변수
        hisRefSteerEta                                 % 추종선각도저장       배열(1X{iterSim})     모의실험 수행과정에 따른 추종선각도 저장 변수
    end
    
    methods
        function setLookAheadDistance(ctrl,veh)                                                         % 추종거리 및 추종점 설정
            ctrl.refLfw = veh.ratioLfw*veh.length;
            ctrl.posLfw = veh.hisPos(1,1:2);
        end
        
        function setControlTalos(ctrl)                                                                  % TALOS (MIT 자율주행자동차 애칭) 세팅 및 초기화
            ctrl.lfwMin = 3;
            ctrl.lfwMax = 12;
            ctrl.lfwSlope = 2.24;
            ctrl.refVel = 0;
            ctrl.sumVehicleSpeed = 0;
        end
        
        function setPID(ctrl,p,i,d)                                                                     % 추력입력값 관련 PID 제어이득 설정
            ctrl.Kp = p;
            ctrl.Ki = i;
            ctrl.Kd = d;
        end
        
        function SteeringControl(ctrl,env,veh,sim)                                                      % 조향제어
            % 추종거리를 산출한다.
            %  추종속도가 1.34m/s 미만이면 최소추종거리
            %  추종속도가 1.34m/s 이상 5.36m/s 미만이면 {추종거리변화율}*{추종속도}
            %  추종속도가 5.36m/s 이상이면 최대추종거리
            ctrl.lfw=(ctrl.refVel<1.34)*ctrl.lfwMin+...
                (ctrl.refVel>=5.36)*ctrl.lfwMax+...
                ((ctrl.refVel>=1.34)&&(ctrl.refVel<5.36))*(ctrl.lfwSlope*ctrl.refVel);
            
            % 조향각을 구하기 위하여 논문에서 나온 조향각 유도식을 계산한다.
            %  1) 주행로봇의 위치에서 추종경로점으로 이은 벡터의 방향을 추종방향으로 잡는다.            
            RefDirection = (env.Waypt(sim.WayptIdx,1:2)-veh.Pos(1:2))/norm((env.Waypt(sim.WayptIdx,1:2)-veh.Pos(1:2)));
            %  2) 주행로봇 위치에서 추종경로점으로 이은 벡터의 방향에서 주행로봇의 방향을 빼주어 상대적 방향각을 구한다.
            ctrl.refSteerEta=veh.Pos(3)-atan2(RefDirection(2),RefDirection(1));
            %  3) 논문에서 제시한 유도식을 이용하여 추종조향각을 구한다.
            ctrl.refSteer = -atan2(veh.length*sin(ctrl.refSteerEta),ctrl.lfw/2+ctrl.refLfw*cos(ctrl.refSteerEta));
            %  4) 추종조향각은 주행로봇의 조향각 크기에 제한이 있으므로 제한조건을 적용한다.
            ctrl.refSteer = min(max(ctrl.refSteer,-veh.deltaMax),veh.deltaMax);
        end
        
        
        
        function SpeedControl(ctrl,env,veh,sim)                                                      % 속도제어
            % 현재 위치부터 최종경로점까지의 거리를 계산한다. 
            %  이 때 경유해야 할 경로점을 통과한 거리를 계산한다.
            %  1) 추종점의 위치를 구한다.
            % ctrl.posLfw=veh.Pos(1:2)+ctrl.lfw*[cos(ctrl.refSteerEta-veh.Pos(3)),-sin(ctrl.refSteerEta-veh.Pos(3))];
            % plot(ctrl.posLfw(1),ctrl.posLfw(2),'go');
            
            %  2) 추종점의 위치에서부터 추종경로점까지의 길이를 구한다.
            %  본 연구에서는 현재위치-추종경로점 사이 거리로 하였다. (단순화)
            DistToGoal=norm(veh.Pos(1:2)-env.Waypt(sim.WayptIdx,1:2));
            %  3) 추종경로점 이후부터 최종경로점 사이의 거리를 더해준다.
            for iter = sim.WayptIdx : env.nWaypt-1
                DistToGoal=DistToGoal+env.distBtwWaypt(iter);
            end
            
            % 추종주행속도를 구한다.
            %  속도의 프로파일은 논문에 나온바와 같이 가속-등속-감속의 모습을 보이며,
            %  속도의 프로파일에 의하여 진행되는 이동거리를 계산한다.
            %  최대속력으로 가속되어 주행하다가 감속할 경우의 이동거리의 크기가 
            %  가야 할 거리보다 작게 된다면, 추종주행속도는 최대속력이 된다.
            if (veh.speedMax^2/(2*veh.speedDec)+veh.decAlphaTwo*veh.speedMax^2+...
                    veh.speedMax*veh.coastVelTime+veh.decAlphaOne*veh.speedMax+veh.decAlphaZero+...
                    (veh.speedMax^2-veh.Speed^2)/(2*veh.speedAcc) < DistToGoal)
                ctrl.refCoastVel=veh.speedMax;
            %  만일 그렇지 않고 가야할 거리보다 크거나 같다면, 추종주행속도를 따로
            %  구해주어야 한다. 최대 속력으로 넣은 항을 미지수로 두고 2차항의 방정식을
            %  풀어 준다.
            else
                % 2차 방정식에서 근의 공식을 활용하여 구한다. 
                CoastVelEqnA=1/(2*veh.speedAcc)+1/(2*veh.speedDec)+veh.decAlphaTwo;
                CoastVelEqnB=veh.coastVelTime+veh.decAlphaOne;
                CoastVelEqnC=-veh.Speed^2/(2*veh.speedAcc)+veh.decAlphaZero-DistToGoal;
                ctrl.refCoastVel=(-CoastVelEqnB+sqrt(CoastVelEqnB^2-4*CoastVelEqnA*CoastVelEqnC))/(2*CoastVelEqnA);
            end
            
            % 추종주행속도가 현재 속도보다 낮으면 속도가 일정시간(time step)만큼 가속된 값을 추종속도로 할당하고,
            %  같으면 추종주행속도 값으로 할당하며, 그렇지 않으면 일정시간 만큼 감속된 값을 추종속도로 할당한다.  
            ctrl.refVel = ctrl.refCoastVel;
            %ctrl.refVel=(veh.Speed<ctrl.refCoastVel)*(veh.Speed+veh.speedAcc*sim.deltaT)+(veh.Speed>=ctrl.refCoastVel)*(veh.Speed-veh.speedDec*sim.deltaT);
        end
        
        
        
        function setHistoryControlState(ctrl,sim)                                                       % 제어 관련 변수를 저장해둔다.
            ctrl.hisLfw(sim.iterSim) = ctrl.lfw;
            ctrl.hisPosLfw(sim.iterSim,:) = ctrl.posLfw;
            ctrl.hisRefVel(sim.iterSim) = ctrl.refVel; 
            ctrl.hisRefCoastVel(sim.iterSim) = ctrl.refCoastVel;
            ctrl.hisRefSteer(sim.iterSim) = ctrl.refSteer;
            ctrl.hisRefSteerEta(sim.iterSim) = ctrl.refSteerEta;
        end
    end
    
end

