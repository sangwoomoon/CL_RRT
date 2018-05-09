classdef Vehicle < handle
    %VEHICLE 객체
    %   자율주행로봇과 관련한 변수들과 변수를 예측하기 위한 모델링된 기구학적 관계식이 담긴 함수가 있음
    
    properties
                                               %  변수명                  형태                                           설명
                                               % ===============================================================================================================
        Pos                                    %  위치                  배열(4X1)             [x좌표(meter),y좌표(meter),heading(radian),조향 각도(radian)]
        Vel                                    %  속도                  배열(4X1)          [x축 속도(m/s),y축 속도(m/s),heading 각속도(rad/s),조향 각속도(rad/s)]
        Acc                                    %  가속도                배열(4X1)     [x축 가속도(m/s^2),y축 가속도(m/s^2),heading 각가속도(rad/s^2),조향 각가속도(rad/s^2)]
        Speed                                  %  속력                    변수
        AccMag                                 %  가속도 크기             변수
        
        deltaMax                               %  최대조향각               변수          radian, 조향이 가능한 최대 각, || {조향각도} || =< {최대조향각}
        deltaMaxDot                            %  최대조향각속도           변수          rad/s, 조향이 가능한 최대 각속도, || {조향각속도} || =< {최대조향각속도}
        steerTime                              %  조향각속도 전달계수      변수          조향각속도 변화에 대한 1차 lag 전달함수의 계수. {조향각속도변화}=1/{조향각속도전달계수} * ({추종조향각속도}-{실제조향각속도})
        charaVel                               %  사이드슬립변수           변수          m^2/s^2, 사이드슬립(side-slip) 관련 변수. G_ss = 1 / (1 + ({속도}/{사이드슬립변수})^2)
        
        accTime                                %  가속도 전달계수          변수          가속도 변화에 대한 1차 lag 전달함수의 계수. {가속도변화}=1/{가속도전달계수} * ({추종가속도}-{실제가속도})
        accMin                                 %  최소 가속도              변수          m/s^2, 최소 가속도 값으로 감속하는 가속력이 최대인 값을 의미
        accMax                                 %  최대 가속도              변수          m/s^2, 최대 가속도 값으로 가속하는 가속력이 최대인 값을 의미 
        length                                 %  주행로봇차축사잇길이      변수          meter. 주행로봇의 앞 차축과 뒤 차축 사이의 길이
        ratioLfw                               %  추종길이비율             변수          주행로봇이 추종해야 할 추종점과의 거리를 차축사잇길이로 나눈 값.
        
        speedAcc                               %  속도명령계산변수(가속)    변수          m/s^2, 주행로봇의 속도명령값을 계산하기 위하여 설정한 가속도변수
        speedDec                               %  속도명령계산변수(감속)    변수          m/s^2, 주행로봇의 속도명령값을 계산하기 위하여 설정한 감속도변수
        speedMax                               %  최대속도                 변수          m/s, 주행로봇의 속도명령 최대값. 논문의 그래프를 통해 확인
        coastVelTime                           %  속도명령지속시간          변수          second, 최대속도가 아닌 속도의 명령값이 지속되는 시간 
        
        decAlphaZero                           %  감속모델계수(0차원)       변수          meter, 감속 브레이크의 사용에 대한 이동거리와 관계된 다항식(polynomial)의 0차원 계수
        decAlphaOne                            %  감속모델계수(1차원)       변수          second, 감속 브레이크의 사용에 대한 이동거리와 관계된 다항식(polynomial)의 1차원 계수
        decAlphaTwo                            %  감속모델계수(2차원)       변수          s^2, 감속 브레이크의 사용에 대한 이동거리와 관계된 다항식(polynomial)의 2차원 계수
        
        constThrottle                          %  추종추력제어이득계수     배열(3X1)       추종해야 할 추력에 필요한 제어이득을 계산하기 위한 다항식의 계수
        constThrVel                            %  추력대비속도전달함수계수 배열(2X1)       추력에 대한 속도의 전달함수 계수 
        speedModel                             %  추력-속도 전달함수      전달함수(TF)     추력에 따른 속도의 전달함수
        speedModelProfile                      %  추력속도전달함수결과     배열(2Xn)       논문에서 주어진 속도모델 TF 중, 상수로 취급되는 것을 제외한 항들로 STEP 신호 입력시 출력을 표현한 변수
        
        hisPos                                 %  위치저장            배열(4X{iterSim})            [x좌표(meter),y좌표(meter),heading(radian),조향 각도(radian)]
        hisVel                                 %  속도저장            배열(4X{iterSim})         [x축 속도(m/s),y축 속도(m/s),heading 각속도(rad/s),조향 각속도(rad/s)]
        hisAcc                                 %  가속도저장          배열(4X{iterSim})    [x축 가속도(m/s^2),y축 가속도(m/s^2),heading 각가속도(rad/s^2),조향 각가속도(rad/s^2)]
        hisSpeed                               %  속력저장            배열({iterSim})
        hisAccMag                              %  가속도 크기 저장     배열({iterSim})
        

    end
    
    methods
        function setInitialVehicleState(veh,pos,vel,acc)                                               % 주행로봇 관련 변수(위치, 속도, 가속도) 초기화
           veh.Pos=pos;
           veh.Vel=vel;
           veh.Acc=acc;
           veh.Speed = norm(vel(1:2));
           veh.AccMag = norm(acc(1:2));
           
           veh.hisPos(1,1:4) = pos';
           veh.hisVel(1,1:4) = vel';
           veh.hisAcc(1,1:4) = acc';
           veh.hisSpeed(1) = veh.Speed;
           veh.hisAccMag(1) = veh.AccMag;
        end
        
        function dispVehicle(veh)                                                                      % 주행로봇을 그래프에 표시
            figure(1)
            plot(veh.Pos(1),veh.Pos(2),'go');
        end
        
        function setTalos(veh)                                                                         % TALOS (MIT 자율주행자동차 애칭) 세팅
            veh.deltaMax = 0.5435;
            veh.deltaMaxDot = 0.3294;
            veh.steerTime = 0.05;
            veh.accTime = 0.3;
            veh.accMin = -6.0;
            veh.accMax = 1.8;
            veh.length = 2.885;
            veh.ratioLfw = 0.4;
            veh.charaVel = 20;
            
            veh.speedAcc = 1;                    % m/s^2
            veh.speedDec = 2.5;                  % m/s^2
            veh.speedMax = 11;                   % m/s (from result graph in the paper.)
            veh.coastVelTime = 3;                % s (not determined in the paper.)
            
            veh.decAlphaZero = -0.5347;          % m
            veh.decAlphaOne = 1.2344;            % s
            veh.decAlphaTwo = -0.0252;           % s^2
            
            veh.constThrottle = [0.1013 0.5788 49.1208];  % [c1 c2 c3] : Kn(v) = c1*v^2 + c2*v + c3
            veh.constThrVel = [12 1];                     % [tau_v 1] : V(s) = Kn(V)*U(s) / (tau_v(s)+1)
        end
        
        function setVechicleSpeedTF(veh)                                                                % 추력에 대한 주행로봇의 속도를 구하기 위한 전달함수를 세팅
            veh.speedModel = tf(1,veh.constThrVel);
            % step 신호에 대한 경향을 데이터로 받는다. 이 값에 제어이득을 곱하여 속도개형으로 활용한다.
            [veh.speedModelProfile(1,:),veh.speedModelProfile(2,:)] = step(veh.speedModel);
        end
        
        function CalculateSpeed(veh,ctrl,sim)                                                           % 속도 계산 함수
            % 속도를 업데이트 하면 속도변수의 값이 바뀌므로 먼저 속도저장변수에 업데이트 전 속도를 할당한다.
            veh.hisSpeed(sim.iterSim)=veh.Speed;
            
            % PI 제어에 의하여 추력의 제어이득변수가 결정된다.
            %  먼저, 초기부터 현재까지의 속도 오차를 합산한다.
            %  이 값을 이용해 I 제어이득과 곱하고, 현재의 상태를 이용해 P 제어이득과 곱한다.
            ctrl.sumVehicleSpeed = ctrl.sumVehicleSpeed+(ctrl.refVel-veh.Speed)*sim.deltaT;
            ThrottleInput=ctrl.Kp*(ctrl.refVel-veh.Speed)+ctrl.Ki*ctrl.sumVehicleSpeed;
           
            % Kn(v) (속도에 따른 추력 제어이득값)을 계산한다.
            Kn = veh.constThrottle(1)*veh.Speed^2+veh.constThrottle(2)*veh.Speed+veh.constThrottle(3);

            % 주행로봇의 예상 속도를 역라플라스변환 공식으로 표현한 결과로써 사용한다. ( V(t) = a*(1-e^(t/a'))
            veh.Speed = veh.Speed+Kn*ThrottleInput*(1-1/exp(sim.deltaT/veh.constThrVel(1)));
        end
        
        function CalculateState(veh,ctrl,sim)                                                           % 주행로봇 상태 계산 함수
            % 현재 주행로봇의 위치와 속도, 가속도를 저장변수에 할당한다.
            veh.hisPos(sim.iterSim,:) = veh.Pos;
            veh.hisVel(sim.iterSim,:) = veh.Vel;
            veh.hisAcc(sim.iterSim,:) = veh.Acc;
            
            % 먼저 주행로봇의 조향각속도를 산출하여 업데이트한다.
            veh.Vel(4) = min(max((ctrl.refSteer-veh.hisPos(sim.iterSim,4))/veh.steerTime,-veh.deltaMaxDot),veh.deltaMaxDot);
            % 주행로봇의 조향각도를 산출하여 업데이트한다.
            veh.Pos(4) = veh.hisPos(sim.iterSim,4)+veh.Vel(4)*sim.deltaT;
            % 주행로봇의 나머지 속도인 x,y축 속도와 방향각속도를 구한다. 
            %  x,y축 속도는 속력을 통해 얻고, 방향각속도는 사이드슬립을 고려하여 구한다. 
            %  {방향각속도} = {속도}/{차축사잇거리} * tan({조향각도}) * {사이드슬립계수(G_ss)}
            veh.Vel(1:3)... 
            = [veh.hisSpeed(sim.iterSim)*cos(veh.hisPos(sim.iterSim,3)),...
                veh.hisSpeed(sim.iterSim)*sin(veh.hisPos(sim.iterSim,3)),...
                veh.hisSpeed(sim.iterSim)*tan(veh.Pos(4))/veh.length*1/(1+(veh.hisSpeed(sim.iterSim)/veh.charaVel)^2)];
            % 주행로봇 속도를 이용하여 주행로봇의 위치를 구한다.
            veh.Pos = veh.hisPos(sim.iterSim,:)+veh.Vel*sim.deltaT;
            % 주행로봇 속도를 이용하여 주행로봇의 가속도를 구한다.
            veh.Acc = (veh.Vel-veh.hisVel(sim.iterSim,:))/sim.deltaT;
            % 주행로봇의 가속력은 주행로봇 가속도의 크기이다.
            veh.AccMag = norm(veh.Acc(1:2));
        end
       
        function PlotVehicleTrajectory(veh)                                                 % 주행로봇의 궤적을 그래프에 표시하는 함수
            figure(1)
            plot(veh.hisPos(:,1),veh.hisPos(:,2),'b.','MarkerSize',6);
            hold on
        end
    end
    
end

