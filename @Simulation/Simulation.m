classdef Simulation < handle
    % SIMULATION 객체
    %   모의실험을 수행하기 위하여 필요한 기본적인 변수들과 함수들이 할당되어 있음
    
    properties
                                       %  변수명                  형태                                           설명
                                       % ===============================================================================================================
        deltaT                         % 시간간격                 변수                second. 시뮬레이션 수행시 불연속적인(discrete) 시간의 간격을 의미 (time step)
        iniTime                        % 초기시각                 변수                second. 시뮬레이션 수행을 시작할 때의 시각 (보통은 0으로 설정)
        totTime                        % 최종시각                 변수       second. 시뮬레이션을 수행하는 최대한의 시각으로, 이 시각을 넘어서면 무조건 모의실험을 중단시킨다.
        
        chkWayptRange                  % 최종경로점통과범위        변수       meter반경. 최종경로점은 look-ahead point가 아닌 주행로봇이 직접 도달해야 함 
        WayptIdx                       % 경로점인덱스             변수      경로점 중 현재 수행 중인 경로점의 순서가 담겨 있음 (몇 번째 경로점을 추종하고 있는가?)
        
        iterSim                        % 모의실험반복회수          변수        모의실험은 로봇이 최종점 도달 or 최종시각 도달 까지 계속 수행되는데
                                       %                                        이 때 모의실험이 deltaT에 맞추어 수행할 때의 회수를 의미. {진행시간} = {iterSim} X {deltaT}
    end
    
    
    
    
    methods
        function setSim(sim, dt, t0, totT, range)                            % 모의실험 변수할당 및 초기화 : input을 받아서 모의실험에 필요한 시간적, 임무수행요구 변수를 할당
           sim.deltaT = dt;
           sim.iniTime = t0;
           sim.totTime = totT;
           sim.chkWayptRange = range;
           sim.WayptIdx = 1;
           sim.iterSim = 1;
        end
        
        function evaluateSim(sim, env, veh, ctrl)                            % 모의실험 수행 : Simulation, Environment, Vehicle, Control 객체의 정보를 활용하여 모의실험 수행
            % 초기에는 무조건 수행을 해야 하므로 while 수행 조건을 TRUE로 둔다.
            procDet=1;
            % 모의실험 과정은 크게 두 개의 while loop문으로 구성된다.
            % 1. 최종경로점에 다다를 때까지 진행
            % 2. look-ahead point가 추종경로점에 다다를 때까지 진행
            while (sim.WayptIdx<env.nWaypt+1)
                while (procDet)
                    
                    % 조향 및 속도 제어
                    SteeringControl(ctrl,env,veh,sim);
                    SpeedControl(ctrl,env,veh,sim);

                    % 주행로봇의 속도와 기구학적 변수 계산 및 업데이트
                    CalculateSpeed(veh,ctrl,sim);
                    CalculateState(veh,ctrl,sim);
                    
                    % control 객체의 현재 정보를 해당 객체 내부에 있는 저장소(history)에 업데이트하고, 모의실험 반복회수를 늘린다.
                    setHistoryControlState(ctrl,sim);
                    sim.iterSim=sim.iterSim+1;
                    
                    % 모의실험 경과시간이 최종시간을 초과하면 종료.
%                     if (sim.iterSim > sim.totTime/sim.deltaT)
%                         break;
%                     end
                    
                    % 앞서 언급한 while loop문 중 2번의 조건을 만족시키기 위하여 계산한다.
                    %  만일 현재 추종하는 경로점이 최종경로점이 아니면, 추종경로점과 look-ahead point 사이의 거리가 look-ahead point 거리보다 작은지를 판별하고,
                    %  그렇지 않다면 추종경로점과 주행로봇 사이의 거리가 미리 할당한 최종경로점통과범위에 포함되어 있는지를 판별한다.
                    procDet = (sim.WayptIdx ~= env.nWaypt)*(norm(env.Waypt(sim.WayptIdx,1:2)-veh.Pos(1:2)) > ctrl.lfw)+...
                        (sim.WayptIdx == env.nWaypt)*(norm(env.Waypt(sim.WayptIdx,1:2)-veh.Pos(1:2)) > sim.chkWayptRange);
                end
               
                % 다음 추종경로점을 할당하기 위해 경로점인덱스를 늘리고, 
                % 역시 다음 추종경로점을 추종하기 위한 초기 추종에서는 무조건 수행을 해야 하므로 while 수행 조건을 TRUE로 둔다.
                sim.WayptIdx=sim.WayptIdx+1;
                procDet=1;
            end
        end
    end
    
end

