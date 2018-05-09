classdef Plot < handle
    %PLOT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        elaspedTime
        
        vehFrontAxle
        vehRealAxle
        vehMissionShaft
        vehCover
        vehTire
    end
    
    methods
        function setTimeProfile(plt,sim)
            plt.elaspedTime = 0:sim.deltaT:(sim.iterSim-2)*sim.deltaT;
        end
        
        function PlotSpeedProfile(plt,veh,ctrl)
            figure(2)
            plot(plt.elaspedTime,ctrl.hisRefVel,plt.elaspedTime,veh.hisSpeed);
            xlabel('Elasped Time (sec)');
            ylabel('Speed (m/s)');
        end
    end
    
end

