
clc
clear all
close all
clf


%% ----------- Simulation Setup -----------%

simulation = Simulation;
setSim(simulation, 0.2, 0, 200, 0.5);


%% ----------- Environment Setup ------------%

environment = Environment;
setBound(environment,[0 200 0 200]);
dispField(environment);

%% ----------- Vehicle Setup ------------%

vehicle = Vehicle;
setTalos(vehicle);
setInitialVehicleState(vehicle,[50 50 0.25*pi 0],[0 0 0 0],[0 0 0 0]);
dispVehicle(vehicle);



%% ------------ Look Ahead Point Setup --------------%

control = Control;
setLookAheadDistance(control,vehicle);
setControlTalos(control);

%% --------------- PID Setup ---------------%

setPID(control,0.2,0.04,0);



%% --------------- Algorithm Proceeding ---------------%
% 
% evaluateSim(simulation, environment, vehicle, control)






%% ------------ RRT Test -------------%
rrtPlanner = RRTPlanner;
setRRT(rrtPlanner,vehicle) 
TreeExpansion(rrtPlanner,environment,vehicle, control,simulation) 






%% --------------- Result Plotting ----------------%

% PlotVehicleTrajectory(vehicle);


dt = simulation.deltaT;
Speed = vehicle.hisSpeed;
Vel = vehicle.hisVel(:,1:2);
for i=1:length(vehicle.hisSpeed); VelCar(i) = norm(Vel(i,:));end
VelCmd = control.hisRefVel;

Time=0:dt:(length(vehicle.hisSpeed)-1)*dt;

figure(2)
plot(Time,VelCmd,Time,Speed);
legend('VelCmd','Speed');xlabel('Time (sec)');ylabel('Speed (m/s)');










% figure(3)
% plot(Time,Speed,Time,VelCar);
% legend('Speed','VelCar');xlabel('Time (sec)');ylabel('Speed (m/s)');


% figure(3);plot(vehicle.hisPos(:,1),vehicle.hisPos(:,2),'r');axis([-100 100 -100 100])
% figure(2);plot(Time,Vel,Time,VelCmd);legend('Vel','VelCmd');xlabel('Time (sec)');ylabel('Speed (m/s)');
% figure(3);plot(Time,Vel,Time,Speed);legend('Vel','Speed');xlabel('Time (sec)');ylabel('Speed (m/s)');




