close all;
% for mpc based results
%% output data
lat_error = out.cross_track;
yaw_error = out.Theta_e;
x_state = out.Host_x;
x_state = x_state(1:n(1));
n = size(yaw_error);
% match the reference input size to state output
tRefq = (linspace(0,sim_time,n(1)))';
xRefq = interp1(tRef,xRef,tRefq);
yRefq = interp1(tRef,yRef,tRefq);
yawRefq = interp1(tRef,yawRef,tRefq); % degrees

for ii = 1:n(1)
    y_state(ii) = lat_error(ii) + yRefq(ii);
    yaw_state(ii) = yaw_error(ii) + deg2rad(yawRefq(ii));
end

%% target state
y_target = 2*ones(1,n(1));
x_target = out.Target_x;
x_target =x_target(1:n(1)); 

% all states for the trajectory equation
all_data = [x_state(1:n(1)) y_state' yaw_state' x_target y_target'];


%% plots
% 1. ACC Range_target vs Range_host
figure(1)
plot(x_target,y_target,'Linewidth',2);
grid on ; xlabel('X (m)','FontSize',12,'FontWeight','bold','Color','k');ylabel('Y (m)','FontSize',12,'FontWeight','bold','Color','k');
hold on
plot(x_state,y_state,'Linewidth',2);
legend('Target trajectory', 'Host Trajectory');

%2. Range vs Range rate
figure(2)
plot(x_target,'Linewidth',2);
grid on ; xlabel('Time(s)','FontSize',12,'FontWeight','bold','Color','k');ylabel('Y (m)','FontSize',12,'FontWeight','bold','Color','k');
hold on
plot(out.Range_rate(1:n(1)),'Linewidth',2);
legend('Range', 'Range rate');
% 3. V host
figure(3)
plot(out.V_host,'Linewidth',2);
grid on ; xlabel('Time(s)','FontSize',12,'FontWeight','bold','Color','k');ylabel('Velocity (mps)','FontSize',12,'FontWeight','bold','Color','k');

figure(4)
plot(yaw_error,'Linewidth',2);
grid on ; xlabel('Time(s)','FontSize',12,'FontWeight','bold','Color','k');ylabel('Yaw angle error (rads)','FontSize',12,'FontWeight','bold','Color','k');


%%
HA_simulation(all_data,lat_error,tRefq)