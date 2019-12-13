q3_p=csvread('joint_reference.csv');  %read data from csv
time = 1:size(q3_p)

figure

subplot (2,1,1)
plot(time,q3_p(:,1),'r',time,q3_p(:,4),'b')
xlabel('time steps')
ylabel('q1 reference velocities (m/s)')
title('Velocity controller for SCARA')
lgd = legend({'q1 reference','q1 actual'},'FontSize',12,'TextColor','black','Location','southeast')

subplot (2,1,2)
plot(time,q3_p(:,2),'r',time,q3_p(:,5),'b')
xlabel('time steps')
ylabel('q2 reference velocities (m/s)')
title('Velocity controller for SCARA')
lgd = legend({'q2 reference','q2 actual'},'FontSize',12,'TextColor','black','Location','southeast')

