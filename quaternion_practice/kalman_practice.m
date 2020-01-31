clear all
a = arduino('COM8', 'Uno', 'Libraries', 'I2C');
fs = 100; % Sample Rate in Hz   
imu = mpu9250(a,'SampleRate',fs,'OutputFormat','matrix'); 
stopTimer = 100;

viewer = HelperOrientationViewer('Title',{'KALMAN Filter'});
% while(toc < stopTimer)
%     [accel,gyro,mag] = readSensorDataMPU9250(imu);
% %     rotators = FUSE(accel,gyro,mag);
% end
dt = 10.846803/1000;
global firstRun H Q R x P q_mag_saved avg_mag
if isempty(firstRun)
    H = eye(4);
% %     Q = [   0.0032    0.0039    0.0031    0.0083;
%             0.0039    0.0047    0.0037    0.0100;
%             0.0031    0.0037    0.0029    0.0079;
%             0.0083    0.0100    0.0079    0.0213];
    Q = [    0   -0.4000   -0.6500   -0.4000;
        0.4000         0    0.4000   -0.6500;
        0.6500   -0.4000         0    0.4000;
        0.4000    0.6500   -0.4000         0];
        % p q r 표준편차  = 0.0008 0.0013 0.0008
        % acc mag 표준편차 = [0.0077 0.0017 0.0017 0.0077]
        
    x = [1 0 0 0]';
    P = 1*eye(4);
%     R = [   0.1849    0.0430    0.0473    0.1591;
%             0.0430    0.0100    0.0110    0.0370;
%             0.0473    0.0110    0.0121    0.0407;
%             0.1591    0.0370    0.0407    0.1369];
    R = 1.0e-04*[   0.5929    0    0    0;
            0    0.0289    0    0;
            0    0    0.0289    0;
            0    0    0    0.5929];
    q_mag_saved = [0 0 0 0];
    
    count = 0;
end
tic;
while(toc < stopTimer)
    [accel,gyro,mag] = readSensorDataMPU9250(imu); % 값 받아오기!!!
    for i = 1 : 10     
        
% 마그네토 변환하기
        lx = mag(i,1);
        ly = mag(i,2);
        lz = mag(i,3);
        gamma = (lx^2 + ly^2);
%         if isempty (firstRun)
%             for k = 1 :100
%                 if(mag(i,1) >= 0 )
%                     q_mag = [sqrt(gamma + lx*sqrt(gamma))/sqrt(2*gamma); 0; 0; ly/(2^0.5*sqrt(gamma+lx*sqrt(gamma)))];
%                 end
%                 if(lx < 0)
%                     q_mag = [ly/(2^0.5*sqrt(gamma-lx*sqrt(gamma))); 0; 0; sqrt(gamma - lx*sqrt(gamma))/sqrt(2*gamma)];
%                 end
%                 q_mag_saved = [q_mag_saved; q_mag'];
%             end
%             avg_mag = [mean(q_mag_saved(:,1)); mean(q_mag_saved(:,2)); mean(q_mag_saved(:,3)); mean(q_mag_saved(:,4))];
%         end
        
        if(mag(i,1) >= 0 )
            q_mag = [sqrt(gamma + lx*sqrt(gamma))/sqrt(2*gamma); 0; 0; ly/(2^0.5*sqrt(gamma+lx*sqrt(gamma)))];
        end
        if(lx < 0)
            q_mag = [ly/(2^0.5*sqrt(gamma-lx*sqrt(gamma))); 0; 0; sqrt(gamma - lx*sqrt(gamma))/sqrt(2*gamma)];
        end
        
        ax = accel(i,1) / sqrt(accel(i,1)^2 +accel(i,2)^2 + accel(i,3)^2);
        ay = accel(i,2) / sqrt(accel(i,1)^2 +accel(i,2)^2 + accel(i,3)^2);
        az = accel(i,3) / sqrt(accel(i,1)^2 +accel(i,2)^2 + accel(i,3)^2);
%       쿼터니언으로 바꾸기(가속도 -> 쿼터니언)
%         z = [cos(phi/2)*cos(theta/2); sin(phi/2)*cos(theta/2);  cos(phi/2)*sin(theta/2); (- sin(phi/2)*sin(theta/2))];
        q_acc = [sqrt(0.5*(az + 1)); -ay/(2*sqrt(0.5*(az+1))); ax/(2*sqrt(0.5*(az+1))); 0];
        z = [0;0;0;0];
%         q_acc_mag = [0;0;0;0];
        % 엑셀과 마그네토 쿼터니언 곱으로 z 만들기
%         
%         [q_acc_mag(1), q_acc_mag(2),q_acc_mag(3),q_acc_mag(4)]=quat_mult(q_acc(1),q_acc(2),q_acc(3),q_acc(4),q_mag(1),q_mag(2),q_mag(3),q_mag(4));
%         [z(1),z(2),z(3),z(4)] = quat_mult(q_acc_mag(1), q_acc_mag(2),q_acc_mag(3),q_acc_mag(4),avg_mag(1),-avg_mag(2),-avg_mag(3),-avg_mag(4));

        [z(1),z(2),z(3),z(4)] = quat_mult(q_acc(1),q_acc(2),q_acc(3),q_acc(4),q_mag(1),q_mag(2),q_mag(3),q_mag(4));
%         gyro_quat = angle2quat(gyro(i,3)*dt,gyro(i,2)*dt, gyro(i,1)*dt)
        p = gyro(i,1) -0.0248;
        q = gyro(i,2) +0.0316; 
        r = gyro(i,3) -0.0225;
        A = eye(4) + dt*0.5*[0 -p -q -r; p 0 r -q; q -r 0 p; r q -p 0];

        xp = A*x;
        Pp = A*P*A' + Q;
        K = Pp*H'*inv(H*Pp*H' + R);
        x = xp + K*(z - H*xp);     % x = [ q1 q2 q3 q4 ] 
        x = x/norm(x);
        P = Pp - K*H*Pp;

        roll = atan2( 2*(x(3)*x(4) + x(1)*x(2)), 1- 2*(x(2)^2 + x(3)^2 )) ;
        pitch = -asin( 2*(x(2)*x(4) - x(1)*x(3)) ) ;
        yaw = atan2( 2*(x(2)*x(1)*x(4)), 1 - 2*(x(3)^2 + x(4)^2 ));
        if isempty(firstRun)
            roll_saved = roll;
            pitch_saved = pitch;
            yaw_saved = yaw;
            firstRun = 1;
        end
        
        quat = quaternion(x(1),x(2),x(3),x(4));
        viewer(quat);
        roll_saved = [roll_saved; roll];
        pitch_saved = [pitch_saved ; pitch];
        yaw_saved = [yaw_saved ; yaw];
        count = count +1;
        if(count == 1000)
            toc
        end
    end
%     figure(1)
%     plot(roll_saved, 'b');
%     grid on
%     figure(2)
%     plot(pitch_saved, 'b');
%     grid on
%     plot(3)
%     plot(yaw_saved,'b');
%     grid on  
    
    
end
% 쿼터니언 곱
function [q_0,q_1,q_2,q_3] = quat_mult(a_1,a_2,a_3,a_4,b_1,b_2,b_3,b_4)
q_0 = a_1*b_1 - a_2*b_2 - a_3*b_3 - a_4*b_4;
q_1 = a_1*b_2 + a_2*b_1 + a_3*b_4 - a_4*b_3;
q_2 = a_1*b_3 - a_2*b_4 + a_3*b_1 + a_4*b_2;
q_3 = a_1*b_4 + a_2*b_3 - a_3*b_2 + a_4*b_1;
end

% figure(1)
% plot(roll_saved, 'b');
% grid on
% figure(2)
% plot(pitch_saved, 'b');
% grid on
% figure(3)
% plot(yaw_saved, 'b');
% grid on
% for i = 1 :numel(roll_saved)
% 
%     figure(1)
%     plot(i,roll_saved(i), 'b-o');
%     hold on 
%     grid on
%     
%     figure(2)
%     plot(i , pitch_saved(i), 'b-o');
%     hold on
%     grid on
%     
%     figure (3)
%     plot(i, yaw_saved(i), 'b-o');
%     hold on 
%     grid on
%     
% end