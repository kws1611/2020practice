clear all
a = arduino('COM8', 'Uno', 'Libraries', 'I2C');
fs = 100; % Sample Rate in Hz   
imu = mpu9250(a,'SampleRate',fs,'OutputFormat','matrix'); 
stopTimer = 100;
tic;
dt = 1.924928/100;
global firstRun H Q R x P
if isempty(firstRun)
    H = eye(4);
    Q = 0.0001*eye(4);
    x = [1 0 0 0]';
    P = 1*eye(4);
end
while(toc < stopTimer)
    [accel,gyro,mag] = readSensorDataMPU9250(imu); % 값 받아오기!!!
    for i = 1 : 10     
        
% 마그네토 변환하기
        lx = mag(i,1);
        ly = mag(i,2);
        lz = mag(i,3);
        gamma = (lx^2 + ly^2);
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
        % 엑셀과 마그네토 쿼터니언 곱으로 z 만들기
        [z(1), z(2),z(3),z(4)]=quat_mult(q_acc(1),q_acc(2),q_acc(3),q_acc(4),q_mag(1),q_mag(2),q_mag(3),q_mag(4));
        
%         gyro_quat = angle2quat(gyro(i,3)*dt,gyro(i,2)*dt, gyro(i,1)*dt)
        p = gyro(i,1) ;
        q = gyro(i,2) ; 
        r = gyro(i,3) ;
        A = eye(4) + dt*0.5*[0 -p -q -r; p 0 r -q; q -r 0 p; r q -p 0];

%         x = A*x;
%         x = x/norm(x);
        x = [p q r];
        if isempty(firstRun)
            x_saved = x;
            z_saved = z';
            q_mag_saved = q_mag';
            q_acc_saved = q_acc';
            firstRun = 1;
        end
        x_saved = [x_saved; x];
        z_saved = [z_saved ; z'];
        q_mag_saved = [q_mag_saved ; q_mag'];
        q_acc_saved = [q_acc_saved ; q_acc'];
    end
end
% 쿼터니언 곱
function [q_0,q_1,q_2,q_3] = quat_mult(a_1,a_2,a_3,a_4,b_1,b_2,b_3,b_4)
q_0 = a_1*b_1 - a_2*b_2 - a_3*b_3 - a_4*b_4;
q_1 = a_1*b_2 + a_2*b_1 + a_3*b_4 - a_4*b_3;
q_2 = a_1*b_3 - a_2*b_4 + a_3*b_1 + a_4*b_2;
q_3 = a_1*b_4 + a_2*b_3 - a_3*b_2 + a_4*b_1;
end
