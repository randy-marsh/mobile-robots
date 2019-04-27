rosinit('http://172.29.30.173:11311')
%publisher
pub_enable=rospublisher('/cmd_motor_state','std_msgs/Int32');
%declaración mensaje
msg_enable_motor=rosmessage(pub_enable);
%activar motores enviando enable_motor = 1
msg_enable_motor.Data=1;
send(pub_enable,msg_enable_motor);
rosshutdown
