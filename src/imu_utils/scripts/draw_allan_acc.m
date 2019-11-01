clear 
close all

dt = dlmread('/home/brucesz/git/opensource/vio_assignment/gyro_15_data_r_400_60min/data_imu_name_test_acc_t.txt');         
data_x = dlmread('/home/brucesz/git/opensource/vio_assignment/gyro_15_data_r_400_60min/data_imu_name_test_acc_x.txt'); 
data_y= dlmread('/home/brucesz/git/opensource/vio_assignment/gyro_15_data_r_400_60min/data_imu_name_test_acc_y.txt'); 
data_z = dlmread('/home/brucesz/git/opensource/vio_assignment/gyro_15_data_r_400_60min/data_imu_name_test_acc_z.txt'); 
data_draw=[data_x data_y data_z] ;

data_sim_x= dlmread('/home/brucesz/git/opensource/vio_assignment/gyro_15_data_r_400_60min/data_imu_name_test_sim_acc_x.txt'); 
data_sim_y= dlmread('/home/brucesz/git/opensource/vio_assignment/gyro_15_data_r_400_60min/data_imu_name_test_sim_acc_y.txt'); 
data_sim_z= dlmread('/home/brucesz/git/opensource/vio_assignment/gyro_15_data_r_400_60min/data_imu_name_test_sim_acc_z.txt'); 
data_sim_draw=[data_sim_x data_sim_y data_sim_z] ;


figure
loglog(dt, data_draw , 'r+');
% loglog(dt, data_sim_draw , '-');
xlabel('time:sec');                
ylabel('acc allan.');             
% legend('x','y','z');      
grid on;                           
hold on;                           
loglog(dt, data_sim_draw , 'r-');