clear 
close all
prefix = '/home/brucesz/git/opensource/vio_assignment/gyro_15_data_r_400_60min/data_imu_name_test'
dt = dlmread('/home/brucesz/git/opensource/vio_assignment/gyro_15_data_r_400_60min/data_imu_name_test_gyr_t.txt');         
data_x = dlmread('/home/brucesz/git/opensource/vio_assignment/gyro_15_data_r_400_60min/data_imu_name_test_gyr_x.txt'); 
data_y= dlmread('/home/brucesz/git/opensource/vio_assignment/gyro_15_data_r_400_60min/data_imu_name_test_gyr_y.txt'); 
data_z = dlmread('/home/brucesz/git/opensource/vio_assignment/gyro_15_data_r_400_60min/data_imu_name_test_gyr_z.txt'); 
data_draw=[data_x data_y data_z] ;

data_sim_x= dlmread('/home/brucesz/git/opensource/vio_assignment/gyro_15_data_r_400_60min/data_imu_name_test_sim_gyr_x.txt'); 
data_sim_y= dlmread('/home/brucesz/git/opensource/vio_assignment/gyro_15_data_r_400_60min/data_imu_name_test_sim_gyr_y.txt'); 
data_sim_z= dlmread('/home/brucesz/git/opensource/vio_assignment/gyro_15_data_r_400_60min/data_imu_name_test_sim_gyr_z.txt'); 
data_sim_draw=[data_sim_x data_sim_y  data_sim_z] ;


figure
loglog(dt, data_draw , 'o');
% loglog(dt, data_sim_draw , '-');
xlabel('time:sec');                
ylabel('gyro allan deviation/h');             
% legend('x','y','z');      
grid on;                           
hold on;                           
loglog(dt, data_sim_draw , '-');
