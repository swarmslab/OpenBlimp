import pickle
import numpy as np
import sys

if __name__ == '__main__':
    directory = (""+sys.argv[1].replace("\\","/").replace("\"", ""))
    with open(directory+"/log_dest_x.dat", "rb") as file:
        log_destination_x = pickle.load(file)
    with open(directory+"/log_dest_y.dat", "rb") as file:
        log_destination_y = pickle.load(file)
    with open(directory+"/log_dest_z.dat", "rb") as file:
        log_destination_z = pickle.load(file)
    with open(directory+"/log_pos_x.dat", "rb") as file:
        log_position_x = pickle.load(file)
    with open(directory+"/log_pos_y.dat", "rb") as file:
        log_position_y = pickle.load(file)
    with open(directory+"/log_pos_z.dat", "rb") as file:
        log_position_z = pickle.load(file)
    with open(directory+"/log_err_x.dat", "rb") as file:
        log_error_x = pickle.load(file)
    with open(directory+"/log_err_y.dat", "rb") as file:
        log_error_y = pickle.load(file)
    with open(directory+"/log_err_z.dat", "rb") as file:
        log_error_z = pickle.load(file)
    with open(directory+"/log_err_yaw.dat", "rb") as file:
        log_error_yaw = pickle.load(file)
    with open(directory+"/log_err_vel_x.dat", "rb") as file:
        log_error_vel_x = pickle.load(file)
    with open(directory+"/log_err_vel_y.dat", "rb") as file:
        log_error_vel_y = pickle.load(file)
    with open(directory+"/log_err_vel_z.dat", "rb") as file:
        log_error_vel_z = pickle.load(file)
    with open(directory+"/t.dat", "rb") as file:
        t = pickle.load(file)
    with open(directory+"/time.dat", "rb") as file:
        flightTime = pickle.load(file)
    with open(directory+"/log_m1", "rb") as file:
        log_m1 = pickle.load(file)
    with open(directory+"/log_m2", "rb") as file:
        log_m2 = pickle.load(file)
    with open(directory+"/log_m3", "rb") as file:
        log_m3 = pickle.load(file)
    with open(directory+"/log_m4", "rb") as file:
        log_m4 = pickle.load(file)
    with open(directory+"/t2.dat", "rb") as file:
        t2 = pickle.load(file)
        
    index = 300
        
    mag_pos_err = np.linalg.norm(np.array([log_error_x[index:], log_error_y[index:], log_error_z[index:]]),axis=0)
    avg_pos_err = np.mean(mag_pos_err)
    max_pos_err = np.max(mag_pos_err)
    min_pos_err = np.min(mag_pos_err)
    
    mag_vel_err = np.linalg.norm(np.array([log_error_vel_x[index:], log_error_vel_y[index:], log_error_vel_z[index:]]),axis=0)
    avg_vel_err = np.mean(mag_vel_err)
    max_vel_err = np.max(mag_vel_err)
    min_vel_err = np.min(mag_vel_err)
    
    mag_xy_pos_err = np.linalg.norm(np.array([log_error_x[index:], log_error_y[index:]]),axis=0)
    avg_xy_err = np.mean(mag_xy_pos_err)
    max_xy_err = np.max(mag_xy_pos_err)
    min_xy_err = np.min(mag_xy_pos_err)
    
    avg_z_err = np.mean(log_error_z[index:])
    max_z_err = np.max(log_error_z[index:])
    min_z_err = np.min(log_error_z[index:])
    
    print(np.mean(mag_pos_err[len(mag_pos_err)//4:]))
    print(np.mean(mag_vel_err[len(mag_vel_err)//4:]))
    
    with open(directory+"/data.txt", "w") as file:
            file.write(
                "Average Position Error Magnitude: %fm\n" \
                "Maximum Position Error Magnitude: %fm\n" \
                "Minimum Position Error Magnitude: %fm\n" \
                "Average Velocity Error Magnitude: %fm/s\n" \
                "Maximum Velocity Error Magnitude: %fm/s\n" \
                "Minimum Velocity Error Magnitude: %fm/s\n" \
                "Average XY Position Error Magnitude: %fm\n" \
                "Maximum XY Position Error Magnitude: %fm\n" \
                "Minimum XY Position Error Magnitude: %fm\n" \
                "Average Z Position Error Magnitude: %fm\n" \
                "Maximum Z Position Error Magnitude: %fm\n" \
                "Minimum Z Position Error Magnitude: %fm"% (
                avg_pos_err,max_pos_err,min_pos_err,avg_vel_err,max_vel_err,min_vel_err,avg_xy_err,max_xy_err,min_xy_err,avg_z_err, max_z_err, min_z_err
                     ))
    
    
    