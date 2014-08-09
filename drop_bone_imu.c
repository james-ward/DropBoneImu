#include "drop_bone_imu.h"

#include <stdio.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <unistd.h>
#include <string.h>

static int fd; // file descriptor for the I2C bus

int main(int argc, char **argv){
    init();

    // Loop and read from FIFO
    short accel[3], gyro[3], sensors[1];
    long quat[4];
    unsigned long timestamp;
    unsigned char more[0];
    for (;;) {
        int fifo_read = dmp_read_fifo(gyro, accel, quat, &timestamp, sensors, more);
        if (sensors[0]) {
            printf("Quaternions: %i\t%i\t%i\t%i\n", quat[0], quat[1], quat[2], quat[3]);
        }
    }
    return 0;
}

int init(void) {
    struct int_param_s int_param;
    
    unsigned char whoami=0;
    unsigned char dmp_state = 0;
    
    printf("Open bus: %i\n", open_bus());
    i2c_read(MPU6050_ADDR, MPU6050_WHO_AM_I, 1, &whoami);
    printf("WHO_AM_I: %x\n", whoami);

    printf("MPU init: %i\n", mpu_init(&int_param));

    /* Get/set hardware configuration. Start gyro. */
    /* Wake up all sensors. */
    mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    /* Push both gyro and accel data into the FIFO. */
    mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    mpu_set_sample_rate(DEFAULT_MPU_HZ);

    
    /* Now do the DMP stuff. */
    printf("DMP firmware load: %i\n", dmp_load_motion_driver_firmware());
    dmp_set_fifo_rate(DEFAULT_FIFO_HZ);
    /*dmp_enable_gyro_cal(1); // Will reset biases after 8 sec of no motion
    dmp_enable_lp_quat(1); // Generate quaternions
    * */
    unsigned short dmp_features = DMP_FEATURE_LP_QUAT | DMP_FEATURE_TAP |
        DMP_FEATURE_SEND_RAW_GYRO | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
        DMP_FEATURE_GYRO_CAL;
    dmp_enable_feature(dmp_features);
    //dmp_set_interrupt_mode(DMP_INT_CONTINUOUS); // Fire interrupt on new FIFO value
    mpu_set_dmp_state(1); // Turn on DMP
    mpu_get_dmp_state(&dmp_state);
    printf("DMP state: %i\n", dmp_state);
    //set_int_enable(1); // Enable interrupt
    return 0;
}

int i2c_write(unsigned char slave_addr, unsigned char reg_addr,
    unsigned char length, unsigned char const *data){
    unsigned char buf[length+1];
    buf[0] = reg_addr;
    memcpy(buf+1, data, length);
    return write(fd, buf, length+1) != length+1;
}
int i2c_read(unsigned char slave_addr, unsigned char reg_addr,
    unsigned char length, unsigned char *data){
    write(fd, &reg_addr, 1);
    return read(fd, data, length) != length; 
}

int open_bus() { 
    if ((fd = open(BBB_I2C_FILE, O_RDWR)) < 0) {
        /* ERROR HANDLING: you can check errno to see what went wrong */
        perror("Failed to open the i2c bus");
        return 1;
    }
    if (ioctl(fd, I2C_SLAVE, MPU6050_ADDR) < 0) {
        perror("Failed to acquire bus access and/or talk to slave.\n");
        /* ERROR HANDLING; you can check errno to see what went wrong */
        return 1;
    }
    return 0;
}

void delay_ms(unsigned long num_ms){
    
}
void get_ms(unsigned long *count){
    
}
void reg_int_cb(struct int_param_s *param){
    
}

inline int min ( int a, int b ){
    return a < b ? a : b;
}
inline void __no_operation(){
    
}


