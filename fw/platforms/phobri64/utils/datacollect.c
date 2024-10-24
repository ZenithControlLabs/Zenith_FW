#include <math.h>
#include <time.h>
#include <hidapi/hidapi.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#define VENDOR_ID 0x7318
#define PRODUCT_ID 0x0001
//#define VENDOR_ID 0x20d6
//#define PRODUCT_ID 0xa714

int main(int argc, char **argv) {
    hid_device *handle;
    unsigned char device_resp[33];
    int result;
    struct timespec tp;

    result = hid_init();

    handle = hid_open(VENDOR_ID, PRODUCT_ID, NULL);

    if (handle == NULL) {
        printf("Can't open Phobri device.\n");
        result = hid_exit();
        exit(-1);
    }

    FILE* csv_file = fopen("ctlr.csv", "w");
    if (csv_file == NULL) {
        printf("Can't open ctrl.csv for writing.\n");
        exit(-1);
    }

    int bytes_read = 0;
    int cnt = 0;
    struct timespec start;
    clock_gettime(CLOCK_MONOTONIC, &start);


    fprintf(csv_file, "micros,hx_x,hx_y,hx_z,hx_temp,ax,ay\n");
    printf("OK!\n");
    while (1) {
	clock_gettime(CLOCK_MONOTONIC, &tp);
        bytes_read = hid_read(handle, device_resp, 13);

        if (bytes_read == 13) {
	    long long micros = (tp.tv_sec - start.tv_sec) * 1000 * 1000 + tp.tv_nsec / 1000;
            int16_t hx_x = (int16_t)((device_resp[2] << 8) + device_resp[1]);
            int16_t hx_y = (int16_t)((device_resp[4] << 8) + device_resp[3]);
            int16_t hx_z = (int16_t)((device_resp[6] << 8) + device_resp[5]);
            int16_t hx_temp = (int16_t)((device_resp[8] << 8) + device_resp[7]);
            uint16_t ax = (device_resp[10] << 8) + device_resp[9];
            uint16_t ay = (device_resp[12] << 8) + device_resp[11];
	    
            printf("%lld,%d,%d,%d,%d,%d,%d\n", micros,  hx_x, hx_y, hx_z, hx_temp, ax, ay);
            fprintf(csv_file, "%lld,%d,%d,%d,%d,%d,%d\n", micros, hx_x, hx_y, hx_z, hx_temp, ax, ay);

        } else {
            printf("Didn't read the proper amount.. %d\n", bytes_read);
        }
        cnt++;
	if (cnt % 2000 == 0) {
		fflush(csv_file);
	}
    }
    fclose(csv_file);

}
