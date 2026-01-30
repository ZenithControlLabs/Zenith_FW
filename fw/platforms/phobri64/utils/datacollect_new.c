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


    fprintf(csv_file, "micros,ax,ay\n");
    printf("OK!\n");
    while (1) {
	clock_gettime(CLOCK_MONOTONIC, &tp);
        bytes_read = hid_read(handle, device_resp, 20);

        if (bytes_read == 20) {
	    long long micros = (tp.tv_sec - start.tv_sec) * 1000 * 1000 + tp.tv_nsec / 1000;
            uint32_t ax_d = 
                ((uint32_t)(device_resp[15] << 24) +
                ((uint32_t)(device_resp[14]) << 16) +
                ((uint32_t)(device_resp[13]) << 8) +
                ((uint32_t)(device_resp[12])));
            uint32_t ay_d = 
                ((uint32_t)(device_resp[19] << 24) +
                ((uint32_t)(device_resp[18]) << 16) +
                ((uint32_t)(device_resp[17]) << 8) +
                ((uint32_t)(device_resp[16])));
            float ax_f = *((float*)&ax_d);
            float ay_f = *((float*)&ay_d);

            printf("%lld,%0.4f, %0.4f\n", micros,ax_f, ay_f);
            fprintf(csv_file, "%lld,%0.4f,%0.4f\n", micros, ax_f, ay_f);

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
