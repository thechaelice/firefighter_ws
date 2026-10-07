// Reads frames from a Melexis MLX90641 thermal camera (I2C @ 0x33 on /dev/i2c-1)
// and writes them as CSV on stdout: one 16x12 temperature block per frame,
// blocks separated by a blank line. Diagnostics go to stderr.
//
// Build with scripts/thermal/build.sh (needs melexis/mlx90641-library).
//
// Usage: mlx90641_frames [num_frames] [delay_ms]

#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>

#include "MLX90641_API.h"
#include "MLX90641_I2C_Driver.h"

#ifndef I2C_DEV
#define I2C_DEV "/dev/i2c-1"
#endif
#define ADDR 0x33

// ---- Linux I2C backend for the vendor API -----------------------------------
static int g_fd = -1;

void MLX90641_I2CInit(void) {
    if (g_fd >= 0) return;
    g_fd = open(I2C_DEV, O_RDWR);
    if (g_fd < 0) perror("open " I2C_DEV);
}

void MLX90641_I2CFreqSet(int) { /* the kernel owns the bus clock */ }

int MLX90641_I2CGeneralReset(void) {
    if (g_fd < 0) return -1;
    uint8_t cmd[2] = {0x00, 0x06};                 // general-call software reset
    if (write(g_fd, cmd, 2) != 2) return -1;
    usleep(1000);
    return 0;
}

int MLX90641_I2CRead(uint8_t slaveAddr, uint16_t startAddress,
                     uint16_t n, uint16_t *data) {
    if (g_fd < 0) return -1;
    uint8_t addr[2] = {(uint8_t)(startAddress >> 8), (uint8_t)(startAddress & 0xFF)};
    static uint8_t buf[4096];
    if (2u * n > sizeof(buf)) return -1;

    struct i2c_msg msgs[2];
    msgs[0].addr = slaveAddr; msgs[0].flags = 0;        msgs[0].len = 2;   msgs[0].buf = addr;
    msgs[1].addr = slaveAddr; msgs[1].flags = I2C_M_RD; msgs[1].len = 2*n; msgs[1].buf = buf;

    struct i2c_rdwr_ioctl_data x = { msgs, 2 };
    if (ioctl(g_fd, I2C_RDWR, &x) < 0) return -1;

    for (int i = 0; i < n; i++)
        data[i] = (uint16_t)buf[2 * i] * 256u + buf[2 * i + 1];   // MSB first
    return 0;
}

int MLX90641_I2CWrite(uint8_t slaveAddr, uint16_t reg, uint16_t val) {
    if (g_fd < 0) return -1;
    uint8_t cmd[4] = {(uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF),
                      (uint8_t)(val >> 8), (uint8_t)(val & 0xFF)};
    struct i2c_msg m = { slaveAddr, 0, 4, cmd };
    struct i2c_rdwr_ioctl_data x = { &m, 1 };
    return ioctl(g_fd, I2C_RDWR, &x) < 0 ? -1 : 0;
}

// ---- main --------------------------------------------------------------------
int main(int argc, char **argv) {
    int frames   = (argc > 1) ? atoi(argv[1]) : 3;
    int delay_ms = (argc > 2) ? atoi(argv[2]) : 800;
    if (frames < 1) frames = 1;

    MLX90641_I2CInit();
    if (g_fd < 0) return 1;

    static uint16_t ee[832];
    if (MLX90641_DumpEE(ADDR, ee) != 0) { fprintf(stderr, "DumpEE failed\n"); return 1; }

    static paramsMLX90641 p;
    if (MLX90641_ExtractParameters(ee, &p) != 0) {
        fprintf(stderr, "ExtractParameters failed (eeData[10]=0x%04X)\n", ee[10]);
        return 1;
    }

    float emissivity = MLX90641_GetEmissivity(&p);
    if (emissivity <= 0.0f || emissivity > 1.0f) emissivity = 0.95f;

    fprintf(stderr, "deviceSelect(eeData[10])=0x%04X emissivity=%.3f frames=%d\n",
            ee[10], emissivity, frames);

    static uint16_t frame[242];
    static float to[192];

    for (int f = 0; f < frames; f++) {
        if (MLX90641_GetFrameData(ADDR, frame) < 0) {
            fprintf(stderr, "frame %d: read failed\n", f);
            continue;
        }
        float vdd = MLX90641_GetVdd(frame, &p);
        float ta  = MLX90641_GetTa(frame, &p);
        MLX90641_CalculateTo(frame, &p, emissivity, ta, to);

        fprintf(stderr, "frame %d: Vdd=%.2f Ta=%.2f subpage=%d\n", f, vdd, ta, frame[241]);

        for (int r = 0; r < 12; r++) {
            for (int c = 0; c < 16; c++) {
                if (c) putchar(',');
                printf("%.2f", to[r * 16 + c]);
            }
            putchar('\n');
        }
        putchar('\n');                 // block separator
        fflush(stdout);
        if (f + 1 < frames) usleep((useconds_t)delay_ms * 1000);
    }
    return 0;
}
