#ifndef DAQ_DRIVER_H
#define DAQ_DRIVER_H

#ifdef __KERNEL__
#include <linux/types.h>
#include <linux/ioctl.h>
#else
#include <stdint.h>
#include <sys/ioctl.h>
#endif

#define DAQ_DEVICE_NAME     "rt_daq"
#define DAQ_CLASS_NAME      "daq"
#define DAQ_MAGIC_NUMBER    'q'

/*
 * Sensor data frame transmitted from kernel ring buffer to user space.
 * Total size: 24 bytes (strictly 64-bit aligned).
 */
struct daq_frame {
    uint64_t timestamp_ns;  /* 8 bytes Nanosecond timestamp from ktime_get_ns() */
    uint32_t sequence_id;   /* 4 bytes Incrementing packet counter (detects loss) */
    int32_t  raw_value;     /* 4 bytes Scaled analog or simulated sensor reading */
    uint16_t channel_id;    /* 2 bytes Hardware ADC or sensor channel (0, 1, 2...) */
    uint16_t status_flags;  /* 2 bytes Error bits: 0x00=OK, 0x01=Overrun, etc. */
    uint32_t checksum;      /* 4 bytes Simple XOR or Fletcher checksum */
};

/*
 * Diagnostic telemetry returned by DAQ_IOCTL_GET_STATS.
 */
struct daq_stats {
    uint64_t total_samples_generated;
    uint64_t samples_read_by_user;
    uint64_t samples_dropped;
    uint32_t current_buffer_fill;
    uint32_t max_buffer_capacity;
};

/*
 * IOCTL Command Definitions
 * Formula: _IO / _IOR / _IOW / _IOWR (Magic, Command Number, DataType)
 */

/* Flush and reset the circular ring buffer (No data transferred) */
#define DAQ_IOCTL_RESET_BUFFER   _IO(DAQ_MAGIC_NUMBER, 1)

/* Configure sample interval in microseconds (User writes uint32_t to Kernel) */
#define DAQ_IOCTL_SET_SAMPLE_US  _IOW(DAQ_MAGIC_NUMBER, 2, uint32_t)

/* Query runtime driver statistics (Kernel writes struct daq_stats to User) */
#define DAQ_IOCTL_GET_STATS      _IOR(DAQ_MAGIC_NUMBER, 3, struct daq_stats)

/* Enable or disable data acquisition stream (User writes uint32_t: 1=start, 0=stop) */
#define DAQ_IOCTL_STREAM_CONTROL _IOW(DAQ_MAGIC_NUMBER, 4, uint32_t)

#endif /* DAQ_DRIVER_H */
