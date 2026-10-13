# Real Time Edge Data Acquasition & Telementry Subsystem (RT DAQ)

An industrial grade embedded Linux data acquisition subsytem featuring:
- **Kernel Space:** High resolution timer (`hrtimer`) character device driver with circular ring buffering, wait queues, and `ioctl` interface.
- **User Space:** Real time multi threaded c++17 daemon utilizing `SCHED_FIFO` POSIX scheduling and lock free concurrency.
- **Diagnostics:** Latency and jitter benchmarking under heavy CPU and memory stress.

## Project Structure
- `driver/` : Linux kernel character device module.
- `daemon/` : Real time user space C++ telementary service.
- `benchmarks/` : Jitter measurement scripts and latency analysis tools.
