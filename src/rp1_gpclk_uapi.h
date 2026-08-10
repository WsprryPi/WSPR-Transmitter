#ifndef WSPRRYPI_RP1_GPCLK_UAPI_H
#define WSPRRYPI_RP1_GPCLK_UAPI_H

#ifdef __linux__
#include <linux/ioctl.h>
#include <linux/types.h>
#else
#include <stdint.h>
#include <sys/ioctl.h>
typedef uint16_t __u16;
typedef uint32_t __u32;
typedef uint64_t __u64;
typedef uint8_t __u8;
#endif

#define RP1_GPCLK_UAPI_VERSION 1U
#define RP1_GPCLK_IOC_MAGIC 0xb7
#define RP1_GPCLK_WRITES_PER_SYMBOL 66792U
#define RP1_GPCLK_WSPR_SYMBOL_COUNT 162U
#define RP1_GPCLK_TICK_DIVIDER 511U

enum rp1_gpclk_state {
	RP1_GPCLK_STATE_IDLE = 0,
	RP1_GPCLK_STATE_RUNNING = 1,
	RP1_GPCLK_STATE_DRAINING = 2,
	RP1_GPCLK_STATE_COMPLETE = 3,
	RP1_GPCLK_STATE_FAILED = 4,
};

struct rp1_gpclk_acquire {
	__u16 version;
	__u16 size;
	__u32 drive_ma;
	__u32 flags;
	__u32 reserved;
};

struct rp1_gpclk_symbol {
	__u64 lower_divider_word;
	__u64 upper_divider_word;
	__u32 lower_count;
	__u32 upper_count;
};

struct rp1_gpclk_program {
	__u16 version;
	__u16 size;
	__u32 fractional_bits;
	__u32 writes_per_symbol;
	__u32 tick_divider;
	__u32 symbol_count;
	__u32 tone_count;
	__u32 reserved;
	__u64 generation;
	struct rp1_gpclk_symbol tones[4];
	__u8 symbols[RP1_GPCLK_WSPR_SYMBOL_COUNT];
	__u8 reserved_tail[6];
};

struct rp1_gpclk_generation {
	__u16 version;
	__u16 size;
	__u32 state;
	__u64 generation;
};

#define RP1_GPCLK_IOC_ACQUIRE _IOW(RP1_GPCLK_IOC_MAGIC, 0x00, struct rp1_gpclk_acquire)
#define RP1_GPCLK_IOC_SUBMIT _IOW(RP1_GPCLK_IOC_MAGIC, 0x01, struct rp1_gpclk_program)
#define RP1_GPCLK_IOC_STOP _IOW(RP1_GPCLK_IOC_MAGIC, 0x02, struct rp1_gpclk_generation)
#define RP1_GPCLK_IOC_STATE _IOWR(RP1_GPCLK_IOC_MAGIC, 0x03, struct rp1_gpclk_generation)
#define RP1_GPCLK_IOC_RELEASE _IO(RP1_GPCLK_IOC_MAGIC, 0x04)

#endif
