#ifndef __SYSTEM_H
#define __SYSTEM_H

#ifdef __cplusplus
extern "C" {
#endif

void flush_cpu_icache(void);
void flush_cpu_dcache(void);
void flush_l2_cache(void);

void busy_wait(unsigned int ms);

#ifdef __cplusplus
}
#endif

#endif /* __SYSTEM_H */
