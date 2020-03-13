#ifndef __DEBUG_LOGS_H__
#define __DEBUG_LOGS_H__

#if DEBUG_LOGS > 0
    #define LOG(f_ , ...) printf((f_), ##__VA_ARGS__)
#else
    #define LOG(f_ , ...) do {} while (0)
#endif

#endif /* __DEBUG_LOGS_H__ */