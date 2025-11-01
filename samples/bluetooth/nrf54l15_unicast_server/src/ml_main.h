#ifndef ML_MAIN_H_
#define ML_MAIN_H_
#ifdef __cplusplus
#define EXTERNC extern "C"
#else
#define EXTERNC
#endif

EXTERNC int ml_init(void);
EXTERNC int ml_process(void* buffer, size_t size);

#undef EXTERNC
#endif /* ML_MAIN_H_ */