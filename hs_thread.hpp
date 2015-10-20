#ifndef _HS_THREAD_HPP__
#define _HS_THREAD_HPP__

#include "hsDataStruct.hpp"


void *thread_udp(void *arg);
void *thread_serial(void *arg);
void *thread_cv(void *arg);
void *thread_file(void *arg);
void generateFileName(char *f_name);

#endif
