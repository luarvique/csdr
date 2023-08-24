#ifndef SHIMS_H                                                                                                                                                                  
#define SHIMS_H

#define _DARWIN_C_SOURCE
#define MSG_NOSIGNAL 0x2000 /* don't raise SIGPIPE */
#define F_SETPIPE_SZ 0x407

#endif
