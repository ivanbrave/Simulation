#ifndef __MESSAGE_HPP__
#define __MESSAGE_HPP__

#include <stdio.h>
#include <stdlib.h>

PRJ_BEGIN

/// STR(X): turns X into a string literature
/// XSTR(X): turns X into string of expansion of macro X
#define STR(X) #X
#define XSTR(X) STR(X)

#ifdef MESSAGE_ON
#define ASSERT(x) \
  do{if(!(x)){ \
    fprintf(stderr,"[FAILED: "STR(x)"] "__FILE__":%d\n",__LINE__); \
    fflush(stderr); \
    exit(EXIT_FAILURE); \
  }}while(0)
#define ASSERT_MSG(x,msg) \
  do{if(!(x)){ \
    fprintf(stderr,"[FAILED: "STR(x)"] "__FILE__":%d\n",__LINE__); \
    fprintf(stderr,"> "msg"\n"); \
    fflush(stderr); \
    exit(EXIT_FAILURE); \
  }}while(0)
#define ASSERT_MSGV(x,fmt,...) \
  do{if(!(x)){ \
    fprintf(stderr,"[FAILED: "STR(x)"] "__FILE__":%d\n",__LINE__); \
    fprintf(stderr,"> "fmt"\n",__VA_ARGS__); \
    fflush(stderr); \
    exit(EXIT_FAILURE); \
  }}while(0)
#define WARN(msg) \
  do{ \
    fprintf(stderr,"[WARNING] "msg"\n"); \
    fflush(stderr); \
  }while(0)
#define WARNV(fmt,...) \
  do{ \
    fprintf(stderr,"[WARNING] "fmt"\n",__VA_ARGS__); \
    fflush(stderr); \
  }while(0)
#define INFO(msg) \
  do{ \
    fprintf(stderr,"[INFO] "msg"\n"); \
    fflush(stderr); \
  }while(0)
#define INFOV(fmt,...) \
  do{ \
    fprintf(stderr,"[INFO] "fmt"\n",__VA_ARGS__); \
    fflush(stderr); \
  }while(0)
#else
#define ASSERT(x) do{x;}while(0)
#define ASSERT_MSG(x,msg) do{x;}while(0)
#define ASSERT_MSGV(x,fmt,...) do{x;}while(0)
#define WARN(msg) do{}while(0)
#define WARNV(fmt,...) do{}while(0)
#define INFO(msg) do{}while(0)
#define INFOV(fmt,...) do{}while(0)
#endif

PRJ_END

#endif //__MESSAGE_HPP__
