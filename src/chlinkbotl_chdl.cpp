#include<ch.h>
#include<math.h>
#include<stdio.h>
#include <string.h>
#include<functional>
#include "linkbot_wrapper.h"

#ifdef _WIN32
#include <windows.h>
#include <Shlobj.h>
#endif

#define unimplemented() \
fprintf(stderr, "Function %s is currently unimplemented.\n", __func__); \
exit(-1)

extern int linkbot_count;

/*class creator*/
EXPORTCH void CLinkbotL_CLinkbotL_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
	const char* serialId;
	class LinkbotWrapper *l;
    
    Ch_VaStart(interp, ap, varg);
	if (Ch_VaCount(interp, ap) == 0){
		l = newLinkbotLWrapper(nullptr);
		Ch_CppChangeThisPointer(interp, l, sizeof(LinkbotWrapper));
	}
	else if (Ch_VaCount(interp, ap) == 1){
		serialId = Ch_VaArg(interp, ap, const char *);
		l = newLinkbotLWrapper(serialId);
		Ch_CppChangeThisPointer(interp, l, sizeof(LinkbotWrapper));
	}
	else {
		printf("Wrong number of argument passed\n");
	}
	Ch_VaEnd(interp, ap);
	return;
}
 
/* class destructor*/   
EXPORTCH void CLinkbotL_dCLinkbotL_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class LinkbotWrapper *l;
     
    Ch_VaStart(interp, ap, varg);
    l=Ch_VaArg(interp, ap, class LinkbotWrapper *);
    if(Ch_CppIsArrayElement(interp))
        l->~LinkbotWrapper();
    else
        delete l;
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbot connect*/
EXPORTCH int CLinkbotL_connect_chdl(void *varg) {
    unimplemented();
#if 0
    ChInterp_t interp;
    ChVaList_t ap;
    class LinkbotWrapper *l;
	int type;
    int rc;
    Ch_VaStart(interp, ap, varg);
   
    l=Ch_VaArg(interp, ap, class LinkbotWrapper *);
    rc = l->connect();
    Ch_VaEnd(interp, ap);
    l->getFormFactor(type);
	if (type == 0)
	{
		printf("WARNING: A LinkbotWrapper-I is connected, not a LinkbotWrapper-L.\nPlease connect a Linbot-L.\nExiting...\n");
		exit(-1);
	}
    return rc;
#endif
}

/*linkbot connectWithSerialID*/
EXPORTCH int CLinkbotL_connectWithSerialID_chdl(void *varg) {
    unimplemented();
#if 0
    ChInterp_t interp;
    ChVaList_t ap;
    class LinkbotWrapper *l;
    const char *id;
	int type;
    int rc;
    Ch_VaStart(interp, ap, varg);
   
    l=Ch_VaArg(interp, ap, class LinkbotWrapper *);
    id = Ch_VaArg(interp, ap, const char*);
    rc = l->connectWithSerialID(id);
    Ch_VaEnd(interp, ap);
    l->getFormFactor(type);
	if (type == 0)
	{
		printf("WARNING: A barobo::CLinkbotL-I is connected, not a barobo::CLinkbotL-L.\nPlease connect a Linbot-L.\nExiting...\n");
		exit(-1);
	}
    return rc;
#endif
}

/*linkbot disconnect*/
EXPORTCH void CLinkbotL_disconnect_chdl(void *varg) {
    unimplemented();
#if 0
    ChInterp_t interp;
    ChVaList_t ap;
    class LinkbotWrapper *l;
    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class LinkbotWrapper *);
    l->disconnect();
    Ch_VaEnd(interp, ap);
    return;
#endif
}

/*MOVEMENT FUNCTIONS*/

/*END MOVEMENT FUNCTIONS*/
/*GET FUNCTIONS*/

/*END SET FUNCTIONS*/
/*MISCELLANEOUS FUNCTIONS*/

/* CLinkbotLGroup functions */

/*Constructor*/
EXPORTCH void CLinkbotLGroup_CLinkbotLGroup_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;

  Ch_VaStart(interp, ap, varg);
  auto *c=new barobo::CLinkbotLGroup();
  Ch_CppChangeThisPointer(interp, c, sizeof(barobo::CLinkbotLGroup));
  Ch_VaEnd(interp, ap);
}

/*Destructor*/
EXPORTCH void CLinkbotLGroup_dCLinkbotLGroup_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  barobo::CLinkbotLGroup *c;
  
  Ch_VaStart(interp, ap, varg);
  c = Ch_VaArg(interp, ap, barobo::CLinkbotLGroup *);
  if(Ch_CppIsArrayElement(interp)){
    c->~CLinkbotLGroup();
  }
  else {
    delete c;
  }
  Ch_VaEnd(interp, ap);
  return;
}

