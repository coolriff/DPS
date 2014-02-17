#ifndef __IntermediateTutorial5_h_
#define __IntermediateTutorial5_h_
 
#include "BaseApplication.h"
 
class IntermediateTutorial5 : public BaseApplication
{
public:
    IntermediateTutorial5(void);
    virtual ~IntermediateTutorial5(void);
 
protected:
    virtual void createScene(void);
	virtual void createGrassMesh(void);
};
 
#endif // #ifndef __IntermediateTutorial5_h_