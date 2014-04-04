 /*
00002  * Orca-Robotics Project: Components for robotics 
00003  *               http://orca-robotics.sf.net/
00004  * Copyright (c) 2004-2009 Alex Brooks, Alexei Makarenko, Tobias Kaupp
00005  *
00006  * This distribution is licensed to you under the terms described in
00007  * the LICENSE file included in this distribution.
00008  *
00009  */

#ifndef HYDROUTIL_UNCOPYABLE_H
#define HYDROUTIL_UNCOPYABLE_H

namespace hydroutil {

	
class Uncopyable
{
public:
    Uncopyable() {}
private:
    Uncopyable(const Uncopyable&);
    void operator=(const Uncopyable&);
};

}

#endif