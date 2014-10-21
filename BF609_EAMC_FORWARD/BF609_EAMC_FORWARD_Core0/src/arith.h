/*
 * arith.h
 *
 *  Created on: 2014-7-30
 *      Author: Administrator
 */

#ifndef ARITH_H_
#define ARITH_H_

/**
* \brief Time structure to handle  time information
 */
typedef struct
{
	int seconds;
	int nanoseconds;
} TimeInternal;

#pragma inline
void
normalizeTime ( TimeInternal *r )
{
	r->seconds += r->nanoseconds / 1000000000;
	r->nanoseconds -= ( r->nanoseconds / 1000000000 ) * 1000000000;

	if ( r->seconds > 0 && r->nanoseconds < 0 )
	{
		r->seconds -= 1;
		r->nanoseconds += 1000000000;
	}

	else if ( r->seconds < 0 && r->nanoseconds > 0 )
	{
		r->seconds += 1;
		r->nanoseconds -= 1000000000;
	}
}
#pragma inline
void
addTime ( TimeInternal *r, const TimeInternal *x, const TimeInternal *y )
{
	r->seconds = x->seconds + y->seconds;
	r->nanoseconds = x->nanoseconds + y->nanoseconds;

	normalizeTime ( r );
}
#pragma inline
void
subTime ( TimeInternal *r, const TimeInternal *x, const TimeInternal *y )
{
	r->seconds = x->seconds - y->seconds;
	r->nanoseconds = x->nanoseconds - y->nanoseconds;

	normalizeTime ( r );
}

#endif /* ARITH_H_ */
