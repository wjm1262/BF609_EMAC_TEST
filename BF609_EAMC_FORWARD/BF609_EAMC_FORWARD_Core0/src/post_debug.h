/*
 *
 * (C) Copyright 2012 - Analog Devices, Inc.  All rights reserved.
 *
 * FILE:     post_debug.h ( )
 *
 * CHANGES:  1.00.0  - initial release
 */

#ifndef __POST_DEBUG_H__
#define __POST_DEBUG_H__

/**************************************************************
 * defining one of these methods will enabling debug printing *
 **************************************************************/
//#define __DEBUG_FILE__	/* prints are directed to file __DEBUG_FILE_NAME__ */
#define __DEBUG_UART__	/* prints are directed to the UART */
//#define __DEBUG_CCES__		/* prints are directed to the CCES console window (MUCH SLOWER!!!) */
/**************************************************************/

#define TEST_PASS				1
#define TEST_FAIL				0
#define TEST_WARNING			2

#if defined(__DEBUG_UART__)
	#define _NORMALTEXT_		"\e[0m"
	#define _BOLDTEXT_			"\e[1m"
	#define _ITALICTEXT_		"\e[3m"
	#define _BLINKTEXT_			"\e[5m"
	#define _REDTEXT_			"\e[31m"
	#define _GREENTEXT_			"\e[32m"
	#define _YELLOWTEXT_		"\e[33m"
	#define _BLUETEXT_			"\e[34m"
	#define _MAGENTATEXT_		"\e[35m"
	#define _CYANTEXT_			"\e[36m"
	#define _WHITETEXT_			"\e[37m"
	#define _BLACKTEXT_			"\e[30m"
#else
	#define _NORMALTEXT_		
	#define _BOLDTEXT_			
	#define _ITALICTEXT_		
	#define _BLINKTEXT_			
	#define _REDTEXT_			
	#define _GREENTEXT_			
	#define _YELLOWTEXT_		
	#define _BLUETEXT_			
	#define _MAGENTATEXT_		
	#define _CYANTEXT_			
	#define _WHITETEXT_			
	#define _BLACKTEXT_			
#endif

#define DO_CYCLE_COUNTS		/* used in cycles.h, do not rename this define - calculate cycle counts for tests */

/* if debug printing is enabled */
#if defined(__DEBUG_FILE__) || defined(__DEBUG_UART__) || defined(__DEBUG_CCES__)

	#include <stdio.h>

	/* if printing to a file */
    #if defined(__DEBUG_FILE__)
        #undef _PRINT_CYCLES
        #define __DEBUG_FILE_NAME__ "post_debug.txt"	/* output file name */
        #define DEBUG_STREAM pDebugFile
        #define  _PRINT_CYCLES(_STRG, _DAT)   fprintf(DEBUG_STREAM,"%s%llu\n", (_STRG), (_DAT)) __TRAILING_SC__
        #define DEBUG_CLOSE()	fclose(DEBUG_STREAM)
        #define DEBUG_PRINT(fmt,...) fprintf(DEBUG_STREAM,fmt, __VA_ARGS__);
    	#define DEBUG_STATEMENT(statement) fprintf(DEBUG_STREAM, statement);

	/* else if printing to the UART */
	#elif defined(__DEBUG_UART__)
		#define UART_DEBUG_BUFFER_LINE_SIZE 256
		extern char UART_DEBUG_BUFFER[];
		int UART_DEBUG_PRINT(void);
		#define DEBUG_PRINT(fmt,...)		snprintf(UART_DEBUG_BUFFER, UART_DEBUG_BUFFER_LINE_SIZE, fmt, __VA_ARGS__); \
											UART_DEBUG_PRINT();
    	#define DEBUG_STATEMENT(statement)	snprintf(UART_DEBUG_BUFFER, UART_DEBUG_BUFFER_LINE_SIZE, statement); \
    										UART_DEBUG_PRINT();

	/* else printing to CCES console window */
    #elif defined(__DEBUG_CCES__)
        #define DEBUG_STREAM stdout
        #define DEBUG_CLOSE
        #define DEBUG_PRINT(fmt,...) fprintf(DEBUG_STREAM,fmt, __VA_ARGS__);
    	#define DEBUG_STATEMENT(statement) fprintf(DEBUG_STREAM, statement);
    #endif

/* else debug printing not defined */
#else

	#ifdef DO_CYCLE_COUNTS
        #include <stdio.h>
    #endif

    #undef  __DEBUG_FILE__
    #define DEBUG_PRINT(fmt,...)
    #define DEBUG_STATEMENT(statement)
#endif

/* every method can use these defines */

#define DEBUG_HEADER(header)  		DEBUG_STATEMENT("\n\n"); \
   									DEBUG_STATEMENT(""_BOLDTEXT_"*******************************************************\n"); \
                               		DEBUG_STATEMENT("*** "); \
                               		DEBUG_STATEMENT(""_BOLDTEXT_""_YELLOWTEXT_""header""_NORMALTEXT_"\n"); \
                               		DEBUG_STATEMENT(""_BOLDTEXT_"********************************************************"_NORMALTEXT_"\n");
#define DEBUG_SUBHEADER(subheader)	DEBUG_STATEMENT("\n\n"); \
									DEBUG_STATEMENT("\n"_BOLDTEXT_"----------------------------------------"); \
                               		DEBUG_STATEMENT("\n--- "); \
                               		DEBUG_STATEMENT(subheader); \
                               		DEBUG_STATEMENT("\n----------------------------------------"_NORMALTEXT_"");
#define DEBUG_SUBHEADER2(subheader2)DEBUG_STATEMENT("\n\n"); \
									DEBUG_STATEMENT("\n"_BOLDTEXT_"----------------------------"); \
                               		DEBUG_STATEMENT("\n- "); \
                               		DEBUG_STATEMENT(subheader2); \
                               		DEBUG_STATEMENT("\n----------------------------"_NORMALTEXT_"");

#define DEBUG_RESULT(result, message)		if(result == TEST_PASS)\
												{if(message != "")\
													{DEBUG_STATEMENT("\n"_BOLDTEXT_""_GREENTEXT_"Test Passed " ); \
													DEBUG_STATEMENT(message""_NORMALTEXT_"");} \
												else \
													{DEBUG_STATEMENT("\n"_BOLDTEXT_""_GREENTEXT_"Test Passed"_NORMALTEXT_"");}} \
											else if (result == TEST_FAIL)\
												{if(message != "")\
													{DEBUG_STATEMENT("\n"_BOLDTEXT_""_REDTEXT_"Test Failed " ); \
													DEBUG_STATEMENT(message""_NORMALTEXT_"");} \
												else \
													{DEBUG_STATEMENT("\n"_BOLDTEXT_""_REDTEXT_"Test Failed"_NORMALTEXT_"" );}} \
											else if (result == TEST_WARNING)\
												{if(message != "")\
													{DEBUG_STATEMENT("\n"_BOLDTEXT_""_YELLOWTEXT_"Warning: " ); \
													DEBUG_STATEMENT(message""_NORMALTEXT_"");} \
												else \
													{DEBUG_STATEMENT("\n"_BOLDTEXT_""_YELLOWTEXT_"Warning"_NORMALTEXT_"" );}} \
											else \
											{DEBUG_STATEMENT("\n"_BOLDTEXT_""_MAGENTATEXT_"INCORRECT PARAMETER"_NORMALTEXT_"" );}

#endif /* __POST_DEBUG_H__ */
