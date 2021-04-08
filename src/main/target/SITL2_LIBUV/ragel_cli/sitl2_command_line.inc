
/* #line 1 "sitl2_command_line.rl" */
#ifdef TEST_RAGEL_PARSER

#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#endif

#define MAX_CALL_PARAMS 3


/* #line 15 "sitl2_command_line.inc" */
static const int sitl2_command_line_start = 1;
static const int sitl2_command_line_first_final = 13;
static const int sitl2_command_line_error = 0;

static const int sitl2_command_line_en_main = 1;


/* #line 14 "sitl2_command_line.rl" */


#ifdef TEST_RAGEL_PARSER

static void for_debug(void)
{
  printf("for_debug\n");
}

static int parse_sitl2_cli_command(const char *data, int length, int *is_exit)
#else
static int parse_sitl2_cli_command(servo_usb_control_context_t *ctx, const char *data, int length) 
#endif
{

    //int rc;
    const char *p = data, *pe = data + length;
    const char *eof = pe;
    const char *start = data;

    int tmp;
    int sign = 1;
   
    int params[MAX_CALL_PARAMS] = {0};
    int param_num = 0;

    int cs;

    
/* #line 53 "sitl2_command_line.inc" */
	{
	cs = sitl2_command_line_start;
	}

/* #line 58 "sitl2_command_line.inc" */
	{
	if ( p == pe )
		goto _test_eof;
	switch ( cs )
	{
case 1:
	switch( (*p) ) {
		case 47: goto st2;
		case 101: goto st3;
		case 113: goto st15;
		case 115: goto st7;
	}
	goto tr0;
tr0:
/* #line 85 "sitl2_command_line.rl" */
	{ 
#ifdef TEST_RAGEL_PARSER
            printf(" error at %d \"%s\"\n", (int)(p-start), p); 
#endif
            //return -1;
        }
	goto st0;
/* #line 81 "sitl2_command_line.inc" */
st0:
cs = 0;
	goto _out;
st2:
	if ( ++p == pe )
		goto _test_eof2;
case 2:
	if ( (*p) == 47 )
		goto st13;
	goto tr0;
st13:
	if ( ++p == pe )
		goto _test_eof13;
case 13:
	goto st13;
st3:
	if ( ++p == pe )
		goto _test_eof3;
case 3:
	if ( (*p) == 120 )
		goto st4;
	goto tr0;
st4:
	if ( ++p == pe )
		goto _test_eof4;
case 4:
	if ( (*p) == 105 )
		goto st5;
	goto tr0;
st5:
	if ( ++p == pe )
		goto _test_eof5;
case 5:
	if ( (*p) == 116 )
		goto st14;
	goto tr0;
st14:
	if ( ++p == pe )
		goto _test_eof14;
case 14:
	switch( (*p) ) {
		case 32: goto st14;
		case 40: goto st6;
	}
	if ( 9 <= (*p) && (*p) <= 13 )
		goto st14;
	goto tr0;
st6:
	if ( ++p == pe )
		goto _test_eof6;
case 6:
	switch( (*p) ) {
		case 32: goto st6;
		case 41: goto st15;
	}
	if ( 9 <= (*p) && (*p) <= 13 )
		goto st6;
	goto tr0;
st15:
	if ( ++p == pe )
		goto _test_eof15;
case 15:
	if ( (*p) == 32 )
		goto st15;
	if ( 9 <= (*p) && (*p) <= 13 )
		goto st15;
	goto tr0;
st7:
	if ( ++p == pe )
		goto _test_eof7;
case 7:
	if ( (*p) == 116 )
		goto st8;
	goto tr0;
st8:
	if ( ++p == pe )
		goto _test_eof8;
case 8:
	if ( (*p) == 97 )
		goto st9;
	goto tr0;
st9:
	if ( ++p == pe )
		goto _test_eof9;
case 9:
	if ( (*p) == 116 )
		goto st10;
	goto tr0;
st10:
	if ( ++p == pe )
		goto _test_eof10;
case 10:
	if ( (*p) == 117 )
		goto st11;
	goto tr0;
st11:
	if ( ++p == pe )
		goto _test_eof11;
case 11:
	if ( (*p) == 115 )
		goto st16;
	goto tr0;
st16:
	if ( ++p == pe )
		goto _test_eof16;
case 16:
	switch( (*p) ) {
		case 32: goto st16;
		case 40: goto st12;
	}
	if ( 9 <= (*p) && (*p) <= 13 )
		goto st16;
	goto tr0;
st12:
	if ( ++p == pe )
		goto _test_eof12;
case 12:
	switch( (*p) ) {
		case 32: goto st12;
		case 41: goto st17;
	}
	if ( 9 <= (*p) && (*p) <= 13 )
		goto st12;
	goto tr0;
st17:
	if ( ++p == pe )
		goto _test_eof17;
case 17:
	if ( (*p) == 32 )
		goto st17;
	if ( 9 <= (*p) && (*p) <= 13 )
		goto st17;
	goto tr0;
	}
	_test_eof2: cs = 2; goto _test_eof; 
	_test_eof13: cs = 13; goto _test_eof; 
	_test_eof3: cs = 3; goto _test_eof; 
	_test_eof4: cs = 4; goto _test_eof; 
	_test_eof5: cs = 5; goto _test_eof; 
	_test_eof14: cs = 14; goto _test_eof; 
	_test_eof6: cs = 6; goto _test_eof; 
	_test_eof15: cs = 15; goto _test_eof; 
	_test_eof7: cs = 7; goto _test_eof; 
	_test_eof8: cs = 8; goto _test_eof; 
	_test_eof9: cs = 9; goto _test_eof; 
	_test_eof10: cs = 10; goto _test_eof; 
	_test_eof11: cs = 11; goto _test_eof; 
	_test_eof16: cs = 16; goto _test_eof; 
	_test_eof12: cs = 12; goto _test_eof; 
	_test_eof17: cs = 17; goto _test_eof; 

	_test_eof: {}
	if ( p == eof )
	{
	switch ( cs ) {
	case 1: 
	case 2: 
	case 3: 
	case 4: 
	case 5: 
	case 6: 
	case 7: 
	case 8: 
	case 9: 
	case 10: 
	case 11: 
	case 12: 
/* #line 85 "sitl2_command_line.rl" */
	{ 
#ifdef TEST_RAGEL_PARSER
            printf(" error at %d \"%s\"\n", (int)(p-start), p); 
#endif
            //return -1;
        }
/* #line 92 "sitl2_command_line.rl" */
	{ 
#ifdef TEST_RAGEL_PARSER
            printf(" eof at %d\n", (int)(p-start));
#endif
        }
	break;
	case 16: 
	case 17: 
/* #line 99 "sitl2_command_line.rl" */
	{ 
#ifdef TEST_RAGEL_PARSER
            printf(" STATUS\n");
#else
            sitl2_cli_STATUS(ctx);
#endif
        }
/* #line 92 "sitl2_command_line.rl" */
	{ 
#ifdef TEST_RAGEL_PARSER
            printf(" eof at %d\n", (int)(p-start));
#endif
        }
	break;
	case 14: 
	case 15: 
/* #line 107 "sitl2_command_line.rl" */
	{ 
#ifdef TEST_RAGEL_PARSER
            printf(" EXIT\n");
            *is_exit = 1;
#else
            sitl2_cli_EXIT(ctx);
#endif
        }
/* #line 92 "sitl2_command_line.rl" */
	{ 
#ifdef TEST_RAGEL_PARSER
            printf(" eof at %d\n", (int)(p-start));
#endif
        }
	break;
	case 13: 
/* #line 117 "sitl2_command_line.rl" */
	{ 
#ifdef TEST_RAGEL_PARSER
            printf(" COMMENT\n");
#else
#endif
        }
/* #line 92 "sitl2_command_line.rl" */
	{ 
#ifdef TEST_RAGEL_PARSER
            printf(" eof at %d\n", (int)(p-start));
#endif
        }
	break;
/* #line 313 "sitl2_command_line.inc" */
	}
	}

	_out: {}
	}

/* #line 155 "sitl2_command_line.rl" */


#ifdef TEST_RAGEL_PARSER
    printf(" cs: %d\n", cs);
#endif

    if(cs==sitl2_command_line_error){
       printf(" error\n");
       return -1;
    }

    if(cs>=sitl2_command_line_first_final){
#ifdef TEST_RAGEL_PARSER
       printf(" ok\n");
#endif

       return 0;
    }
    printf("unknown command\n");


    return -2;
};


#ifdef TEST_RAGEL_PARSER


#define BUFSIZE 1024

int main()
{
    int rc;
    int is_exit=0;
    char buf[BUFSIZE];

    while ( fgets( buf, sizeof(buf), stdin ) != 0 ) {
        printf( "buf:%s", buf);
        rc = parse_sitl2_cli_command(buf, strlen(buf), &is_exit);
        printf( "rc:%d\n\n", rc );
        if(is_exit){
           break;
        }
    }
    return 0;
}

#endif