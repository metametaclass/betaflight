
/* #line 1 "sitl2_command_line.rl" */
#ifdef TEST_RAGEL_PARSER

#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#define WMQE_INVALID_CLI_COMMAND -1
#define WMQE_PARSE_ERROR -1

#endif

#define SITL2_CLI_MAX_CALL_PARAMS 3
#define SITL2_CLI_STACK_SIZE 16


/* #line 19 "sitl2_command_line.inc" */
static const unsigned char _sitl2_command_line_key_offsets[] = {
	0, 0, 6, 7, 8, 9, 13, 14, 
	15, 16, 17, 18, 19, 23, 25, 26, 
	27, 28, 29, 30, 31, 32, 32, 36, 
	39, 43, 47, 50, 54, 58, 61, 65, 
	71, 75, 78, 83, 86, 95, 98, 103, 
	107, 111, 115, 119, 123, 127, 131, 135, 
	139, 143, 147, 151, 155, 159, 163, 167, 
	171, 175, 179, 183
};

static const char _sitl2_command_line_trans_keys[] = {
	47, 101, 104, 113, 115, 119, 47, 105, 
	116, 32, 41, 9, 13, 108, 112, 116, 
	116, 117, 115, 32, 41, 9, 13, 41, 
	44, 116, 99, 104, 115, 116, 111, 112, 
	32, 120, 9, 13, 32, 9, 13, 32, 
	40, 9, 13, 32, 101, 9, 13, 32, 
	9, 13, 32, 117, 9, 13, 32, 97, 
	9, 13, 32, 9, 13, 32, 40, 9, 
	13, 32, 40, 97, 115, 9, 13, 32, 
	40, 9, 13, 32, 9, 13, 32, 40, 
	95, 9, 13, 32, 9, 13, 41, 42, 
	44, 61, 97, 109, 114, 115, 116, 41, 
	44, 61, 41, 44, 61, 108, 114, 41, 
	44, 61, 108, 41, 44, 61, 109, 41, 
	44, 61, 111, 41, 44, 61, 116, 41, 
	44, 61, 111, 41, 44, 61, 114, 41, 
	44, 61, 115, 41, 44, 61, 97, 41, 
	44, 61, 116, 41, 44, 61, 101, 41, 
	44, 61, 99, 41, 44, 61, 104, 41, 
	44, 61, 101, 41, 44, 61, 100, 41, 
	44, 61, 117, 41, 44, 61, 108, 41, 
	44, 61, 101, 41, 44, 61, 114, 41, 
	44, 61, 105, 41, 44, 61, 109, 41, 
	44, 61, 101, 0
};

static const char _sitl2_command_line_single_lengths[] = {
	0, 6, 1, 1, 1, 2, 1, 1, 
	1, 1, 1, 1, 2, 2, 1, 1, 
	1, 1, 1, 1, 1, 0, 2, 1, 
	2, 2, 1, 2, 2, 1, 2, 4, 
	2, 1, 3, 1, 9, 3, 5, 4, 
	4, 4, 4, 4, 4, 4, 4, 4, 
	4, 4, 4, 4, 4, 4, 4, 4, 
	4, 4, 4, 4
};

static const char _sitl2_command_line_range_lengths[] = {
	0, 0, 0, 0, 0, 1, 0, 0, 
	0, 0, 0, 0, 1, 0, 0, 0, 
	0, 0, 0, 0, 0, 0, 1, 1, 
	1, 1, 1, 1, 1, 1, 1, 1, 
	1, 1, 1, 1, 0, 0, 0, 0, 
	0, 0, 0, 0, 0, 0, 0, 0, 
	0, 0, 0, 0, 0, 0, 0, 0, 
	0, 0, 0, 0
};

static const short _sitl2_command_line_index_offsets[] = {
	0, 0, 7, 9, 11, 13, 17, 19, 
	21, 23, 25, 27, 29, 33, 36, 38, 
	40, 42, 44, 46, 48, 50, 51, 55, 
	58, 62, 66, 69, 73, 77, 80, 84, 
	90, 94, 97, 102, 105, 115, 119, 125, 
	130, 135, 140, 145, 150, 155, 160, 165, 
	170, 175, 180, 185, 190, 195, 200, 205, 
	210, 215, 220, 225
};

static const char _sitl2_command_line_indicies[] = {
	1, 2, 3, 4, 5, 6, 0, 7, 
	0, 8, 0, 9, 0, 10, 11, 10, 
	0, 12, 0, 13, 0, 14, 0, 15, 
	0, 16, 0, 17, 0, 18, 19, 18, 
	0, 20, 21, 0, 22, 0, 23, 0, 
	24, 0, 25, 0, 26, 0, 27, 0, 
	28, 0, 7, 11, 29, 11, 0, 11, 
	11, 0, 9, 10, 9, 0, 13, 30, 
	13, 0, 13, 13, 0, 11, 29, 11, 
	0, 19, 31, 19, 0, 19, 19, 0, 
	17, 18, 17, 0, 32, 33, 34, 28, 
	32, 0, 32, 33, 32, 0, 20, 20, 
	0, 32, 33, 35, 32, 0, 28, 28, 
	0, 38, 37, 38, 38, 39, 40, 41, 
	42, 43, 36, 44, 44, 44, 36, 45, 
	45, 45, 46, 47, 36, 48, 48, 48, 
	49, 36, 48, 48, 48, 50, 36, 51, 
	51, 51, 52, 36, 48, 48, 48, 53, 
	36, 48, 48, 48, 54, 36, 48, 48, 
	48, 55, 36, 48, 48, 48, 56, 36, 
	57, 57, 57, 58, 36, 48, 48, 48, 
	59, 36, 48, 48, 48, 60, 36, 61, 
	61, 61, 62, 36, 48, 48, 48, 63, 
	36, 48, 48, 48, 64, 36, 48, 48, 
	48, 65, 36, 48, 48, 48, 66, 36, 
	48, 48, 48, 67, 36, 48, 48, 48, 
	68, 36, 48, 48, 48, 69, 36, 70, 
	70, 70, 71, 36, 48, 48, 48, 72, 
	36, 48, 48, 48, 73, 36, 0
};

static const char _sitl2_command_line_trans_targs[] = {
	0, 2, 22, 25, 27, 8, 31, 21, 
	4, 24, 5, 23, 7, 26, 28, 10, 
	11, 30, 12, 29, 33, 13, 15, 16, 
	34, 18, 19, 20, 35, 3, 6, 9, 
	32, 13, 14, 17, 37, 37, 0, 38, 
	41, 46, 49, 57, 36, 36, 39, 40, 
	36, 37, 37, 36, 42, 43, 44, 45, 
	37, 36, 47, 48, 37, 36, 50, 51, 
	52, 53, 54, 55, 56, 37, 36, 58, 
	59, 37
};

static const char _sitl2_command_line_trans_actions[] = {
	3, 0, 0, 0, 0, 0, 0, 0, 
	0, 0, 0, 0, 0, 0, 0, 0, 
	0, 0, 0, 0, 0, 4, 0, 0, 
	0, 0, 0, 0, 0, 0, 0, 0, 
	0, 10, 0, 0, 13, 14, 0, 0, 
	0, 0, 0, 0, 15, 16, 0, 0, 
	17, 18, 19, 20, 0, 0, 0, 0, 
	21, 22, 0, 0, 23, 24, 0, 0, 
	0, 0, 0, 0, 0, 25, 26, 0, 
	0, 27
};

static const char _sitl2_command_line_to_state_actions[] = {
	0, 1, 0, 0, 0, 0, 0, 0, 
	0, 0, 0, 0, 0, 1, 0, 0, 
	0, 0, 0, 0, 0, 0, 0, 0, 
	0, 0, 0, 0, 0, 0, 0, 0, 
	0, 0, 0, 0, 1, 0, 0, 0, 
	0, 0, 0, 0, 0, 0, 0, 0, 
	0, 0, 0, 0, 0, 0, 0, 0, 
	0, 0, 0, 0
};

static const char _sitl2_command_line_from_state_actions[] = {
	0, 0, 0, 0, 0, 0, 0, 0, 
	0, 0, 0, 0, 0, 0, 0, 0, 
	0, 0, 0, 0, 0, 0, 0, 0, 
	0, 0, 0, 0, 0, 0, 0, 0, 
	0, 0, 0, 0, 12, 0, 0, 0, 
	0, 0, 0, 0, 0, 0, 0, 0, 
	0, 0, 0, 0, 0, 0, 0, 0, 
	0, 0, 0, 0
};

static const char _sitl2_command_line_eof_actions[] = {
	0, 2, 2, 2, 2, 2, 2, 2, 
	2, 2, 2, 2, 2, 2, 2, 2, 
	2, 2, 2, 2, 2, 5, 6, 6, 
	6, 7, 7, 6, 8, 8, 8, 9, 
	9, 9, 9, 11, 0, 0, 0, 0, 
	0, 0, 0, 0, 0, 0, 0, 0, 
	0, 0, 0, 0, 0, 0, 0, 0, 
	0, 0, 0, 0
};

static const short _sitl2_command_line_eof_trans[] = {
	0, 0, 0, 0, 0, 0, 0, 0, 
	0, 0, 0, 0, 0, 0, 0, 0, 
	0, 0, 0, 0, 0, 0, 0, 0, 
	0, 0, 0, 0, 0, 0, 0, 0, 
	0, 0, 0, 0, 0, 45, 46, 49, 
	49, 52, 49, 49, 49, 49, 58, 49, 
	49, 62, 49, 49, 49, 49, 49, 49, 
	49, 71, 49, 49
};

static const int sitl2_command_line_start = 1;
static const int sitl2_command_line_first_final = 21;
static const int sitl2_command_line_error = 0;

static const int sitl2_command_line_en_watch_param_scanner = 36;
static const int sitl2_command_line_en_main = 1;


/* #line 18 "sitl2_command_line.rl" */


#ifdef TEST_RAGEL_PARSER

/*
static void for_debug(void)
{
  printf("for_debug\n");
}*/

static int parse_sitl2_cli_command(const char *data, size_t length, int *is_exit)
#else
static int parse_sitl2_cli_command(sitl2_cli_context_t *ctx, const char *data, size_t length) 
#endif
{
#ifndef TEST_RAGEL_PARSER
    int rc;
#endif

    const char *p = data, *pe = data + length;
    const char *eof = pe;
    const char *start = data;

    //for scanners
    const char *ts;
    const char *te;
    int act;

    //for stack
    int top;
    int stack[SITL2_CLI_STACK_SIZE];

    //const char *key = data;
    //size_t key_size = 0;
    //int quoted = 0;

//    int tmp;
//    int sign = 1;
   
    // int params[SITL2_CLI_MAX_CALL_PARAMS] = {0};
    // int param_num = 0;

    int cs;

    
/* #line 247 "sitl2_command_line.inc" */
	{
	cs = sitl2_command_line_start;
	top = 0;
	ts = 0;
	te = 0;
	act = 0;
	}

/* #line 256 "sitl2_command_line.inc" */
	{
	int _klen;
	const char *_keys;
	int _trans;

	if ( p == pe )
		goto _test_eof;
	if ( cs == 0 )
		goto _out;
_resume:
	switch ( _sitl2_command_line_from_state_actions[cs] ) {
	case 12:
/* #line 1 "NONE" */
	{ts = p;}
	break;
/* #line 272 "sitl2_command_line.inc" */
	}

	_keys = _sitl2_command_line_trans_keys + _sitl2_command_line_key_offsets[cs];
	_trans = _sitl2_command_line_index_offsets[cs];

	_klen = _sitl2_command_line_single_lengths[cs];
	if ( _klen > 0 ) {
		const char *_lower = _keys;
		const char *_mid;
		const char *_upper = _keys + _klen - 1;
		while (1) {
			if ( _upper < _lower )
				break;

			_mid = _lower + ((_upper-_lower) >> 1);
			if ( (*p) < *_mid )
				_upper = _mid - 1;
			else if ( (*p) > *_mid )
				_lower = _mid + 1;
			else {
				_trans += (unsigned int)(_mid - _keys);
				goto _match;
			}
		}
		_keys += _klen;
		_trans += _klen;
	}

	_klen = _sitl2_command_line_range_lengths[cs];
	if ( _klen > 0 ) {
		const char *_lower = _keys;
		const char *_mid;
		const char *_upper = _keys + (_klen<<1) - 2;
		while (1) {
			if ( _upper < _lower )
				break;

			_mid = _lower + (((_upper-_lower) >> 1) & ~1);
			if ( (*p) < _mid[0] )
				_upper = _mid - 2;
			else if ( (*p) > _mid[1] )
				_lower = _mid + 2;
			else {
				_trans += (unsigned int)((_mid - _keys)>>1);
				goto _match;
			}
		}
		_trans += _klen;
	}

_match:
	_trans = _sitl2_command_line_indicies[_trans];
_eof_trans:
	cs = _sitl2_command_line_trans_targs[_trans];

	if ( _sitl2_command_line_trans_actions[_trans] == 0 )
		goto _again;

	switch ( _sitl2_command_line_trans_actions[_trans] ) {
	case 3:
/* #line 244 "sitl2_command_line.rl" */
	{ 
#ifdef TEST_RAGEL_PARSER
            printf(" error at %d \"%s\"\n", (int)(p-start), p); 
#endif
            return WMQE_INVALID_CLI_COMMAND;
        }
	break;
	case 10:
/* #line 290 "sitl2_command_line.rl" */
	{ {stack[top++] = cs; cs = 36;goto _again;} }
	break;
	case 4:
/* #line 290 "sitl2_command_line.rl" */
	{ {stack[top++] = cs; cs = 36;goto _again;} }
	break;
	case 26:
/* #line 135 "sitl2_command_line.rl" */
	{te = p;p--;{
#ifdef TEST_RAGEL_PARSER
            printf(" watch_param_time" );
#else
            ctx->watch_time = 1;
            ctx->watch_any = 1;
#endif
            {cs = stack[--top]; goto _again;}
        }}
	break;
	case 20:
/* #line 145 "sitl2_command_line.rl" */
	{te = p;p--;{
#ifdef TEST_RAGEL_PARSER
            printf(" watch_param_motors" );
#else
            ctx->watch_motors = 1;
            ctx->watch_any = 1;
#endif
            {cs = stack[--top]; goto _again;}
        }}
	break;
	case 22:
/* #line 155 "sitl2_command_line.rl" */
	{te = p;p--;{
#ifdef TEST_RAGEL_PARSER
            printf(" watch_param_task_rate" );
#else
            ctx->watch_task_rate = 1;
            ctx->watch_any = 1;
#endif
            {cs = stack[--top]; goto _again;}
        }}
	break;
	case 24:
/* #line 165 "sitl2_command_line.rl" */
	{te = p;p--;{
#ifdef TEST_RAGEL_PARSER
            printf(" watch_param_scheduler" );
#else
            ctx->watch_scheduler = 1;
            ctx->watch_any = 1;
#endif
            {cs = stack[--top]; goto _again;}
        }}
	break;
	case 16:
/* #line 175 "sitl2_command_line.rl" */
	{te = p;p--;{
#ifdef TEST_RAGEL_PARSER
            printf(" watch_param_arm_flags" );
#else
            ctx->watch_arm_flags = 1;
            ctx->watch_any = 1;
#endif
            {cs = stack[--top]; goto _again;}
        }}
	break;
	case 17:
/* #line 185 "sitl2_command_line.rl" */
	{te = p;p--;{
#ifdef TEST_RAGEL_PARSER
            printf(" watch_param ERROR\n" ); 
#else
            //rc = WMQE_INVALID_CLI_COMMAND;
#endif
            return WMQE_PARSE_ERROR;
            {cs = stack[--top]; goto _again;}
        }}
	break;
	case 15:
/* #line 1 "NONE" */
	{	switch( act ) {
	case 1:
	{{p = ((te))-1;}
#ifdef TEST_RAGEL_PARSER
            printf(" watch_param_all" );
#else
            ctx->watch_all = 1;
            ctx->watch_any = 1;
#endif
            {cs = stack[--top]; goto _again;}
        }
	break;
	case 2:
	{{p = ((te))-1;}
#ifdef TEST_RAGEL_PARSER
            printf(" watch_param_all" );
#else
            ctx->watch_all = 1;
            ctx->watch_any = 1;
#endif
            {cs = stack[--top]; goto _again;}
        }
	break;
	case 3:
	{{p = ((te))-1;}
#ifdef TEST_RAGEL_PARSER
            printf(" watch_param_time" );
#else
            ctx->watch_time = 1;
            ctx->watch_any = 1;
#endif
            {cs = stack[--top]; goto _again;}
        }
	break;
	case 5:
	{{p = ((te))-1;}
#ifdef TEST_RAGEL_PARSER
            printf(" watch_param_motors" );
#else
            ctx->watch_motors = 1;
            ctx->watch_any = 1;
#endif
            {cs = stack[--top]; goto _again;}
        }
	break;
	case 7:
	{{p = ((te))-1;}
#ifdef TEST_RAGEL_PARSER
            printf(" watch_param_task_rate" );
#else
            ctx->watch_task_rate = 1;
            ctx->watch_any = 1;
#endif
            {cs = stack[--top]; goto _again;}
        }
	break;
	case 9:
	{{p = ((te))-1;}
#ifdef TEST_RAGEL_PARSER
            printf(" watch_param_scheduler" );
#else
            ctx->watch_scheduler = 1;
            ctx->watch_any = 1;
#endif
            {cs = stack[--top]; goto _again;}
        }
	break;
	case 11:
	{{p = ((te))-1;}
#ifdef TEST_RAGEL_PARSER
            printf(" watch_param_arm_flags" );
#else
            ctx->watch_arm_flags = 1;
            ctx->watch_any = 1;
#endif
            {cs = stack[--top]; goto _again;}
        }
	break;
	case 13:
	{{p = ((te))-1;}
#ifdef TEST_RAGEL_PARSER
            printf(" watch_param ERROR\n" ); 
#else
            //rc = WMQE_INVALID_CLI_COMMAND;
#endif
            return WMQE_PARSE_ERROR;
            {cs = stack[--top]; goto _again;}
        }
	break;
	}
	}
	break;
	case 18:
/* #line 1 "NONE" */
	{te = p+1;}
/* #line 125 "sitl2_command_line.rl" */
	{act = 1;}
	break;
	case 14:
/* #line 1 "NONE" */
	{te = p+1;}
/* #line 125 "sitl2_command_line.rl" */
	{act = 2;}
	break;
	case 27:
/* #line 1 "NONE" */
	{te = p+1;}
/* #line 135 "sitl2_command_line.rl" */
	{act = 3;}
	break;
	case 21:
/* #line 1 "NONE" */
	{te = p+1;}
/* #line 145 "sitl2_command_line.rl" */
	{act = 5;}
	break;
	case 23:
/* #line 1 "NONE" */
	{te = p+1;}
/* #line 155 "sitl2_command_line.rl" */
	{act = 7;}
	break;
	case 25:
/* #line 1 "NONE" */
	{te = p+1;}
/* #line 165 "sitl2_command_line.rl" */
	{act = 9;}
	break;
	case 19:
/* #line 1 "NONE" */
	{te = p+1;}
/* #line 175 "sitl2_command_line.rl" */
	{act = 11;}
	break;
	case 13:
/* #line 1 "NONE" */
	{te = p+1;}
/* #line 185 "sitl2_command_line.rl" */
	{act = 13;}
	break;
/* #line 563 "sitl2_command_line.inc" */
	}

_again:
	switch ( _sitl2_command_line_to_state_actions[cs] ) {
	case 1:
/* #line 1 "NONE" */
	{ts = 0;}
	break;
/* #line 572 "sitl2_command_line.inc" */
	}

	if ( cs == 0 )
		goto _out;
	if ( ++p != pe )
		goto _resume;
	_test_eof: {}
	if ( p == eof )
	{
	if ( _sitl2_command_line_eof_trans[cs] > 0 ) {
		_trans = _sitl2_command_line_eof_trans[cs] - 1;
		goto _eof_trans;
	}
	switch ( _sitl2_command_line_eof_actions[cs] ) {
	case 8:
/* #line 106 "sitl2_command_line.rl" */
	{ 
#ifdef TEST_RAGEL_PARSER
            printf(" STATUS\n");
#else
            rc = sitl2_cli_STATUS(ctx);
            WMQ_CHECK_ERROR_AND_RETURN_RESULT(rc, "sitl2_cli_STATUS");
#endif
        }
/* #line 251 "sitl2_command_line.rl" */
	{ 
#ifdef TEST_RAGEL_PARSER
            printf(" eof at %d\n", (int)(p-start));
#endif
        }
	break;
	case 9:
/* #line 196 "sitl2_command_line.rl" */
	{ 
#ifdef TEST_RAGEL_PARSER
            printf(" WATCH\n");
#else
            rc = sitl2_cli_WATCH(ctx);
            WMQ_CHECK_ERROR_AND_RETURN_RESULT(rc, "sitl2_cli_WATCH");
#endif
        }
/* #line 251 "sitl2_command_line.rl" */
	{ 
#ifdef TEST_RAGEL_PARSER
            printf(" eof at %d\n", (int)(p-start));
#endif
        }
	break;
	case 11:
/* #line 205 "sitl2_command_line.rl" */
	{ 
#ifdef TEST_RAGEL_PARSER
            printf(" WATCH_STOP\n");
#else
            rc = sitl2_cli_WATCH_STOP(ctx);
            WMQ_CHECK_ERROR_AND_RETURN_RESULT(rc, "sitl2_cli_WATCH_STOP");
#endif
        }
/* #line 251 "sitl2_command_line.rl" */
	{ 
#ifdef TEST_RAGEL_PARSER
            printf(" eof at %d\n", (int)(p-start));
#endif
        }
	break;
	case 7:
/* #line 215 "sitl2_command_line.rl" */
	{ 
#ifdef TEST_RAGEL_PARSER
            printf(" HELP\n");
#else
            rc = sitl2_cli_HELP(ctx);
            WMQ_CHECK_ERROR_AND_RETURN_RESULT(rc, "sitl2_cli_HELP");
#endif
        }
/* #line 251 "sitl2_command_line.rl" */
	{ 
#ifdef TEST_RAGEL_PARSER
            printf(" eof at %d\n", (int)(p-start));
#endif
        }
	break;
	case 6:
/* #line 225 "sitl2_command_line.rl" */
	{ 
#ifdef TEST_RAGEL_PARSER
            printf(" EXIT\n");
            *is_exit = 1;
#else
            rc = sitl2_cli_EXIT(ctx);
            WMQ_CHECK_ERROR_AND_RETURN_RESULT(rc, "sitl2_cli_EXIT");
#endif
        }
/* #line 251 "sitl2_command_line.rl" */
	{ 
#ifdef TEST_RAGEL_PARSER
            printf(" eof at %d\n", (int)(p-start));
#endif
        }
	break;
	case 5:
/* #line 236 "sitl2_command_line.rl" */
	{ 
#ifdef TEST_RAGEL_PARSER
            printf(" COMMENT\n");
#else
#endif
        }
/* #line 251 "sitl2_command_line.rl" */
	{ 
#ifdef TEST_RAGEL_PARSER
            printf(" eof at %d\n", (int)(p-start));
#endif
        }
	break;
	case 2:
/* #line 244 "sitl2_command_line.rl" */
	{ 
#ifdef TEST_RAGEL_PARSER
            printf(" error at %d \"%s\"\n", (int)(p-start), p); 
#endif
            return WMQE_INVALID_CLI_COMMAND;
        }
/* #line 251 "sitl2_command_line.rl" */
	{ 
#ifdef TEST_RAGEL_PARSER
            printf(" eof at %d\n", (int)(p-start));
#endif
        }
	break;
/* #line 703 "sitl2_command_line.inc" */
	}
	}

	_out: {}
	}

/* #line 327 "sitl2_command_line.rl" */


#ifdef TEST_RAGEL_PARSER
    printf(" cs: %d\n", cs);
#endif

    if(cs==sitl2_command_line_error){
       printf(" error\n");
       return WMQE_PARSE_ERROR;
    }

    if(cs>=sitl2_command_line_first_final){
#ifdef TEST_RAGEL_PARSER
       printf(" ok\n");
#endif

       return 0;
    }
    printf("unknown command\n");


    return WMQE_PARSE_ERROR;
}


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