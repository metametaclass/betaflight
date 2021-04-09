
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
	0, 0, 7, 8, 9, 10, 14, 15, 
	16, 17, 19, 20, 21, 22, 24, 25, 
	26, 27, 28, 29, 30, 34, 36, 37, 
	38, 39, 40, 41, 42, 43, 43, 47, 
	50, 54, 58, 61, 65, 68, 72, 75, 
	78, 82, 85, 89, 95, 99, 102, 107, 
	110, 119, 122, 127, 131, 135, 139, 143, 
	147, 151, 155, 159, 163, 167, 171, 175, 
	179, 183, 187, 191, 195, 199, 203, 207
};

static const char _sitl2_command_line_trans_keys[] = {
	47, 101, 104, 113, 114, 115, 119, 47, 
	105, 116, 32, 41, 9, 13, 108, 112, 
	120, 105, 116, 109, 115, 116, 97, 111, 
	114, 116, 112, 116, 117, 115, 32, 41, 
	9, 13, 41, 44, 116, 99, 104, 115, 
	116, 111, 112, 32, 120, 9, 13, 32, 
	9, 13, 32, 40, 9, 13, 32, 101, 
	9, 13, 32, 9, 13, 32, 117, 9, 
	13, 32, 9, 13, 32, 95, 9, 13, 
	32, 9, 13, 32, 9, 13, 32, 97, 
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
	0, 7, 1, 1, 1, 2, 1, 1, 
	1, 2, 1, 1, 1, 2, 1, 1, 
	1, 1, 1, 1, 2, 2, 1, 1, 
	1, 1, 1, 1, 1, 0, 2, 1, 
	2, 2, 1, 2, 1, 2, 1, 1, 
	2, 1, 2, 4, 2, 1, 3, 1, 
	9, 3, 5, 4, 4, 4, 4, 4, 
	4, 4, 4, 4, 4, 4, 4, 4, 
	4, 4, 4, 4, 4, 4, 4, 4
};

static const char _sitl2_command_line_range_lengths[] = {
	0, 0, 0, 0, 0, 1, 0, 0, 
	0, 0, 0, 0, 0, 0, 0, 0, 
	0, 0, 0, 0, 1, 0, 0, 0, 
	0, 0, 0, 0, 0, 0, 1, 1, 
	1, 1, 1, 1, 1, 1, 1, 1, 
	1, 1, 1, 1, 1, 1, 1, 1, 
	0, 0, 0, 0, 0, 0, 0, 0, 
	0, 0, 0, 0, 0, 0, 0, 0, 
	0, 0, 0, 0, 0, 0, 0, 0
};

static const short _sitl2_command_line_index_offsets[] = {
	0, 0, 8, 10, 12, 14, 18, 20, 
	22, 24, 27, 29, 31, 33, 36, 38, 
	40, 42, 44, 46, 48, 52, 55, 57, 
	59, 61, 63, 65, 67, 69, 70, 74, 
	77, 81, 85, 88, 92, 95, 99, 102, 
	105, 109, 112, 116, 122, 126, 129, 134, 
	137, 147, 151, 157, 162, 167, 172, 177, 
	182, 187, 192, 197, 202, 207, 212, 217, 
	222, 227, 232, 237, 242, 247, 252, 257
};

static const char _sitl2_command_line_indicies[] = {
	1, 2, 3, 4, 5, 6, 7, 0, 
	8, 0, 9, 0, 10, 0, 11, 12, 
	11, 0, 13, 0, 14, 0, 15, 0, 
	16, 17, 0, 18, 0, 19, 0, 20, 
	0, 21, 22, 0, 23, 0, 24, 0, 
	25, 0, 26, 0, 27, 0, 28, 0, 
	29, 30, 29, 0, 31, 32, 0, 33, 
	0, 34, 0, 35, 0, 36, 0, 37, 
	0, 38, 0, 39, 0, 8, 12, 40, 
	12, 0, 12, 12, 0, 10, 11, 10, 
	0, 14, 41, 14, 0, 14, 14, 0, 
	12, 40, 12, 0, 15, 15, 0, 24, 
	42, 24, 0, 24, 24, 0, 25, 25, 
	0, 30, 43, 30, 0, 30, 30, 0, 
	28, 29, 28, 0, 44, 45, 46, 39, 
	44, 0, 44, 45, 44, 0, 31, 31, 
	0, 44, 45, 47, 44, 0, 39, 39, 
	0, 50, 49, 50, 50, 51, 52, 53, 
	54, 55, 48, 56, 56, 56, 48, 57, 
	57, 57, 58, 59, 48, 60, 60, 60, 
	61, 48, 60, 60, 60, 62, 48, 63, 
	63, 63, 64, 48, 60, 60, 60, 65, 
	48, 60, 60, 60, 66, 48, 60, 60, 
	60, 67, 48, 60, 60, 60, 68, 48, 
	69, 69, 69, 70, 48, 60, 60, 60, 
	71, 48, 60, 60, 60, 72, 48, 73, 
	73, 73, 74, 48, 60, 60, 60, 75, 
	48, 60, 60, 60, 76, 48, 60, 60, 
	60, 77, 48, 60, 60, 60, 78, 48, 
	60, 60, 60, 79, 48, 60, 60, 60, 
	80, 48, 60, 60, 60, 81, 48, 82, 
	82, 82, 83, 48, 60, 60, 60, 84, 
	48, 60, 60, 60, 85, 48, 0
};

static const char _sitl2_command_line_trans_targs[] = {
	0, 2, 30, 33, 35, 8, 9, 43, 
	29, 4, 32, 5, 31, 7, 34, 36, 
	10, 40, 37, 12, 13, 14, 16, 15, 
	38, 39, 18, 19, 42, 20, 41, 45, 
	21, 23, 24, 46, 26, 27, 28, 47, 
	3, 6, 11, 17, 44, 21, 22, 25, 
	49, 49, 0, 50, 53, 58, 61, 69, 
	48, 48, 51, 52, 48, 49, 49, 48, 
	54, 55, 56, 57, 49, 48, 59, 60, 
	49, 48, 62, 63, 64, 65, 66, 67, 
	68, 49, 48, 70, 71, 49
};

static const char _sitl2_command_line_trans_actions[] = {
	3, 0, 0, 0, 0, 0, 0, 0, 
	0, 0, 0, 0, 0, 0, 0, 0, 
	0, 0, 0, 0, 0, 0, 0, 0, 
	0, 0, 0, 0, 0, 0, 0, 0, 
	4, 0, 0, 0, 0, 0, 0, 0, 
	0, 0, 0, 0, 0, 13, 0, 0, 
	16, 17, 0, 0, 0, 0, 0, 0, 
	18, 19, 0, 0, 20, 21, 22, 23, 
	0, 0, 0, 0, 24, 25, 0, 0, 
	26, 27, 0, 0, 0, 0, 0, 0, 
	0, 28, 29, 0, 0, 30
};

static const char _sitl2_command_line_to_state_actions[] = {
	0, 1, 0, 0, 0, 0, 0, 0, 
	0, 0, 0, 0, 0, 0, 0, 0, 
	0, 0, 0, 0, 0, 1, 0, 0, 
	0, 0, 0, 0, 0, 0, 0, 0, 
	0, 0, 0, 0, 0, 0, 0, 0, 
	0, 0, 0, 0, 0, 0, 0, 0, 
	1, 0, 0, 0, 0, 0, 0, 0, 
	0, 0, 0, 0, 0, 0, 0, 0, 
	0, 0, 0, 0, 0, 0, 0, 0
};

static const char _sitl2_command_line_from_state_actions[] = {
	0, 0, 0, 0, 0, 0, 0, 0, 
	0, 0, 0, 0, 0, 0, 0, 0, 
	0, 0, 0, 0, 0, 0, 0, 0, 
	0, 0, 0, 0, 0, 0, 0, 0, 
	0, 0, 0, 0, 0, 0, 0, 0, 
	0, 0, 0, 0, 0, 0, 0, 0, 
	15, 0, 0, 0, 0, 0, 0, 0, 
	0, 0, 0, 0, 0, 0, 0, 0, 
	0, 0, 0, 0, 0, 0, 0, 0
};

static const char _sitl2_command_line_eof_actions[] = {
	0, 2, 2, 2, 2, 2, 2, 2, 
	2, 2, 2, 2, 2, 2, 2, 2, 
	2, 2, 2, 2, 2, 2, 2, 2, 
	2, 2, 2, 2, 2, 5, 6, 6, 
	6, 7, 7, 6, 8, 9, 9, 10, 
	11, 11, 11, 12, 12, 12, 12, 14, 
	0, 0, 0, 0, 0, 0, 0, 0, 
	0, 0, 0, 0, 0, 0, 0, 0, 
	0, 0, 0, 0, 0, 0, 0, 0
};

static const short _sitl2_command_line_eof_trans[] = {
	0, 0, 0, 0, 0, 0, 0, 0, 
	0, 0, 0, 0, 0, 0, 0, 0, 
	0, 0, 0, 0, 0, 0, 0, 0, 
	0, 0, 0, 0, 0, 0, 0, 0, 
	0, 0, 0, 0, 0, 0, 0, 0, 
	0, 0, 0, 0, 0, 0, 0, 0, 
	0, 57, 58, 61, 61, 64, 61, 61, 
	61, 61, 70, 61, 61, 74, 61, 61, 
	61, 61, 61, 61, 61, 83, 61, 61
};

static const int sitl2_command_line_start = 1;
static const int sitl2_command_line_first_final = 29;
static const int sitl2_command_line_error = 0;

static const int sitl2_command_line_en_watch_param_scanner = 48;
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

    
/* #line 264 "sitl2_command_line.inc" */
	{
	cs = sitl2_command_line_start;
	top = 0;
	ts = 0;
	te = 0;
	act = 0;
	}

/* #line 273 "sitl2_command_line.inc" */
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
	case 15:
/* #line 1 "NONE" */
	{ts = p;}
	break;
/* #line 289 "sitl2_command_line.inc" */
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
/* #line 270 "sitl2_command_line.rl" */
	{ 
#ifdef TEST_RAGEL_PARSER
            printf(" error at %d \"%s\"\n", (int)(p-start), p); 
#endif
            return WMQE_INVALID_CLI_COMMAND;
        }
	break;
	case 13:
/* #line 316 "sitl2_command_line.rl" */
	{ {stack[top++] = cs; cs = 48;goto _again;} }
	break;
	case 4:
/* #line 316 "sitl2_command_line.rl" */
	{ {stack[top++] = cs; cs = 48;goto _again;} }
	break;
	case 29:
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
	case 23:
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
	case 25:
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
	case 27:
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
	case 19:
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
	case 20:
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
	case 18:
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
	case 21:
/* #line 1 "NONE" */
	{te = p+1;}
/* #line 125 "sitl2_command_line.rl" */
	{act = 1;}
	break;
	case 17:
/* #line 1 "NONE" */
	{te = p+1;}
/* #line 125 "sitl2_command_line.rl" */
	{act = 2;}
	break;
	case 30:
/* #line 1 "NONE" */
	{te = p+1;}
/* #line 135 "sitl2_command_line.rl" */
	{act = 3;}
	break;
	case 24:
/* #line 1 "NONE" */
	{te = p+1;}
/* #line 145 "sitl2_command_line.rl" */
	{act = 5;}
	break;
	case 26:
/* #line 1 "NONE" */
	{te = p+1;}
/* #line 155 "sitl2_command_line.rl" */
	{act = 7;}
	break;
	case 28:
/* #line 1 "NONE" */
	{te = p+1;}
/* #line 165 "sitl2_command_line.rl" */
	{act = 9;}
	break;
	case 22:
/* #line 1 "NONE" */
	{te = p+1;}
/* #line 175 "sitl2_command_line.rl" */
	{act = 11;}
	break;
	case 16:
/* #line 1 "NONE" */
	{te = p+1;}
/* #line 185 "sitl2_command_line.rl" */
	{act = 13;}
	break;
/* #line 580 "sitl2_command_line.inc" */
	}

_again:
	switch ( _sitl2_command_line_to_state_actions[cs] ) {
	case 1:
/* #line 1 "NONE" */
	{ts = 0;}
	break;
/* #line 589 "sitl2_command_line.inc" */
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
	case 11:
/* #line 106 "sitl2_command_line.rl" */
	{ 
#ifdef TEST_RAGEL_PARSER
            printf(" STATUS\n");
#else
            rc = sitl2_cli_STATUS(ctx);
            WMQ_CHECK_ERROR_AND_RETURN_RESULT(rc, "sitl2_cli_STATUS");
#endif
        }
/* #line 277 "sitl2_command_line.rl" */
	{ 
#ifdef TEST_RAGEL_PARSER
            printf(" eof at %d\n", (int)(p-start));
#endif
        }
	break;
	case 12:
/* #line 196 "sitl2_command_line.rl" */
	{ 
#ifdef TEST_RAGEL_PARSER
            printf(" WATCH\n");
#else
            rc = sitl2_cli_WATCH(ctx);
            WMQ_CHECK_ERROR_AND_RETURN_RESULT(rc, "sitl2_cli_WATCH");
#endif
        }
/* #line 277 "sitl2_command_line.rl" */
	{ 
#ifdef TEST_RAGEL_PARSER
            printf(" eof at %d\n", (int)(p-start));
#endif
        }
	break;
	case 14:
/* #line 205 "sitl2_command_line.rl" */
	{ 
#ifdef TEST_RAGEL_PARSER
            printf(" WATCH_STOP\n");
#else
            rc = sitl2_cli_WATCH_STOP(ctx);
            WMQ_CHECK_ERROR_AND_RETURN_RESULT(rc, "sitl2_cli_WATCH_STOP");
#endif
        }
/* #line 277 "sitl2_command_line.rl" */
	{ 
#ifdef TEST_RAGEL_PARSER
            printf(" eof at %d\n", (int)(p-start));
#endif
        }
	break;
	case 9:
/* #line 214 "sitl2_command_line.rl" */
	{
#ifdef TEST_RAGEL_PARSER
            printf(" SIM_START\n");
#else
            rc = sitl2_cli_SIM_START(ctx);
            WMQ_CHECK_ERROR_AND_RETURN_RESULT(rc, "sitl2_cli_SIM_START");
#endif
        }
/* #line 277 "sitl2_command_line.rl" */
	{ 
#ifdef TEST_RAGEL_PARSER
            printf(" eof at %d\n", (int)(p-start));
#endif
        }
	break;
	case 10:
/* #line 223 "sitl2_command_line.rl" */
	{
#ifdef TEST_RAGEL_PARSER
            printf(" SIM_STOP\n");
#else
            rc = sitl2_cli_SIM_STOP(ctx);
            WMQ_CHECK_ERROR_AND_RETURN_RESULT(rc, "sitl2_cli_SIM_STOP");
#endif
        }
/* #line 277 "sitl2_command_line.rl" */
	{ 
#ifdef TEST_RAGEL_PARSER
            printf(" eof at %d\n", (int)(p-start));
#endif
        }
	break;
	case 8:
/* #line 232 "sitl2_command_line.rl" */
	{
#ifdef TEST_RAGEL_PARSER
            printf(" RX\n");
#else
            rc = sitl2_cli_RX(ctx);
            WMQ_CHECK_ERROR_AND_RETURN_RESULT(rc, "sitl2_cli_RX");
#endif
        }
/* #line 277 "sitl2_command_line.rl" */
	{ 
#ifdef TEST_RAGEL_PARSER
            printf(" eof at %d\n", (int)(p-start));
#endif
        }
	break;
	case 7:
/* #line 241 "sitl2_command_line.rl" */
	{
#ifdef TEST_RAGEL_PARSER
            printf(" HELP\n");
#else
            rc = sitl2_cli_HELP(ctx);
            WMQ_CHECK_ERROR_AND_RETURN_RESULT(rc, "sitl2_cli_HELP");
#endif
        }
/* #line 277 "sitl2_command_line.rl" */
	{ 
#ifdef TEST_RAGEL_PARSER
            printf(" eof at %d\n", (int)(p-start));
#endif
        }
	break;
	case 6:
/* #line 251 "sitl2_command_line.rl" */
	{ 
#ifdef TEST_RAGEL_PARSER
            printf(" EXIT\n");
            *is_exit = 1;
#else
            rc = sitl2_cli_EXIT(ctx);
            WMQ_CHECK_ERROR_AND_RETURN_RESULT(rc, "sitl2_cli_EXIT");
#endif
        }
/* #line 277 "sitl2_command_line.rl" */
	{ 
#ifdef TEST_RAGEL_PARSER
            printf(" eof at %d\n", (int)(p-start));
#endif
        }
	break;
	case 5:
/* #line 262 "sitl2_command_line.rl" */
	{ 
#ifdef TEST_RAGEL_PARSER
            printf(" COMMENT\n");
#else
#endif
        }
/* #line 277 "sitl2_command_line.rl" */
	{ 
#ifdef TEST_RAGEL_PARSER
            printf(" eof at %d\n", (int)(p-start));
#endif
        }
	break;
	case 2:
/* #line 270 "sitl2_command_line.rl" */
	{ 
#ifdef TEST_RAGEL_PARSER
            printf(" error at %d \"%s\"\n", (int)(p-start), p); 
#endif
            return WMQE_INVALID_CLI_COMMAND;
        }
/* #line 277 "sitl2_command_line.rl" */
	{ 
#ifdef TEST_RAGEL_PARSER
            printf(" eof at %d\n", (int)(p-start));
#endif
        }
	break;
/* #line 771 "sitl2_command_line.inc" */
	}
	}

	_out: {}
	}

/* #line 360 "sitl2_command_line.rl" */


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