
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
	15, 17, 18, 19, 20, 22, 23, 24, 
	25, 26, 27, 28, 32, 34, 35, 36, 
	37, 38, 39, 40, 41, 41, 45, 48, 
	52, 56, 59, 63, 67, 70, 73, 77, 
	80, 84, 90, 94, 97, 102, 105, 114, 
	117, 122, 126, 130, 134, 138, 142, 146, 
	150, 154, 158, 162, 166, 170, 174, 178, 
	182, 186, 190, 194, 198, 202
};

static const char _sitl2_command_line_trans_keys[] = {
	47, 101, 104, 113, 115, 119, 47, 105, 
	116, 32, 41, 9, 13, 108, 112, 105, 
	116, 109, 115, 116, 97, 111, 114, 116, 
	112, 116, 117, 115, 32, 41, 9, 13, 
	41, 44, 116, 99, 104, 115, 116, 111, 
	112, 32, 120, 9, 13, 32, 9, 13, 
	32, 40, 9, 13, 32, 101, 9, 13, 
	32, 9, 13, 32, 117, 9, 13, 32, 
	95, 9, 13, 32, 9, 13, 32, 9, 
	13, 32, 97, 9, 13, 32, 9, 13, 
	32, 40, 9, 13, 32, 40, 97, 115, 
	9, 13, 32, 40, 9, 13, 32, 9, 
	13, 32, 40, 95, 9, 13, 32, 9, 
	13, 41, 42, 44, 61, 97, 109, 114, 
	115, 116, 41, 44, 61, 41, 44, 61, 
	108, 114, 41, 44, 61, 108, 41, 44, 
	61, 109, 41, 44, 61, 111, 41, 44, 
	61, 116, 41, 44, 61, 111, 41, 44, 
	61, 114, 41, 44, 61, 115, 41, 44, 
	61, 97, 41, 44, 61, 116, 41, 44, 
	61, 101, 41, 44, 61, 99, 41, 44, 
	61, 104, 41, 44, 61, 101, 41, 44, 
	61, 100, 41, 44, 61, 117, 41, 44, 
	61, 108, 41, 44, 61, 101, 41, 44, 
	61, 114, 41, 44, 61, 105, 41, 44, 
	61, 109, 41, 44, 61, 101, 0
};

static const char _sitl2_command_line_single_lengths[] = {
	0, 6, 1, 1, 1, 2, 1, 1, 
	2, 1, 1, 1, 2, 1, 1, 1, 
	1, 1, 1, 2, 2, 1, 1, 1, 
	1, 1, 1, 1, 0, 2, 1, 2, 
	2, 1, 2, 2, 1, 1, 2, 1, 
	2, 4, 2, 1, 3, 1, 9, 3, 
	5, 4, 4, 4, 4, 4, 4, 4, 
	4, 4, 4, 4, 4, 4, 4, 4, 
	4, 4, 4, 4, 4, 4
};

static const char _sitl2_command_line_range_lengths[] = {
	0, 0, 0, 0, 0, 1, 0, 0, 
	0, 0, 0, 0, 0, 0, 0, 0, 
	0, 0, 0, 1, 0, 0, 0, 0, 
	0, 0, 0, 0, 0, 1, 1, 1, 
	1, 1, 1, 1, 1, 1, 1, 1, 
	1, 1, 1, 1, 1, 1, 0, 0, 
	0, 0, 0, 0, 0, 0, 0, 0, 
	0, 0, 0, 0, 0, 0, 0, 0, 
	0, 0, 0, 0, 0, 0
};

static const short _sitl2_command_line_index_offsets[] = {
	0, 0, 7, 9, 11, 13, 17, 19, 
	21, 24, 26, 28, 30, 33, 35, 37, 
	39, 41, 43, 45, 49, 52, 54, 56, 
	58, 60, 62, 64, 66, 67, 71, 74, 
	78, 82, 85, 89, 93, 96, 99, 103, 
	106, 110, 116, 120, 123, 128, 131, 141, 
	145, 151, 156, 161, 166, 171, 176, 181, 
	186, 191, 196, 201, 206, 211, 216, 221, 
	226, 231, 236, 241, 246, 251
};

static const char _sitl2_command_line_indicies[] = {
	1, 2, 3, 4, 5, 6, 0, 7, 
	0, 8, 0, 9, 0, 10, 11, 10, 
	0, 12, 0, 13, 0, 14, 15, 0, 
	16, 0, 17, 0, 18, 0, 19, 20, 
	0, 21, 0, 22, 0, 23, 0, 24, 
	0, 25, 0, 26, 0, 27, 28, 27, 
	0, 29, 30, 0, 31, 0, 32, 0, 
	33, 0, 34, 0, 35, 0, 36, 0, 
	37, 0, 7, 11, 38, 11, 0, 11, 
	11, 0, 9, 10, 9, 0, 13, 39, 
	13, 0, 13, 13, 0, 11, 38, 11, 
	0, 22, 40, 22, 0, 22, 22, 0, 
	23, 23, 0, 28, 41, 28, 0, 28, 
	28, 0, 26, 27, 26, 0, 42, 43, 
	44, 37, 42, 0, 42, 43, 42, 0, 
	29, 29, 0, 42, 43, 45, 42, 0, 
	37, 37, 0, 48, 47, 48, 48, 49, 
	50, 51, 52, 53, 46, 54, 54, 54, 
	46, 55, 55, 55, 56, 57, 46, 58, 
	58, 58, 59, 46, 58, 58, 58, 60, 
	46, 61, 61, 61, 62, 46, 58, 58, 
	58, 63, 46, 58, 58, 58, 64, 46, 
	58, 58, 58, 65, 46, 58, 58, 58, 
	66, 46, 67, 67, 67, 68, 46, 58, 
	58, 58, 69, 46, 58, 58, 58, 70, 
	46, 71, 71, 71, 72, 46, 58, 58, 
	58, 73, 46, 58, 58, 58, 74, 46, 
	58, 58, 58, 75, 46, 58, 58, 58, 
	76, 46, 58, 58, 58, 77, 46, 58, 
	58, 58, 78, 46, 58, 58, 58, 79, 
	46, 80, 80, 80, 81, 46, 58, 58, 
	58, 82, 46, 58, 58, 58, 83, 46, 
	0
};

static const char _sitl2_command_line_trans_targs[] = {
	0, 2, 29, 32, 34, 8, 41, 28, 
	4, 31, 5, 30, 7, 33, 9, 38, 
	35, 11, 12, 13, 15, 14, 36, 37, 
	17, 18, 40, 19, 39, 43, 20, 22, 
	23, 44, 25, 26, 27, 45, 3, 6, 
	10, 16, 42, 20, 21, 24, 47, 47, 
	0, 48, 51, 56, 59, 67, 46, 46, 
	49, 50, 46, 47, 47, 46, 52, 53, 
	54, 55, 47, 46, 57, 58, 47, 46, 
	60, 61, 62, 63, 64, 65, 66, 47, 
	46, 68, 69, 47
};

static const char _sitl2_command_line_trans_actions[] = {
	3, 0, 0, 0, 0, 0, 0, 0, 
	0, 0, 0, 0, 0, 0, 0, 0, 
	0, 0, 0, 0, 0, 0, 0, 0, 
	0, 0, 0, 0, 0, 0, 4, 0, 
	0, 0, 0, 0, 0, 0, 0, 0, 
	0, 0, 0, 12, 0, 0, 15, 16, 
	0, 0, 0, 0, 0, 0, 17, 18, 
	0, 0, 19, 20, 21, 22, 0, 0, 
	0, 0, 23, 24, 0, 0, 25, 26, 
	0, 0, 0, 0, 0, 0, 0, 27, 
	28, 0, 0, 29
};

static const char _sitl2_command_line_to_state_actions[] = {
	0, 1, 0, 0, 0, 0, 0, 0, 
	0, 0, 0, 0, 0, 0, 0, 0, 
	0, 0, 0, 0, 1, 0, 0, 0, 
	0, 0, 0, 0, 0, 0, 0, 0, 
	0, 0, 0, 0, 0, 0, 0, 0, 
	0, 0, 0, 0, 0, 0, 1, 0, 
	0, 0, 0, 0, 0, 0, 0, 0, 
	0, 0, 0, 0, 0, 0, 0, 0, 
	0, 0, 0, 0, 0, 0
};

static const char _sitl2_command_line_from_state_actions[] = {
	0, 0, 0, 0, 0, 0, 0, 0, 
	0, 0, 0, 0, 0, 0, 0, 0, 
	0, 0, 0, 0, 0, 0, 0, 0, 
	0, 0, 0, 0, 0, 0, 0, 0, 
	0, 0, 0, 0, 0, 0, 0, 0, 
	0, 0, 0, 0, 0, 0, 14, 0, 
	0, 0, 0, 0, 0, 0, 0, 0, 
	0, 0, 0, 0, 0, 0, 0, 0, 
	0, 0, 0, 0, 0, 0
};

static const char _sitl2_command_line_eof_actions[] = {
	0, 2, 2, 2, 2, 2, 2, 2, 
	2, 2, 2, 2, 2, 2, 2, 2, 
	2, 2, 2, 2, 2, 2, 2, 2, 
	2, 2, 2, 2, 5, 6, 6, 6, 
	7, 7, 6, 8, 8, 9, 10, 10, 
	10, 11, 11, 11, 11, 13, 0, 0, 
	0, 0, 0, 0, 0, 0, 0, 0, 
	0, 0, 0, 0, 0, 0, 0, 0, 
	0, 0, 0, 0, 0, 0
};

static const short _sitl2_command_line_eof_trans[] = {
	0, 0, 0, 0, 0, 0, 0, 0, 
	0, 0, 0, 0, 0, 0, 0, 0, 
	0, 0, 0, 0, 0, 0, 0, 0, 
	0, 0, 0, 0, 0, 0, 0, 0, 
	0, 0, 0, 0, 0, 0, 0, 0, 
	0, 0, 0, 0, 0, 0, 0, 55, 
	56, 59, 59, 62, 59, 59, 59, 59, 
	68, 59, 59, 72, 59, 59, 59, 59, 
	59, 59, 59, 81, 59, 59
};

static const int sitl2_command_line_start = 1;
static const int sitl2_command_line_first_final = 28;
static const int sitl2_command_line_error = 0;

static const int sitl2_command_line_en_watch_param_scanner = 46;
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

    
/* #line 263 "sitl2_command_line.inc" */
	{
	cs = sitl2_command_line_start;
	top = 0;
	ts = 0;
	te = 0;
	act = 0;
	}

/* #line 272 "sitl2_command_line.inc" */
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
	case 14:
/* #line 1 "NONE" */
	{ts = p;}
	break;
/* #line 288 "sitl2_command_line.inc" */
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
/* #line 261 "sitl2_command_line.rl" */
	{ 
#ifdef TEST_RAGEL_PARSER
            printf(" error at %d \"%s\"\n", (int)(p-start), p); 
#endif
            return WMQE_INVALID_CLI_COMMAND;
        }
	break;
	case 12:
/* #line 307 "sitl2_command_line.rl" */
	{ {stack[top++] = cs; cs = 46;goto _again;} }
	break;
	case 4:
/* #line 307 "sitl2_command_line.rl" */
	{ {stack[top++] = cs; cs = 46;goto _again;} }
	break;
	case 28:
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
	case 22:
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
	case 24:
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
	case 26:
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
	case 18:
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
	case 19:
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
	case 17:
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
	case 20:
/* #line 1 "NONE" */
	{te = p+1;}
/* #line 125 "sitl2_command_line.rl" */
	{act = 1;}
	break;
	case 16:
/* #line 1 "NONE" */
	{te = p+1;}
/* #line 125 "sitl2_command_line.rl" */
	{act = 2;}
	break;
	case 29:
/* #line 1 "NONE" */
	{te = p+1;}
/* #line 135 "sitl2_command_line.rl" */
	{act = 3;}
	break;
	case 23:
/* #line 1 "NONE" */
	{te = p+1;}
/* #line 145 "sitl2_command_line.rl" */
	{act = 5;}
	break;
	case 25:
/* #line 1 "NONE" */
	{te = p+1;}
/* #line 155 "sitl2_command_line.rl" */
	{act = 7;}
	break;
	case 27:
/* #line 1 "NONE" */
	{te = p+1;}
/* #line 165 "sitl2_command_line.rl" */
	{act = 9;}
	break;
	case 21:
/* #line 1 "NONE" */
	{te = p+1;}
/* #line 175 "sitl2_command_line.rl" */
	{act = 11;}
	break;
	case 15:
/* #line 1 "NONE" */
	{te = p+1;}
/* #line 185 "sitl2_command_line.rl" */
	{act = 13;}
	break;
/* #line 579 "sitl2_command_line.inc" */
	}

_again:
	switch ( _sitl2_command_line_to_state_actions[cs] ) {
	case 1:
/* #line 1 "NONE" */
	{ts = 0;}
	break;
/* #line 588 "sitl2_command_line.inc" */
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
	case 10:
/* #line 106 "sitl2_command_line.rl" */
	{ 
#ifdef TEST_RAGEL_PARSER
            printf(" STATUS\n");
#else
            rc = sitl2_cli_STATUS(ctx);
            WMQ_CHECK_ERROR_AND_RETURN_RESULT(rc, "sitl2_cli_STATUS");
#endif
        }
/* #line 268 "sitl2_command_line.rl" */
	{ 
#ifdef TEST_RAGEL_PARSER
            printf(" eof at %d\n", (int)(p-start));
#endif
        }
	break;
	case 11:
/* #line 196 "sitl2_command_line.rl" */
	{ 
#ifdef TEST_RAGEL_PARSER
            printf(" WATCH\n");
#else
            rc = sitl2_cli_WATCH(ctx);
            WMQ_CHECK_ERROR_AND_RETURN_RESULT(rc, "sitl2_cli_WATCH");
#endif
        }
/* #line 268 "sitl2_command_line.rl" */
	{ 
#ifdef TEST_RAGEL_PARSER
            printf(" eof at %d\n", (int)(p-start));
#endif
        }
	break;
	case 13:
/* #line 205 "sitl2_command_line.rl" */
	{ 
#ifdef TEST_RAGEL_PARSER
            printf(" WATCH_STOP\n");
#else
            rc = sitl2_cli_WATCH_STOP(ctx);
            WMQ_CHECK_ERROR_AND_RETURN_RESULT(rc, "sitl2_cli_WATCH_STOP");
#endif
        }
/* #line 268 "sitl2_command_line.rl" */
	{ 
#ifdef TEST_RAGEL_PARSER
            printf(" eof at %d\n", (int)(p-start));
#endif
        }
	break;
	case 8:
/* #line 214 "sitl2_command_line.rl" */
	{
#ifdef TEST_RAGEL_PARSER
            printf(" SIM_START\n");
#else
            rc = sitl2_cli_SIM_START(ctx);
            WMQ_CHECK_ERROR_AND_RETURN_RESULT(rc, "sitl2_cli_SIM_START");
#endif
        }
/* #line 268 "sitl2_command_line.rl" */
	{ 
#ifdef TEST_RAGEL_PARSER
            printf(" eof at %d\n", (int)(p-start));
#endif
        }
	break;
	case 9:
/* #line 223 "sitl2_command_line.rl" */
	{
#ifdef TEST_RAGEL_PARSER
            printf(" SIM_STOP\n");
#else
            rc = sitl2_cli_SIM_STOP(ctx);
            WMQ_CHECK_ERROR_AND_RETURN_RESULT(rc, "sitl2_cli_SIM_STOP");
#endif
        }
/* #line 268 "sitl2_command_line.rl" */
	{ 
#ifdef TEST_RAGEL_PARSER
            printf(" eof at %d\n", (int)(p-start));
#endif
        }
	break;
	case 7:
/* #line 232 "sitl2_command_line.rl" */
	{ 
#ifdef TEST_RAGEL_PARSER
            printf(" HELP\n");
#else
            rc = sitl2_cli_HELP(ctx);
            WMQ_CHECK_ERROR_AND_RETURN_RESULT(rc, "sitl2_cli_HELP");
#endif
        }
/* #line 268 "sitl2_command_line.rl" */
	{ 
#ifdef TEST_RAGEL_PARSER
            printf(" eof at %d\n", (int)(p-start));
#endif
        }
	break;
	case 6:
/* #line 242 "sitl2_command_line.rl" */
	{ 
#ifdef TEST_RAGEL_PARSER
            printf(" EXIT\n");
            *is_exit = 1;
#else
            rc = sitl2_cli_EXIT(ctx);
            WMQ_CHECK_ERROR_AND_RETURN_RESULT(rc, "sitl2_cli_EXIT");
#endif
        }
/* #line 268 "sitl2_command_line.rl" */
	{ 
#ifdef TEST_RAGEL_PARSER
            printf(" eof at %d\n", (int)(p-start));
#endif
        }
	break;
	case 5:
/* #line 253 "sitl2_command_line.rl" */
	{ 
#ifdef TEST_RAGEL_PARSER
            printf(" COMMENT\n");
#else
#endif
        }
/* #line 268 "sitl2_command_line.rl" */
	{ 
#ifdef TEST_RAGEL_PARSER
            printf(" eof at %d\n", (int)(p-start));
#endif
        }
	break;
	case 2:
/* #line 261 "sitl2_command_line.rl" */
	{ 
#ifdef TEST_RAGEL_PARSER
            printf(" error at %d \"%s\"\n", (int)(p-start), p); 
#endif
            return WMQE_INVALID_CLI_COMMAND;
        }
/* #line 268 "sitl2_command_line.rl" */
	{ 
#ifdef TEST_RAGEL_PARSER
            printf(" eof at %d\n", (int)(p-start));
#endif
        }
	break;
/* #line 753 "sitl2_command_line.inc" */
	}
	}

	_out: {}
	}

/* #line 349 "sitl2_command_line.rl" */


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