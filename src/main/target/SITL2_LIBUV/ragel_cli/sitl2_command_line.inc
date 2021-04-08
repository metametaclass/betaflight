
/* #line 1 "sitl2_command_line.rl" */
#ifdef TEST_RAGEL_PARSER

#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#define WMQE_INVALID_CLI_COMMAND -1
#define WMQE_PARSE_ERROR -1

#endif

#define MAX_CALL_PARAMS 3


/* #line 18 "sitl2_command_line.inc" */
static const char _sitl2_command_line_key_offsets[] = {
	0, 0, 4, 5, 6, 7, 8, 12, 
	13, 14, 15, 16, 20, 20, 24, 27, 
	31, 34
};

static const char _sitl2_command_line_trans_keys[] = {
	47, 101, 113, 115, 47, 120, 105, 116, 
	32, 41, 9, 13, 116, 116, 117, 115, 
	32, 41, 9, 13, 32, 40, 9, 13, 
	32, 9, 13, 32, 97, 9, 13, 32, 
	9, 13, 32, 40, 9, 13, 0
};

static const char _sitl2_command_line_single_lengths[] = {
	0, 4, 1, 1, 1, 1, 2, 1, 
	1, 1, 1, 2, 0, 2, 1, 2, 
	1, 2
};

static const char _sitl2_command_line_range_lengths[] = {
	0, 0, 0, 0, 0, 0, 1, 0, 
	0, 0, 0, 1, 0, 1, 1, 1, 
	1, 1
};

static const char _sitl2_command_line_index_offsets[] = {
	0, 0, 5, 7, 9, 11, 13, 17, 
	19, 21, 23, 25, 29, 30, 34, 37, 
	41, 44
};

static const char _sitl2_command_line_indicies[] = {
	1, 2, 3, 4, 0, 5, 0, 6, 
	0, 7, 0, 8, 0, 9, 3, 9, 
	0, 10, 0, 11, 0, 12, 0, 13, 
	0, 14, 15, 14, 0, 5, 8, 9, 
	8, 0, 3, 3, 0, 15, 16, 15, 
	0, 15, 15, 0, 13, 14, 13, 0, 
	0
};

static const char _sitl2_command_line_trans_targs[] = {
	0, 2, 3, 14, 7, 12, 4, 5, 
	13, 6, 15, 9, 10, 17, 11, 16, 
	8
};

static const char _sitl2_command_line_trans_actions[] = {
	2, 0, 0, 0, 0, 0, 0, 0, 
	0, 0, 0, 0, 0, 0, 0, 0, 
	0
};

static const char _sitl2_command_line_eof_actions[] = {
	0, 1, 1, 1, 1, 1, 1, 1, 
	1, 1, 1, 1, 3, 4, 4, 5, 
	5, 5
};

static const int sitl2_command_line_start = 1;
static const int sitl2_command_line_first_final = 12;
static const int sitl2_command_line_error = 0;

static const int sitl2_command_line_en_main = 1;


/* #line 17 "sitl2_command_line.rl" */


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

    int rc;
    const char *p = data, *pe = data + length;
    const char *eof = pe;
    const char *start = data;

//    int tmp;
//    int sign = 1;
   
    int params[MAX_CALL_PARAMS] = {0};
    int param_num = 0;

    int cs;

    
/* #line 117 "sitl2_command_line.inc" */
	{
	cs = sitl2_command_line_start;
	}

/* #line 122 "sitl2_command_line.inc" */
	{
	int _klen;
	const char *_keys;
	int _trans;

	if ( p == pe )
		goto _test_eof;
	if ( cs == 0 )
		goto _out;
_resume:
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
	cs = _sitl2_command_line_trans_targs[_trans];

	if ( _sitl2_command_line_trans_actions[_trans] == 0 )
		goto _again;

	switch ( _sitl2_command_line_trans_actions[_trans] ) {
	case 2:
/* #line 89 "sitl2_command_line.rl" */
	{ 
#ifdef TEST_RAGEL_PARSER
            printf(" error at %d \"%s\"\n", (int)(p-start), p); 
#endif
            return WMQE_INVALID_CLI_COMMAND;
        }
	break;
/* #line 198 "sitl2_command_line.inc" */
	}

_again:
	if ( cs == 0 )
		goto _out;
	if ( ++p != pe )
		goto _resume;
	_test_eof: {}
	if ( p == eof )
	{
	switch ( _sitl2_command_line_eof_actions[cs] ) {
	case 1:
/* #line 89 "sitl2_command_line.rl" */
	{ 
#ifdef TEST_RAGEL_PARSER
            printf(" error at %d \"%s\"\n", (int)(p-start), p); 
#endif
            return WMQE_INVALID_CLI_COMMAND;
        }
/* #line 96 "sitl2_command_line.rl" */
	{ 
#ifdef TEST_RAGEL_PARSER
            printf(" eof at %d\n", (int)(p-start));
#endif
        }
	break;
	case 5:
/* #line 103 "sitl2_command_line.rl" */
	{ 
#ifdef TEST_RAGEL_PARSER
            printf(" STATUS\n");
#else
            rc = sitl2_cli_STATUS(ctx);
            WMQ_CHECK_ERROR_AND_RETURN_RESULT(rc, "sitl2_cli_STATUS");
#endif
        }
/* #line 96 "sitl2_command_line.rl" */
	{ 
#ifdef TEST_RAGEL_PARSER
            printf(" eof at %d\n", (int)(p-start));
#endif
        }
	break;
	case 4:
/* #line 112 "sitl2_command_line.rl" */
	{ 
#ifdef TEST_RAGEL_PARSER
            printf(" EXIT\n");
            *is_exit = 1;
#else
            rc = sitl2_cli_EXIT(ctx);
            WMQ_CHECK_ERROR_AND_RETURN_RESULT(rc, "sitl2_cli_EXIT");
#endif
        }
/* #line 96 "sitl2_command_line.rl" */
	{ 
#ifdef TEST_RAGEL_PARSER
            printf(" eof at %d\n", (int)(p-start));
#endif
        }
	break;
	case 3:
/* #line 123 "sitl2_command_line.rl" */
	{ 
#ifdef TEST_RAGEL_PARSER
            printf(" COMMENT\n");
#else
#endif
        }
/* #line 96 "sitl2_command_line.rl" */
	{ 
#ifdef TEST_RAGEL_PARSER
            printf(" eof at %d\n", (int)(p-start));
#endif
        }
	break;
/* #line 275 "sitl2_command_line.inc" */
	}
	}

	_out: {}
	}

/* #line 162 "sitl2_command_line.rl" */


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