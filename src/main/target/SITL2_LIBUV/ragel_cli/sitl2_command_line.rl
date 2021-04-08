#ifdef TEST_RAGEL_PARSER

#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#endif

#define MAX_CALL_PARAMS 3

%%{
    machine sitl2_command_line;
    write data;
}%%

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

    %%{

        action on_start { 
#ifdef TEST_RAGEL_PARSER
            printf(" on_start: %d \n", (int)(fpc-data)); 
#else
#endif
            start = fpc;
            quoted = 0;
        }

        action on_start_quoted { 
#ifdef TEST_RAGEL_PARSER
            printf(" on_start_quoted: %d \n", (int)(fpc-data)); 
#else
#endif
            start = fpc;
            quoted = 1;
        }

        action on_end_key { 
#ifdef TEST_RAGEL_PARSER
            printf(" >>on_end_key: %d \"%.*s\"\n", (int)(fpc-data), (int)(fpc-start), start); 
#else
#endif
            key = start;
            key_size = fpc-start;

        }

        action on_end_value { 
#ifdef TEST_RAGEL_PARSER
            printf(" >>on_end_value: %d %d %d \"%.*s\":\"%.*s\" q:%d\n", (int)(fpc-data), key_size,  (int)(fpc-start), key_size, key, (int)(fpc-start), start, quoted); 
#else
            rc = cb(context, key_size, key, (int)(fpc-start), start, quoted);
            if(rc){
               return rc;
            }
            //WMQ_CHECK_ERROR_AND_RETURN_RESULT(rc, "on string cb");
#endif
        }


        action error { 
#ifdef TEST_RAGEL_PARSER
            printf(" error at %d \"%s\"\n", (int)(fpc-start), fpc); 
#endif
            //return -1;
        }

        action on_eof { 
#ifdef TEST_RAGEL_PARSER
            printf(" eof at %d\n", (int)(fpc-start));
#endif
        }


        action status { 
#ifdef TEST_RAGEL_PARSER
            printf(" STATUS\n");
#else
            sitl2_cli_STATUS(ctx);
#endif
        }

        action exit{ 
#ifdef TEST_RAGEL_PARSER
            printf(" EXIT\n");
            *is_exit = 1;
#else
            sitl2_cli_EXIT(ctx);
#endif
        }


        action comment { 
#ifdef TEST_RAGEL_PARSER
            printf(" COMMENT\n");
#else
#endif
        }



        #str_literal = '"' ((any - '"')** >begin_literal %end_literal)  '"' ;
        #param = space* str_literal space* ;

        #param_numeric = space* ( (('-' @signed)? >reset_sign) digit+ >begin_literal %end_numeric) space* ; 

        dquote = '"';
        not_dquote_or_escape = [^"\\];
        escaped_something = /\\./;


        key = space* ((any - space - ',' - '=')+ >on_start %on_end_key) space*;

        value = space* ((any - space - ',' - '=' - dquote)** >on_start %on_end_value) space*;
        quoted_value = space* dquote ( not_dquote_or_escape | escaped_something )** >on_start_quoted %on_end_value dquote space*;

        key_value = key '=' ( value | quoted_value);


        # | ( (any*) %unknown_command )

        main := (("status" space* ('(' space*  ')')? space* ) %status |
                 ("exit" space* ('(' space* ')')?  space* ) %exit |
                 ("q" space* ) %exit |
                 ( ( "//" any*) %comment)
                ) $err(error) $eof(on_eof);
                      
        # Initialize and execute.
        write init;
        write exec;
    }%%

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