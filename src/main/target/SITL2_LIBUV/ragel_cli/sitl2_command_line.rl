#ifdef TEST_RAGEL_PARSER

#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#define WMQE_INVALID_CLI_COMMAND -1
#define WMQE_PARSE_ERROR -1

#endif

#define SITL2_CLI_MAX_CALL_PARAMS 3
#define SITL2_CLI_STACK_SIZE 16

%%{
    machine sitl2_command_line;
    write data;
}%%

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



        action status { 
#ifdef TEST_RAGEL_PARSER
            printf(" STATUS\n");
#else
            rc = sitl2_cli_STATUS(ctx);
            WMQ_CHECK_ERROR_AND_RETURN_RESULT(rc, "sitl2_cli_STATUS");
#endif
        }

        action on_end_watch_param { 
#ifdef TEST_RAGEL_PARSER
            printf(" >>on_end_watch_param: %zu \"%.*s\"\n", (size_t)(fpc-data), (int)(fpc-start), start ); 
#else
            //rc = sitl2_cli_WATCH(ctx);
            //WMQ_CHECK_ERROR_AND_RETURN_RESULT(rc, "sitl2_cli_WATCH");
            rc = 0;
#endif
        }

        action watch_param_all {
#ifdef TEST_RAGEL_PARSER
            printf(" watch_param_all" );
#else
            ctx->watch_all = 1;
            ctx->watch_any = 1;
#endif
            fret;
        }

        action watch_param_time {
#ifdef TEST_RAGEL_PARSER
            printf(" watch_param_time" );
#else
            ctx->watch_time = 1;
            ctx->watch_any = 1;
#endif
            fret;
        }

        action watch_param_motors {
#ifdef TEST_RAGEL_PARSER
            printf(" watch_param_motors" );
#else
            ctx->watch_motors = 1;
            ctx->watch_any = 1;
#endif
            fret;
        }

        action watch_param_task_rate {
#ifdef TEST_RAGEL_PARSER
            printf(" watch_param_task_rate" );
#else
            ctx->watch_task_rate = 1;
            ctx->watch_any = 1;
#endif
            fret;
        }

        action watch_param_scheduler {
#ifdef TEST_RAGEL_PARSER
            printf(" watch_param_scheduler" );
#else
            ctx->watch_scheduler = 1;
            ctx->watch_any = 1;
#endif
            fret;
        }

        action watch_param_arm_flags {
#ifdef TEST_RAGEL_PARSER
            printf(" watch_param_arm_flags" );
#else
            ctx->watch_arm_flags = 1;
            ctx->watch_any = 1;
#endif
            fret;
        }

        action watch_param_error {
#ifdef TEST_RAGEL_PARSER
            printf(" watch_param ERROR\n" ); 
#else
            //rc = WMQE_INVALID_CLI_COMMAND;
#endif
            return WMQE_PARSE_ERROR;
            fret;
        }


        action watch { 
#ifdef TEST_RAGEL_PARSER
            printf(" WATCH\n");
#else
            rc = sitl2_cli_WATCH(ctx);
            WMQ_CHECK_ERROR_AND_RETURN_RESULT(rc, "sitl2_cli_WATCH");
#endif
        }

        action watch_stop { 
#ifdef TEST_RAGEL_PARSER
            printf(" WATCH_STOP\n");
#else
            rc = sitl2_cli_WATCH_STOP(ctx);
            WMQ_CHECK_ERROR_AND_RETURN_RESULT(rc, "sitl2_cli_WATCH_STOP");
#endif
        }


        action help{ 
#ifdef TEST_RAGEL_PARSER
            printf(" HELP\n");
#else
            rc = sitl2_cli_HELP(ctx);
            WMQ_CHECK_ERROR_AND_RETURN_RESULT(rc, "sitl2_cli_HELP");
#endif
        }


        action exit{ 
#ifdef TEST_RAGEL_PARSER
            printf(" EXIT\n");
            *is_exit = 1;
#else
            rc = sitl2_cli_EXIT(ctx);
            WMQ_CHECK_ERROR_AND_RETURN_RESULT(rc, "sitl2_cli_EXIT");
#endif
        }


        action comment { 
#ifdef TEST_RAGEL_PARSER
            printf(" COMMENT\n");
#else
#endif
        }


        action error { 
#ifdef TEST_RAGEL_PARSER
            printf(" error at %d \"%s\"\n", (int)(fpc-start), fpc); 
#endif
            return WMQE_INVALID_CLI_COMMAND;
        }

        action on_eof { 
#ifdef TEST_RAGEL_PARSER
            printf(" eof at %d\n", (int)(fpc-start));
#endif
        }



        #str_literal = '"' ((any - '"')** >begin_literal %end_literal)  '"' ;
        #param = space* str_literal space* ;

        #param_numeric = space* ( (('-' @signed)? >reset_sign) digit+ >begin_literal %end_numeric) space* ; 

        dquote = '"';
        not_dquote_or_escape = [^"\\];
        escaped_something = /\\./;

        
        watch_param_scanner := |*
            "all" => watch_param_all;
            "*" => watch_param_all;
            "time" => watch_param_time;
            "t" => watch_param_time;
            "motors" => watch_param_motors;
            "m" => watch_param_motors;
            "rate" => watch_param_task_rate;
            "r" => watch_param_task_rate;
            "scheduler" => watch_param_scheduler;
            "s" => watch_param_scheduler;
            "arm" => watch_param_arm_flags;
            "a" => watch_param_arm_flags;
            #(any - ',' - ')' - '=' )+ => { fret; };
            (any - ',' - ')' - '=' )+ => watch_param_error;
        *|;

        #watch_param = space* ((any - space - ',' - '=' )+ >on_start %on_end_watch_param) space*;
        #watch_param = space* ( ( alpha | '_' ) ( alnum | '_' )* ) >on_start %on_end_watch_param space*;
        #watch_param := {fcall watch_param_scanner};

        watch_params = '(' @{ fcall watch_param_scanner; } (',' @{ fcall watch_param_scanner; } )* ')';


        key = space* ((any - space - ',' - '=')+ >on_start %on_end_key) space*;

        value = space* ((any - space - ',' - '=' - dquote)** >on_start %on_end_value) space*;
        quoted_value = space* dquote ( not_dquote_or_escape | escaped_something )** >on_start_quoted %on_end_value dquote space*;

        key_value = key '=' ( value | quoted_value);


        # | ( (any*) %unknown_command )

        main := (("status" space* ('(' space*  ')')? space* ) %status |
                 ("st" space*) %status |

                 #("watch" space* ('(' watch_param (',' watch_param)* ')' )? space* ) %watch |
                 ("watch" space* watch_params? space* ) %watch |
                 ("w" space* watch_params? space* ) %watch |

                 ("watch_stop" space*) %watch_stop |
                 ("ws" space*) %watch_stop |

                 ("help" space* ) %help |
                 ("h" space* ) %help |

                 ("exit" space* ('(' space* ')')?  space* ) %exit |
                 ("e" space* ) %exit |
                 ("quit" space* ('(' space* ')')?  space* ) %exit |
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