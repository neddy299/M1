/* See COPYING.txt for license details. */

/*
*
*  m1_deauth.h
*
*  M1 LAB_TEST_DEAUTH functions
*
* M1 Project
*
*/

#ifndef M1_LAB_TEST_DEAUTH_H_
#define M1_LAB_TEST_DEAUTH_H_

#include <stdbool.h>
#include "m1_compile_cfg.h"


#define AT_RESP_BUF_SIZE			512    /* Buffer for AT responses */
#define AT_LARGE_RESP_BUF_SIZE		8192   /* Buffer for AT responses - CWLAP can return a lot of data */
#define MULTI_AT_CMD_INTERVAL_MS	50

#define MAX_LIST_ITEMS 		64
#define MAX_SIZE_ADDRESS 	18
#define MAX_SIZE_NAME 		32
#define MAX_SIZE_TITLE 		64

#define LIST_HEADER_HEIGHT  12
#define LIST_ITEM_HEIGHT    9
#define LIST_START_Y        (LIST_HEADER_HEIGHT + 2)
#define LIST_VISIBLE_ITEMS  5

#define INPUT_POLL_DELAY    200

#define TITLE_DEFAULT       "Deauther"
#define TITLE_ERROR         "Deauther Error"
#define TITLE_SYSTEM_CHECK  "Deauth System Check"


void menu_lab_test_deauth_init(void);
void menu_lab_test_deauth_exit(void);

void lab_test_deauth(void);

#endif /* M1_LAB_TEST_DEAUTH_H_ */
