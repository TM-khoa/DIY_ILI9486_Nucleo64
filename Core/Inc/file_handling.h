/*
 * file_handling.h
 *
 *  Created on: 26-Jul-2019
 *      Author: arunr
 */

#ifndef FILE_HANDLING_H_
#define FILE_HANDLING_H_

#include "fatfs.h"
#include "string.h"
#include "stdio.h"
#ifdef __cplusplus
extern "C" {
#endif

/* following are the commands that you can use
 * "ls " : lists all the directories and files in the given path
 * "mkdir " : creates a directory in the given path
 * "mkfil " : creates a file in the given path
 * "read " : reads the content of the given file and send them to uart
 * "write " : write the data to the given file
 * "rm " : removes a file or a directory. Directory can only be removed, if it is empty.
 * "update " : updates the content of the given file
 * "checkfile " : gives the details of the file
 * "checksd " : get free space of the sd card
 */

typedef void (*presult)(char *str_result);
typedef void (*perr)(char *str_error);
typedef UINT (*callback_func)(const BYTE*, UINT);

void fhl_mount_sd(void);
void fhl_unmount_sd(void);

/* get the content inside the buffer, until the buffer is full*/
int fhl_get_content_length(char *buf);

/* counts the length of the command
 * it checks for the space ' ' char. so make sure you give space after the command
 */
int fhl_cmdlength(char *str);

/* copies the path from the buffer to the pathbuffer*/
void fhl_get_path(void);

/* Start node to be scanned (***also used as work area***) */
FRESULT fhl_scan_files(char *pat);

/* write the data to the file
 * @ name : is the path to the file*/
void fhl_write_file(char *name);

/* read data from the file
 * @ name : is the path to the file*/
void fhl_read_file(char *name);

/* creates the file, if it does not exists
 * @ name : is the path to the file*/
void fhl_create_file(char *name);

/* Removes the file from the sd card
 * @ name : is the path to the file*/
void fhl_remove_file(char *name);

/* creates a directory
 * @ name: is the path to the directory
 */
void fhl_create_dir(char *name);

/* checks the free space in the sd card*/
void fhl_check_sd(void);

/* checks the details of the file
 * @ name : is the path to the file
 */
void fhl_check_file(char *name);

/* updates the file. write pointer is set to the end of the file
 * @ name : is the path to the file
 */
void fhl_update_file(char *name);

FRESULT fhl_read_chunk(char *name, void *out_buffer, UINT byte_to_read, FSIZE_t read_offset, UINT *byte_read);

FRESULT fhl_read_stream_data(char *name, UINT (*callback_func)(const BYTE*, UINT));

/**
 * @brief Register a user function to get result from file_handling activity,
 * user can handle his own implement with the result (sending UART PC ...)
 *
 *
 * @param presult
 */
void fhl_register_notify_status(void (*presult)(char *str_result));

void fhl_register_notify_error(void (*perr)(char *str_error));
void fhl_init(char *buff, size_t buff_size, char *path_dir, size_t path_dir_size);

#ifdef __cplusplus
}
#endif
#endif /* FILE_HANDLING_H_ */
