/*
 * file_handling.c
 *
 *  Created on: 26-Jul-2019
 *      Author: arunr
 *      https://controllerstech.com/interface-sd-card-with-sdio-in-stm32/
 */

#include "file_handling.h"

FATFS fs;  // file system
FIL fil; // File
FILINFO fno;
FRESULT fresult;  // result
UINT br, bw;  // File read/write count

/**** capacity related *****/
FATFS *pfs;
DWORD fre_clust;
uint32_t total, free_space;

static char *buffer;  // pointer to store strings..
static uint16_t buffer_size;
static char *path;  // buffer to store path
static uint16_t path_size;
static presult _presult;
static perr _perr;

int i = 0;
static int wait_until(char *str, char *buf);
static void notify_result_to_user(char *str_result) {
	if (_presult != NULL) _presult(str_result);
}

static void notify_error_to_user(char *str_error) {
	if (_perr != NULL) _perr(str_error);
}

static void fhl_clear_buffer(void) {
	if (buffer_size == 0) while (1);
	memset(buffer, 0, buffer_size);
}

void fhl_get_path(void) {
	int start = fhl_cmdlength(buffer) + 1;
	int end = fhl_get_content_length(buffer) - 2;

	int j = 0;
	for (int i = start; i < end; i++) {
		if (buffer[i] != ' ')
			path[j++] = buffer[i];
		else
			break;
	}
}

/* Start node to be scanned (***also used as work area***) */
FRESULT fhl_scan_files(char *pat) {
	DIR dir;
	UINT i;

	char path[_MAX_LFN + 2];
	sprintf(path, "%s", pat);

	fresult = f_opendir(&dir, path); /* Open the directory */
	if (fresult == FR_OK) {
		for (;;) {
			fresult = f_readdir(&dir, &fno); /* Read a directory item */
			if (fresult != FR_OK || fno.fname[0] == 0) break; /* Break on error or end of dir */
			if (fno.fattrib & AM_DIR) /* It is a directory */
			{
				if (!(strcmp("SYSTEM~1", fno.fname))) continue;
				sprintf(buffer, "Dir: %s\r\n", fno.fname);
				notify_result_to_user(buffer);
				i = strlen(path);
				sprintf(&path[i], "/%s", fno.fname);
				fresult = fhl_scan_files(path); /* Enter the directory */
				if (fresult != FR_OK) break;
				path[i] = 0;
			}
			else { /* It is a file. */
				sprintf(buffer, "File: %s/%s\n", path, fno.fname);
				notify_result_to_user(buffer);
			}
		}
		f_closedir(&dir);
	}
	return fresult;
}

void fhl_write_file(char *name) {

	/**** check whether the file exists or not ****/
	fresult = f_stat(name, &fno);
	if (fresult != FR_OK) {
		sprintf(buffer, "*%s* does not exists\n", name);
		notify_error_to_user(buffer);
	}

	else {
		/* Create a file with read write access and open it */
		fresult = f_open(&fil, name, FA_OPEN_EXISTING | FA_WRITE);
		if (fresult != FR_OK) {
			sprintf(buffer, "error no %d in opening file *%s*\n", fresult, name);
			notify_error_to_user(buffer);
		}

		else {
			sprintf(buffer, "file *%s* is opened. Now enter the string you want to write\n", name);
			notify_result_to_user(buffer);
		}

		while (!(wait_until("\r\n", buffer)));

		/* Writing text */

		fresult = f_write(&fil, buffer, fhl_get_content_length(buffer), &bw);

		if (fresult != FR_OK) {
			fhl_clear_buffer();
			sprintf(buffer, "error no %d in writing file *%s*\n", fresult, name);
			notify_error_to_user(buffer);
		}

		else {
			fhl_clear_buffer();
			sprintf(buffer, "*%s* written successfully\n", name);
			notify_result_to_user(buffer);
		}

		/* Close file */
		fresult = f_close(&fil);
		if (fresult != FR_OK) {
			sprintf(buffer, "error no %d in closing file *%s*\n", fresult, name);
			notify_error_to_user(buffer);
		}
	}
}

void fhl_read_file(char *name) {
	/**** check whether the file exists or not ****/
	fresult = f_stat(name, &fno);
	if (fresult != FR_OK) {
		sprintf(buffer, "*%s* does not exists\n", name);
		notify_error_to_user(buffer);
	}
	else {
		if (f_size(&fil) > buffer_size) {
			notify_error_to_user("Err: file size is bigger than buffer size");
			return;
		}
		/* Open file to read */
		fresult = f_open(&fil, name, FA_READ);
		if (fresult != FR_OK) {
			sprintf(buffer, "error no %d in opening file *%s*\n", fresult, name);
			notify_error_to_user(buffer);
		}

		/* Read data from the file
		 * see the function details for the arguments */
		sprintf(buffer, "reading data from the file *%s*\n", name);
		notify_result_to_user(buffer);

		fresult = f_read(&fil, buffer, f_size(&fil), &br);
		if (fresult != FR_OK) {
			fhl_clear_buffer();
			sprintf(buffer, "error no %d in reading file *%s*\n", fresult, name);
			notify_error_to_user(buffer);
		}

		else
			notify_result_to_user(buffer);

		/* Close file */
		fresult = f_close(&fil);
		if (fresult != FR_OK) {
			sprintf(buffer, "error no %d in closing file *%s*\n", fresult, name);
			notify_error_to_user(buffer);
		}
	}
}

void fhl_create_file(char *name) {
	fresult = f_stat(name, &fno);
	if (fresult == FR_OK) {
		sprintf(buffer, "*%s* already exists!!!!\n", name);
		notify_result_to_user(buffer);
	}
	else {
		fresult = f_open(&fil, name, FA_CREATE_ALWAYS | FA_READ | FA_WRITE);
		if (fresult != FR_OK) {
			sprintf(buffer, "error no %d in creating file *%s*\n", fresult, name);
			notify_error_to_user(buffer);
		}
		else {
			sprintf(buffer, "*%s* created successfully\n", name);
			notify_result_to_user(buffer);
		}

		fresult = f_close(&fil);
		if (fresult != FR_OK) {
			sprintf(buffer, "error no %d in closing file *%s*\n", fresult, name);
			notify_error_to_user(buffer);
		}
	}
}

void fhl_remove_file(char *name) {
	/**** check whether the file exists or not ****/
	fresult = f_stat(name, &fno);
	if (fresult != FR_OK) {
		sprintf(buffer, "*%s* does not exists\n", name);
		notify_error_to_user(buffer);
	}

	else {
		fresult = f_unlink(name);
		if (fresult == FR_OK) {
			sprintf(buffer, "*%s* has been removed successfully\n", name);
			notify_result_to_user(buffer);
		}

		else {
			sprintf(buffer, "error in removing *%s*\n", name);
			notify_error_to_user(buffer);
		}
	}

}

void fhl_create_dir(char *name) {
	fresult = f_mkdir(name);
	if (fresult == FR_OK) {
		sprintf(buffer, "*%s* has been created successfully\n", name);
		notify_result_to_user(buffer);
	}
	else {
		sprintf(buffer, "error no %d in creating directory\n", fresult);
		notify_error_to_user(buffer);
	}
}

void fhl_check_sd(void) {
	/* Check free space */
	f_getfree("", &fre_clust, &pfs);

	total = (uint32_t) ((pfs->n_fatent - 2) * pfs->csize * 0.5);
	sprintf(buffer, "SD CARD Total Size: \t%lu\n", total);
	notify_result_to_user(buffer);
	free_space = (uint32_t) (fre_clust * pfs->csize * 0.5);
	sprintf(buffer, "SD CARD Free Space: \t%lu\n", free_space);
	notify_result_to_user(buffer);
}

FRESULT fhl_read_chunk(char *name, void *out_buffer, UINT byte_to_read, FSIZE_t read_offset, UINT *byte_read) {
	/**** check whether the file exists or not ****/
	fresult = f_stat(name, &fno);
	if (fresult != FR_OK) {
		sprintf(buffer, "*%s* does not exists\n", name);
		notify_error_to_user(buffer);
	}
	else {
		/* Open file to read */
		fresult = f_open(&fil, name, FA_READ);
		if (fresult != FR_OK) {
			sprintf(buffer, "error no %d in opening file *%s*\n", fresult, name);
			notify_error_to_user(buffer);
		}

		FSIZE_t fsize = f_size(&fil);
		if (fsize < read_offset) {
			sprintf(buffer, "error read offset invalid");
			notify_error_to_user(buffer);
			return FR_INVALID_PARAMETER;
		}
		UINT valid_read_size = (fsize < (read_offset + byte_to_read)) ? fsize - read_offset : byte_to_read;
		fresult = f_lseek(&fil, read_offset);
		if (fresult != FR_OK) {
			sprintf(buffer, "error no %d in set offset read *%s*\n", fresult, name);
			notify_error_to_user(buffer);
		}
		fresult = f_read(&fil, out_buffer, valid_read_size, byte_read);
		if (valid_read_size < byte_to_read && f_eof(&fil) == 0) notify_error_to_user("Read end but not EOF");
		if (fresult != FR_OK) {
			sprintf(buffer, "error no %d in read *%s*\n", fresult, name);
			notify_error_to_user(buffer);
		}
	}
	/* Close file */
	fresult = f_close(&fil);
	if (fresult != FR_OK) {
		sprintf(buffer, "error no %d in closing file *%s*\n", fresult, name);
		notify_error_to_user(buffer);
	}
	return fresult;
}

FRESULT fhl_read_stream_data(char *name, UINT (*callback_func)(const BYTE*, UINT)) {
	FRESULT rc;
	FIL fil;
	UINT dmy;
	static uint16_t i = 0;
	/* Open the audio file in read only mode */
	rc = f_open(&fil, name, FA_READ);
	if (rc) return rc;

	/* Repeat until the file pointer reaches end of the file */
	while (rc == FR_OK && !f_eof(&fil)) {

		/* some processes... */

		/* Fill output stream periodicaly or on-demand */
		rc = f_forward(&fil, callback_func, 100, &dmy);
		i++;
	}

	/* Close the file and return */
	f_close(&fil);
	return rc;
}

void fhl_check_file(char *name) {
	fresult = f_stat(name, &fno);
	switch (fresult) {
	case FR_OK:

		sprintf(buffer, "Below are the details of the *%s* \nSize: %lu\n", name, fno.fsize);
		notify_result_to_user(buffer);
		sprintf(buffer, "Timestamp: %u/%02u/%02u, %02u:%02u\n", (fno.fdate >> 9) + 1980, fno.fdate >> 5 & 15, fno.fdate & 31, fno.ftime >> 11, fno.ftime >> 5 & 63);
		notify_result_to_user(buffer);
		sprintf(buffer, "Attributes: %c%c%c%c%c\n", (fno.fattrib & AM_DIR) ? 'D' : '-', (fno.fattrib & AM_RDO) ? 'R' : '-', (fno.fattrib & AM_HID) ? 'H' : '-', (fno.fattrib & AM_SYS) ? 'S' : '-', (fno.fattrib & AM_ARC) ? 'A' : '-');
		notify_result_to_user(buffer);
		break;

	case FR_NO_FILE:
		sprintf(buffer, "*%s* does not exist.\n", name);
		notify_error_to_user(buffer);
		break;

	default:
		sprintf(buffer, "An error occurred. (%d)\n", fresult);
		notify_error_to_user(buffer);
	}
}

void fhl_update_file(char *name) {
	/**** check whether the file exists or not ****/
	fresult = f_stat(name, &fno);
	if (fresult != FR_OK) {
		sprintf(buffer, "*%s* does not exists\n", name);
		notify_error_to_user(buffer);
	}

	else {
		/* Create a file with read write access and open it */
		fresult = f_open(&fil, name, FA_OPEN_APPEND | FA_WRITE);
		if (fresult != FR_OK) {
			sprintf(buffer, "error no %d in opening file *%s*\n", fresult, name);
			notify_error_to_user(buffer);
		}

		else {
			sprintf(buffer, "file *%s* is opened. Now enter the string you want to update\n", name);
			notify_result_to_user(buffer);
		}

		while (!(wait_until("\r\n", buffer)));

		/* Writing text */

		fresult = f_write(&fil, buffer, fhl_get_content_length(buffer), &bw);

		if (fresult != FR_OK) {
			fhl_clear_buffer();
			sprintf(buffer, "error no %d in writing file *%s*\n", fresult, name);
			notify_error_to_user(buffer);
		}

		else {
			fhl_clear_buffer();
			sprintf(buffer, "*%s* written successfully\n", name);
			notify_error_to_user(buffer);
		}

		/* Close file */
		fresult = f_close(&fil);
		if (fresult != FR_OK) {
			sprintf(buffer, "error no %d in closing file *%s*\n", fresult, name);
			notify_error_to_user(buffer);
		}
	}
}

int fhl_get_content_length(char *buf) {
	int i = 0;
	while (*buf++ != '\0')
		i++;
	return i;
}

int wait_until(char *str, char *buf) {
	return 0;
}

static void fhl_clear_path(void) {
	if (path_size == 0) while (1);
	memset(path, 0, path_size);
}

int fhl_cmdlength(char *str) {
	int i = 0;
	while (*str++ != ' ')
		i++;
	return i;
}

void fhl_mount_sd(void) {
	fresult = f_mount(&fs, "/", 1);
	if (fresult != FR_OK)
		notify_error_to_user("error in mounting SD CARD...\n");
	else
		notify_result_to_user("SD CARD mounted successfully...\n");
}

void fhl_unmount_sd(void) {
	fresult = f_mount(NULL, "/", 1);
	if (fresult == FR_OK)
		notify_result_to_user("SD CARD UNMOUNTED successfully...\n");
	else
		notify_error_to_user("error!!! in UNMOUNTING SD CARD\n");
	fhl_clear_path();
	fhl_clear_buffer();
}

void fhl_register_notify_status(void (*presult)(char *str_result)) {
	_presult = presult;
}

void fhl_register_notify_error(void (*perr)(char *str_error)) {
	_perr = perr;
}

void fhl_init(char *buff, size_t buff_size, char *path_dir, size_t path_dir_size) {
	buffer = buff;
	buffer_size = buff_size;
	path = path_dir;
	path_size = path_dir_size;
}
