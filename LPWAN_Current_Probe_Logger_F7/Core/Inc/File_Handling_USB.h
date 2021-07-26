/*
 * File_Handling_USB_RTOS.h
 *
 *  Created on: 30-April-2020
 *      Author: Controllerstech
 *      Modified: MikulasekMichal_21-July-2021
 */

#ifndef FILE_HANDLING_USB_H_
#define FILE_HANDLING_USB_H_

#include "fatfs.h"
#include "string.h"
#include "stdio.h"

#define FILE_NAME_SIZE 256

/* mounts the USB*/
void Mount_USB (void);

/* unmounts the USB*/
void Unmount_USB (void);

/* Start node to be scanned (***also used as work area***) */
FRESULT Scan_USB (char* pat);

/* Only supports removing files from home directory. Directory remover to be added soon */
FRESULT Format_USB (void);

/* write the data to the file
 * @ name : is the path to the file*/
FRESULT Write_File_USB (char *name, char *data);

/* read data from the file
 * @ name : is the path to the file*/
FRESULT Read_File_USB (char *name);

/* creates the file, if it does not exists
 * @ name : is the path to the file*/
FRESULT Create_File_USB (char *name);

/* Removes the file from the USB
 * @ name : is the path to the file*/
FRESULT Remove_File_USB (char *name);

/* creates a directory
 * @ name: is the path to the directory
 */
FRESULT Create_Dir_USB (char *name);

/* checks the free space in the USB*/
void Check_USB_Details (void);

/* updates the file. write pointer is set to the end of the file
 * @ name : is the path to the file
 */
FRESULT Update_File_USB (char *name, char *data);




#endif /* FILE_HANDLING_USB_RTOS_H_ */
