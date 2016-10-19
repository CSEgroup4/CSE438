#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <linux/errno.h>
#include <sys/ioctl.h>

#define PAGESIZE 64
#define PAGE_COUNT 512
#define DEVICE_NAME "i2c_flash"
#define I2C_FLASH "/dev/i2c_flash"

int page, i2c_flash = 0;

enum ioctl_id {
    FLASHGETS = 1001,
    FLASHGETP = 1002,
    FLASHSETP = 1003,
    FLASHERASE = 1004
};

//set up the signals for following I2C bus operations and to initialize the current page position to 0.
int EEPROM_init()
{
	page = 0;
	
	i2c_flash = open(I2C_FLASH,O_RDWR);
	
	if (i2c_flash < 0)
	{
		printf("Cannot open device!!");
		return -1;
	}
	else 
		return 0;
}

// read a sequence of count pages from the eeprom device into the user memory pointed by buf.
int EEPROM_read(void* buf, int count)
{
	if (count > 1 || count < PAGE_COUNT)
	{
		int i;
		for (i = 0; i < count ; i++) {
			if ( read(i2c_flash,buf,count) < 0)
			{
				printf("\nReading EEPROM failed!\n");
				return -1;
			}
			else
					printf("Data from EEPROM: %s \n",(char *)buf);

		}
	}
	else
	{	printf("\nout of bounds!\n");
		return -1;}
	return 0;

}

int EEPROM_write(void* buf, int count)
{
	int i = 0;
	int j;
	char transfer[PAGESIZE];
	printf("Writing random data to the EEPROM! Please wait..\n");
	if (count > 1 || count < PAGE_COUNT)
	{
		for (j = 0; j < count ; j++) {
			while(i < PAGESIZE)
			{
				transfer[i] = (rand()%26)+97;
				i++;
			}
			memcpy(buf, transfer, sizeof(buf));
			write(i2c_flash,buf,page);
			memset(buf,'\0',count*PAGESIZE);
		}
	}
	else
		{
		printf("\nout of bounds!\n");
		return -1;
		}
	return 0;
}

int EEPROM_set(int new_position)
{
	if (new_position > 1 || new_position <= PAGE_COUNT)
	{
		if ( ioctl(i2c_flash,FLASHSETP,&page) == -1)
			{
			printf("Failed to set page on EEPROM!");
			return -1;
			}
		else
			printf("\nPage %d set on EEPROM successfully!\n",page);
	}
	else 
		{
		printf("\nout of bounds!\n");
		return -1;		
		}
	return 0;
}

int EEPROM_erase()
{
	printf("Erasing the EEPROM! Please wait..\n");
	if (ioctl(i2c_flash,FLASHERASE) == -1)
		{
		printf("Failed to erase EEPROM!");
		return -1;
		}
	else
		printf("\nEEPROM erased successfully!\n");
	return 0;
}
int main()
{
	int progress = 0;
	int option = 0;
	char *buf;
	buf = (char *) malloc(PAGESIZE);
	progress = EEPROM_init();	
	if (progress != 0)
	{
	 	do
		{
		printf("EEPROM MENUE\n");
		printf("1 to read pages\n");
		printf("2 to write pages\n");
		printf("3 set pages\n");
		printf("4 to erase\n");
		printf("5 to quit\n");
		printf("Please enter an option: ");
		scanf("%d",&option);
		switch(option)
			{
				case 1: //read
					printf("Enter number of pages to read: ");
					scanf("%d",&page);
					progress = EEPROM_read(buf, page);
					break;
				case 2: //write
					printf("Enter number of pages to write: ");
					scanf("%d",&page);
					progress = EEPROM_write(buf, page);
					break;
				case 3: // set
					printf("Enter a page number to set(0-511): ");
					scanf("%d",&page);
					progress = EEPROM_set(page);
					break;
				case 4: // erase
					progress = EEPROM_erase();
					break;
				case 5: //exit
					printf("Quiting Application");
					break;
				default:
						printf("\nPlease enter a valid option!\n");
						break;
			}

		}while(option !=5);
	}	
	return 0;
}
