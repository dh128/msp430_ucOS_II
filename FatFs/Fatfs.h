#ifndef _FATFS
#define _FATFS	8237	/* Revision ID */

#include <Ffconf.h>		/* FatFs configuration options */


#ifdef __cplusplus
extern "C" {
#endif

#if _FATFS != _FFCONF
#error Wrong configuration file (ffconf.h).
#endif


/* Definitions of volume management */

#if _MULTI_PARTITION		/* Multiple partition configuration */
#define LD2PD(vol) (VolToPart[vol].pd)	/* Get physical drive# */
#define LD2PT(vol) (VolToPart[vol].pt)	/* Get partition# */
typedef struct {
	BYTE pd;	/* Physical drive# */
	BYTE pt;	/* Partition # (0-3) */
} PARTITION;
extern const PARTITION VolToPart[];	/* Volume - Physical location resolution table */

#else						/* Single partition configuration */
#define LD2PD(vol) (vol)	/* Logical drive# is bound to the same physical drive# */
#define LD2PT(vol) 0		/* Always mounts the 1st partition */

#endif



/* Type of path name strings on FatFs API */

#if _LFN_UNICODE			/* Unicode string */
#if !_USE_LFN
#error _LFN_UNICODE must be 0 in non-LFN cfg.
#endif
#ifndef _INC_TCHAR
typedef WCHAR TCHAR;
#define _T(x) L ## x
#define _TEXT(x) L ## x
#endif

#else						/* ANSI/OEM string */
#ifndef _INC_TCHAR
#define _T(x) x
#define _TEXT(x) x
#endif

#endif



/* File system object structure (FATFS) */

typedef struct {
	uint8_t		fs_type;		/* FAT sub-type (0:Not mounted) */
	uint8_t		drv;			/* Physical drive number */
	uint8_t		csize;			/* Sectors per cluster (1,2,4...128) */
	uint8_t		n_fats;			/* Number of FAT copies (1,2) */
	uint8_t		wflag;			/* win[] dirty flag (1:must be written back) */
	uint8_t		fsi_flag;		/* fsinfo dirty flag (1:must be written back) */
	uint16_t	id;				/* File system mount ID */
	uint16_t	n_rootdir;		/* Number of root directory entries (FAT12/16) */
#if _MAX_SS != 512
	uint16_t	ssize;			/* Bytes per sector (512,1024,2048,4096) */
#endif
#if _FS_REENTRANT
	_SYNC_t	sobj;			/* Identifier of sync object */
#endif
#if !_FS_READONLY
	uint32_t	last_clust;		/* Last allocated cluster */
	uint32_t	free_clust;		/* Number of free clusters */
	uint32_t	fsi_sector;		/* fsinfo sector (FAT32) */
#endif
#if _FS_RPATH
	uint32_t	cdir;			/* Current directory start cluster (0:root) */
#endif
	uint32_t	n_fatent;		/* Number of FAT entries (= number of clusters + 2) */
	uint32_t	fsize;			/* Sectors per FAT */
	uint32_t	fatbase;		/* FAT start sector */
	uint32_t	dirbase;		/* Root directory start sector (FAT32:Cluster#) */
	uint32_t	database;		/* Data start sector */
	uint32_t	winsect;		/* Current sector appearing in the win[] */
	uint8_t	win[_MAX_SS];	/* Disk access window for Directory, FAT (and Data on tiny cfg) */
} FATFS;



/* File object structure (FIL) */

typedef struct {
	FATFS*		fs;				/* Pointer to the owner file system object */
	uint16_t	id;				/* Owner file system mount ID */
	uint8_t		flag;			/* File status flags */
	uint8_t		pad1;
	uint32_t	fptr;			/* File read/write pointer (0 on file open) */
	uint32_t	fsize;			/* File size */
	uint32_t	sclust;			/* File start cluster (0 when fsize==0) */
	uint32_t	clust;			/* Current cluster */
	uint32_t	dsect;			/* Current data sector */
#if !_FS_READONLY
	uint32_t	dir_sect;		/* Sector containing the directory entry */
	uint8_t*	dir_ptr;		/* Ponter to the directory entry in the window */
#endif
#if _USE_FASTSEEK
	uint32_t*	cltbl;			/* Pointer to the cluster link map table (null on file open) */
#endif
#if _FS_SHARE
	uint16_t	lockid;			/* File lock ID (index of file semaphore table) */
#endif
#if !_FS_TINY
	uint8_t	buf[_MAX_SS];	/* File data read/write buffer */
#endif
} FIL;



/* Directory object structure (DIR) */

typedef struct {
	FATFS*		fs;				/* Pointer to the owner file system object */
	uint16_t	id;				/* Owner file system mount ID */
	uint16_t	index;			/* Current read/write index number */
	uint32_t	sclust;			/* Table start cluster (0:Root dir) */
	uint32_t	clust;			/* Current cluster */
	uint32_t	sect;			/* Current sector */
	uint8_t		*dir;			/* Pointer to the current SFN entry in the win[] */
	uint8_t		*fn;				/* Pointer to the SFN (in/out) {file[8],ext[3],status[1]} */
#if _USE_LFN
	uint16_t*	lfn;			/* Pointer to the LFN working buffer */
	uint16_t	lfn_idx;		/* Last matched LFN index number (0xFFFF:No LFN) */
#endif
} DIR;



/* File status structure (FILINFO) */

typedef struct {
	uint32_t	fsize;			/* File size */
	uint16_t	fdate;			/* Last modified date */
	uint16_t	ftime;			/* Last modified time */
	uint8_t	fattrib;		/* Attribute */
	char	fname[13];		/* Short file name (8.3 format) */
#if _USE_LFN
	char*	lfname;			/* Pointer to the LFN buffer */
	uint16_t 	lfsize;			/* Size of LFN buffer in TCHAR */
#endif
} FILINFO;



/* File function return code (FRESULT) */

typedef enum {
	FR_OK = 0,				/* (0) Succeeded */
	FR_DISK_ERR,			/* (1) A hard error occured in the low level disk I/O layer */
	FR_INT_ERR,				/* (2) Assertion failed */
	FR_NOT_READY,			/* (3) The physical drive cannot work */
	FR_NO_FILE,				/* (4) Could not find the file */
	FR_NO_PATH,				/* (5) Could not find the path */
	FR_INVALID_NAME,		/* (6) The path name format is invalid */
	FR_DENIED,				/* (7) Acces denied due to prohibited access or directory full */
	FR_EXIST,				/* (8) Acces denied due to prohibited access */
	FR_INVALID_OBJECT,		/* (9) The file/directory object is invalid */
	FR_WRITE_PROTECTED,		/* (10) The physical drive is write protected */
	FR_INVALID_DRIVE,		/* (11) The logical drive number is invalid */
	FR_NOT_ENABLED,			/* (12) The volume has no work area */
	FR_NO_FILESYSTEM,		/* (13) There is no valid FAT volume on the physical drive */
	FR_MKFS_ABORTED,		/* (14) The f_mkfs() aborted due to any parameter error */
	FR_TIMEOUT,				/* (15) Could not get a grant to access the volume within defined period */
	FR_LOCKED,				/* (16) The operation is rejected according to the file shareing policy */
	FR_NOT_ENOUGH_CORE,		/* (17) LFN working buffer could not be allocated */
	FR_TOO_MANY_OPEN_FILES	/* (18) Number of open files > _FS_SHARE */
} FRESULT;



/*--------------------------------------------------------------*/
/* FatFs module application interface                           */

FRESULT f_mount (uint8_t, FATFS*);				/* Mount/Unmount a logical drive */
FRESULT f_open (FIL*, const char*, uint8_t);			/* Open or create a file */
FRESULT f_read (FIL*, void*, uint16_t, uint16_t*);			/* Read data from a file */
FRESULT f_lseek (FIL*, uint32_t);						/* Move file pointer of a file object */
FRESULT f_close (FIL*);								/* Close an open file object */
FRESULT f_opendir (DIR*, const char*);				/* Open an existing directory */
FRESULT f_readdir (DIR*, FILINFO*);					/* Read a directory item */
FRESULT f_stat (const char*, FILINFO*);			/* Get file status */
FRESULT f_write (FIL*, const void*, uint16_t, uint16_t*);	/* Write data to a file */
FRESULT f_getfree (const char*, uint32_t*, FATFS**);	/* Get number of free clusters on the drive */
FRESULT f_truncate (FIL*);					/* Truncate file */
FRESULT f_sync (FIL*);								/* Flush cached data of a writing file */
FRESULT f_unlink (const char*);					/* Delete an existing file or directory */
FRESULT	f_mkdir (const char*);						/* Create a new directory */
FRESULT f_chmod (const char*, uint8_t, uint8_t);			/* Change attriburte of the file/dir */
FRESULT f_utime (const char*, const FILINFO*);		/* Change timestamp of the file/dir */
FRESULT f_rename (const char*, const char*);		/* Rename/Move a file or directory */
FRESULT f_forward (FIL*, uint16_t(*)(const uint8_t*,uint16_t), uint16_t, uint16_t*);	/* Forward data to the stream */
FRESULT f_mkfs (uint8_t, uint8_t, uint16_t);					/* Create a file system on the drive */
FRESULT f_chdrive (uint8_t);							/* Change current drive */
FRESULT f_chdir (const char*);						/* Change current directory */
FRESULT f_getcwd (char*, uint16_t);					/* Get current directory */
int f_putc (char, FIL*);							/* Put a character to the file */
int f_puts (const char*, FIL*);					/* Put a string to the file */
int f_printf (FIL*, const char*, ...);				/* Put a formatted string to the file */
char* f_gets (char*, int, FIL*);					/* Get a string from the file */

#ifndef EOF
#define EOF (-1)
#endif

#define f_eof(fp) (((fp)->fptr == (fp)->fsize) ? 1 : 0)
#define f_error(fp) (((fp)->flag & FA__ERROR) ? 1 : 0)
#define f_tell(fp) ((fp)->fptr)
#define f_size(fp) ((fp)->fsize)




/*--------------------------------------------------------------*/
/* Additional user defined functions                            */

/* RTC function */
#if !_FS_READONLY
uint32_t get_fattime (void);
#endif

/* Unicode support functions */
#if _USE_LFN						/* Unicode - OEM code conversion */
WCHAR ff_convert (WCHAR, UINT);		/* OEM-Unicode bidirectional conversion */
WCHAR ff_wtoupper (WCHAR);			/* Unicode upper-case conversion */
#if _USE_LFN == 3					/* Memory functions */
void* ff_memalloc (UINT);			/* Allocate memory block */
void ff_memfree (void*);			/* Free memory block */
#endif
#endif

/* Sync functions */
#if _FS_REENTRANT
int ff_cre_syncobj (BYTE, _SYNC_t*);/* Create a sync object */
int ff_req_grant (_SYNC_t);			/* Lock sync object */
void ff_rel_grant (_SYNC_t);		/* Unlock sync object */
int ff_del_syncobj (_SYNC_t);		/* Delete a sync object */
#endif




/*--------------------------------------------------------------*/
/* Flags and offset address                                     */


/* File access control and file status flags (FIL.flag) */

#define	FA_READ				0x01
#define	FA_OPEN_EXISTING	0x00
#define FA__ERROR			0x80

#if !_FS_READONLY
#define	FA_WRITE			0x02
#define	FA_CREATE_NEW		0x04
#define	FA_CREATE_ALWAYS	0x08
#define	FA_OPEN_ALWAYS		0x10
#define FA__WRITTEN			0x20
#define FA__DIRTY			0x40
#endif


/* FAT sub type (FATFS.fs_type) */

#define FS_FAT12	1
#define FS_FAT16	2
#define FS_FAT32	3


/* File attribute bits for directory entry */

#define	AM_RDO	0x01	/* Read only */
#define	AM_HID	0x02	/* Hidden */
#define	AM_SYS	0x04	/* System */
#define	AM_VOL	0x08	/* Volume label */
#define AM_LFN	0x0F	/* LFN entry */
#define AM_DIR	0x10	/* Directory */
#define AM_ARC	0x20	/* Archive */
#define AM_MASK	0x3F	/* Mask of defined bits */


/* Fast seek function */
#define CREATE_LINKMAP	0xFFFFFFFF



/*--------------------------------*/
/* Multi-byte word access macros  */

#if _WORD_ACCESS == 1	/* Enable word access to the FAT structure */
#define	LD_WORD(ptr)		(uint16_t)(*(uint16_t*)(uint8_t*)(ptr))
#define	LD_DWORD(ptr)		(uint32_t)(*(uint32_t*)(uint8_t*)(ptr))
#define	ST_WORD(ptr,val)	*(uint16_t*)(uint8_t*)(ptr)=(uint16_t)(val)
#define	ST_DWORD(ptr,val)	*(uint32_t*)(uint8_t*)(ptr)=(uint32_t)(val)
#else					/* Use byte-by-byte access to the FAT structure */
#define	LD_WORD(ptr)		(uint16_t)(((uint16_t)*((uint8_t*)(ptr)+1)<<8)|(uint16_t)*(uint8_t*)(ptr))
#define	LD_DWORD(ptr)		(uint32_t)(((uint32_t)*((uint8_t*)(ptr)+3)<<24)|((uint32_t)*((uint8_t*)(ptr)+2)<<16)|((uint16_t)*((uint8_t*)(ptr)+1)<<8)|*(uint8_t*)(ptr))
#define	ST_WORD(ptr,val)	*(uint8_t*)(ptr)=(uint8_t)(val); *((uint8_t*)(ptr)+1)=(uint8_t)((uint16_t)(val)>>8)
#define	ST_DWORD(ptr,val)	*(uint8_t*)(ptr)=(uint8_t)(val); *((uint8_t*)(ptr)+1)=(uint8_t)((uint16_t)(val)>>8); *((uint8_t*)(ptr)+2)=(uint8_t)((uint32_t)(val)>>16); *((uint8_t*)(ptr)+3)=(uint8_t)((uint32_t)(val)>>24)
#endif

#ifdef __cplusplus
}
#endif

#endif /* _FATFS */
