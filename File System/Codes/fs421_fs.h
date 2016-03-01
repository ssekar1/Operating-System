#ifndef _LINUX_FS421_FS_H
#define _LINUX_FS421_FS_H

#include <linux/types.h>
#include <linux/magic.h>

/*
 * The fs421 filesystem constants/structures
 *
 *  Modified from minix fs for use as exercise for UMBC CMSC421 OS course
 *      by John Y. Park
 */

/* Should eventually move this line to include/uapi/linux/magic.h --jyp */
#define FS421_SUPER_MAGIC	0x3234		/* Why this #? :-) */

#define FS421_ROOT_INO 1

/* Not the same as the bogus LINK_MAX in <linux/limits.h>. Oh well. */
#define FS421_LINK_MAX	65530

#define FS421_VALID_FS		0x0001		/* Clean fs. */
#define FS421_ERROR_FS		0x0002		/* fs has errors. */

/*
 * The layout of the inode on disk in fs421.  Size is 64 bytes.
 * (Based on V2 minix fs, but block pointer layout/interpretation
 * completely different from minix)
 */
struct fs421_inode {
	__u16 i_mode;
	__u16 i_nlinks;
	__u16 i_uid;
	__u16 i_gid;
	__u32 i_size;
	__u32 i_atime;
	__u32 i_mtime;
	__u32 i_ctime;
	__u32 i_chunk[10];
};

/*
 * fs421 super-block data on disk
 */
struct fs421_super_block {
	__u16 s_ninodes; /* Total # inodes */
	__u16 s_unused1;/* used to be s_nzones; not used in v2 (use s_zones);
			 * Gone in v3--jyp */
	__u16 s_imap_blocks;	/* Bitmap is indexed directly by inode#
				 * (so, 1-origin; bit 0 always set) */
	__u16 s_zmap_blocks;	/* Note that bitmap of used zones is zone# + 1
				 * (so, 1-origin again, w/bit 0 always set) */
	__u16 s_firstdatazone;	/* Actual index of first zone of data region */
	__u16 s_log_zone_size;	/* In units of s_blocksize; currently 0 */
	__u32 s_max_size;	/* Inited to 2^31 - 1 */
	__u16 s_magic;		/* ?? for FS421 */
	__u16 s_state;		/* Either 1 (ok) or 2 (broken) */
	__u32 s_zones;		/* number of zones on entire partition */
	/* Everything beyond here is new (w.r.t. v2 minix) */
	__u16 s_blocksize;	/* added in v3 minix --jyp */
	__u8  s_disk_version;
};

struct fs421_dir_entry {
	__u16 inode;
	char name[14]; /* was 0--jyp */
};
#endif
