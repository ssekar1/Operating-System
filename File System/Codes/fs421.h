#ifndef FS_FS421_H
#define FS_FS421_H

#include <linux/fs.h>
#include <linux/pagemap.h>
/* Following really belongs in include/uapi/linux/ */
#define CANONICAL
#include "fs421_fs.h"
/*
 *
 *  Modified from minix fs for use as exercise for UMBC CMSC421 OS course
 *      by John Y. Park
 */
#define INODE_VERSION(inode)	fs421_sb(inode->i_sb)->s_version
#define FS421_V2		0x0002		/* fs421 V2 fs */

/*
 * fs421 fs inode data in memory
 *
 * HINT-STUB: You can add to this data structure to store your own
 * per-inode information
 */
struct fs421_inode_info {
	/* Used to store in-core block pointers */
	__u32 i2_data[16];
	struct inode vfs_inode;
};

/*
 * fs421 super-block data in memory
 *
 * HINT-STUB: You can add to this data structure to store your own
 * filesystem-wide information
 */
struct fs421_sb_info {
  unsigned long s_ninodes;
  unsigned long s_nzones;
  unsigned long s_imap_blocks;
  unsigned long s_zmap_blocks;
  unsigned long s_firstdatazone;
  unsigned long s_log_zone_size;
  unsigned long s_max_size;
  int s_dirsize;
  int s_namelen;
  struct buffer_head ** s_imap;
  struct buffer_head ** s_zmap;
  struct buffer_head * s_sbh;
  struct fs421_super_block * s_ms;
  unsigned short s_mount_state;
  unsigned short s_version;
  /*
   * HINT-STUB:
	 * Following should hold precomputed offsets into zmap for
	 * each Binary Buddy level, in bits from start of map
	 * (see fs421_init_zmap_offsets())
	 */
  __u32 s_zmap_offset[32 + 1]; /* need 1 extra for end of last */
  unsigned s_zmap_nlevels;	/* number of levels in BB map */
 
};
//additional added functions
extern int find(int blockNum);
extern int test_Offsets(int level, __u32 *offset, struct buffer_head **adjust_block, int total_bits_Cal);
extern void set_Offsets(int levelOffset, int level, __u32 *offset, struct buffer_head **adjust_block, int total_bits_Cal);
extern void clear_Offsets(int levelOffset, int level, __u32 *offset, struct buffer_head **adjust_block, int total_bits_Cal);
extern unsigned B_Buddy(int level, __u32 *offset, struct buffer_head **adjust_block, int total_bits_Cal);


unsigned long fs421_init_zmap_offsets(struct fs421_sb_info *);

extern struct inode *fs421_iget(struct super_block *, unsigned long);
extern struct fs421_inode * fs421_raw_inode(struct super_block *, ino_t, struct buffer_head **);
extern struct inode * fs421_new_inode(const struct inode *, umode_t, int *);
extern void fs421_free_inode(struct inode * inode);
extern unsigned long fs421_count_free_inodes(struct super_block *sb);

extern int fs421_new_block(struct inode * inode, unsigned size_log2);
extern void fs421_free_block(struct inode *, unsigned long block, unsigned size);

extern unsigned long fs421_count_free_blocks(struct super_block *sb);
extern int fs421_getattr(struct vfsmount *, struct dentry *, struct kstat *);
extern int fs421_prepare_chunk(struct page *page, loff_t pos, unsigned len);
extern void fs421_truncate(struct inode *);
extern void fs421_set_inode(struct inode *, dev_t);
extern int fs421_get_block(struct inode *, sector_t, struct buffer_head *, int);
#ifdef CANONICAL
extern int fs421_release(struct inode *, struct file *);
#endif
extern unsigned fs421_blocks(loff_t, struct super_block *);

extern struct fs421_dir_entry *fs421_find_entry(struct dentry*, struct page**);
extern int fs421_add_link(struct dentry*, struct inode*);
extern int fs421_delete_entry(struct fs421_dir_entry*, struct page*);
extern int fs421_make_empty(struct inode*, struct inode*);
extern int fs421_empty_dir(struct inode*);
extern void fs421_set_link(struct fs421_dir_entry*, struct page*, struct inode*);
extern struct fs421_dir_entry *fs421_dotdot(struct inode*, struct page**);
extern ino_t fs421_inode_by_name(struct dentry*);

extern const struct inode_operations fs421_file_inode_operations;
extern const struct inode_operations fs421_dir_inode_operations;
extern const struct file_operations fs421_file_operations;
extern const struct file_operations fs421_dir_operations;

static inline struct fs421_sb_info *fs421_sb(struct super_block *sb)
{
	return sb->s_fs_info;
}

static inline struct fs421_inode_info *fs421_i(struct inode *inode)
{
	return list_entry(inode, struct fs421_inode_info, vfs_inode);
}

static inline unsigned fs421_blocks_needed(unsigned bits, unsigned blocksize)
{
	return DIV_ROUND_UP(bits, blocksize * 8);
}

/*
 * For sake of simplicity, made fs421 little-endian, and removed all
 * dependencies on following preprocs:
 *   CONFIG_FS421_FS_NATIVE_ENDIAN
 *   CONFIG_FS421_FS_BIG_ENDIAN_16BIT_INDEXED
 *     --jyp
 */
#endif /* FS_FS421_H */
