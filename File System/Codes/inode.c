/*
 *  linux/fs/fs421/inode.c
 *
 *  Copyright (C) 1991, 1992  Linus Torvalds
 *
 *  Copyright (C) 1996  Gertjan van Wingerde
 *	Minix V2 fs support.
 *
 *  Modified for 680x0 by Andreas Schwab
 *  Updated to filesystem version 3 by Daniel Aragones
 *
 *  Modified for use as exercise for UMBC CMSC421 OS course
 *      by John Y. Park
 */

#include <linux/module.h>
#include "fs421.h"
#include <linux/buffer_head.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/highuid.h>
#include <linux/vfs.h>
#include <linux/writeback.h>

static int fs421_write_inode(struct inode *inode,
		struct writeback_control *wbc);
static int fs421_statfs(struct dentry *dentry, struct kstatfs *buf);
static int fs421_remount (struct super_block * sb, int * flags, char * data);

static void fs421_evict_inode(struct inode *inode)
{
	truncate_inode_pages(&inode->i_data, 0);
	if (!inode->i_nlink) {
		inode->i_size = 0;
		fs421_truncate(inode);
	}
	invalidate_inode_buffers(inode);
	clear_inode(inode);
	if (!inode->i_nlink)
		fs421_free_inode(inode);
}

static void fs421_put_super(struct super_block *sb)
{
	int i;
	struct fs421_sb_info *sbi = fs421_sb(sb);

	if (!(sb->s_flags & MS_RDONLY)) {
		/* Removed V3 stuff--following line is v1/v2  --jyp */
		sbi->s_ms->s_state = sbi->s_mount_state;

		mark_buffer_dirty(sbi->s_sbh);
	}
	for (i = 0; i < sbi->s_imap_blocks; i++)
		brelse(sbi->s_imap[i]);
	for (i = 0; i < sbi->s_zmap_blocks; i++)
		brelse(sbi->s_zmap[i]);
	brelse (sbi->s_sbh);
	kfree(sbi->s_imap);  /* Note: this frees up both imap & zmap --jyp */
	sb->s_fs_info = NULL;
	kfree(sbi);
}

static struct kmem_cache * fs421_inode_cachep;

static struct inode *fs421_alloc_inode(struct super_block *sb)
{
	struct fs421_inode_info *ei;
	ei = (struct fs421_inode_info *)kmem_cache_alloc(fs421_inode_cachep, GFP_KERNEL);
	if (!ei)
		return NULL;
	return &ei->vfs_inode;
}

static void fs421_i_callback(struct rcu_head *head)
{
	struct inode *inode = container_of(head, struct inode, i_rcu);
	kmem_cache_free(fs421_inode_cachep, fs421_i(inode));
}

static void fs421_destroy_inode(struct inode *inode)
{
	call_rcu(&inode->i_rcu, fs421_i_callback);
}

static void init_once(void *foo)
{
	struct fs421_inode_info *ei = (struct fs421_inode_info *) foo;

	inode_init_once(&ei->vfs_inode);
}

static int init_inodecache(void)
{
	fs421_inode_cachep = kmem_cache_create("fs421_inode_cache",
					     sizeof(struct fs421_inode_info),
					     0, (SLAB_RECLAIM_ACCOUNT|
						SLAB_MEM_SPREAD),
					     init_once);
	if (fs421_inode_cachep == NULL)
		return -ENOMEM;
	return 0;
}

static void destroy_inodecache(void)
{
	/*
	 * Make sure all delayed rcu free inodes are flushed before we
	 * destroy cache.
	 */
	rcu_barrier();
	kmem_cache_destroy(fs421_inode_cachep);
}

static const struct super_operations fs421_sops = {
	.alloc_inode	= fs421_alloc_inode,
	.destroy_inode	= fs421_destroy_inode,
	.write_inode	= fs421_write_inode,
	.evict_inode	= fs421_evict_inode,
	.put_super	= fs421_put_super,
	.statfs		= fs421_statfs,
	.remount_fs	= fs421_remount,
};

static int fs421_remount (struct super_block * sb, int * flags, char * data)
{
	struct fs421_sb_info * sbi = fs421_sb(sb);
	struct fs421_super_block * ms;

	ms = sbi->s_ms;
	if ((*flags & MS_RDONLY) == (sb->s_flags & MS_RDONLY))
		return 0;
	if (*flags & MS_RDONLY) {
		if (ms->s_state & FS421_VALID_FS ||
		    !(sbi->s_mount_state & FS421_VALID_FS))
			return 0;
		/* Mounting a rw partition read-only. */
		ms->s_state = sbi->s_mount_state;
		mark_buffer_dirty(sbi->s_sbh);
	} else {
	  	/* Mount a partition which is read-only, read-write. */
		sbi->s_mount_state = ms->s_state;
		ms->s_state &= ~FS421_VALID_FS;

		mark_buffer_dirty(sbi->s_sbh);

		if (!(sbi->s_mount_state & FS421_VALID_FS))
			printk("FS421-fs warning: remounting unchecked fs, "
				"running fsck is recommended\n");
		else if ((sbi->s_mount_state & FS421_ERROR_FS))
			printk("FS421-fs warning: remounting fs with errors, "
				"running fsck is recommended\n");
	}
	return 0;
}

static int fs421_fill_super(struct super_block *s, void *data, int silent)
{
	struct buffer_head *bh;
	struct buffer_head **map;
	struct fs421_super_block *ms;
	unsigned long i, block;
	struct inode *root_inode;
	struct fs421_sb_info *sbi;
	int ret = -EINVAL;

	sbi = kzalloc(sizeof(struct fs421_sb_info), GFP_KERNEL);
	if (!sbi)
		return -ENOMEM;
	s->s_fs_info = sbi;

	BUILD_BUG_ON(64 != sizeof(struct fs421_inode));

	/* NB: this is changed later, after fetching actual SB --jyp */
	if (!sb_set_blocksize(s, BLOCK_SIZE))
		goto out_bad_hblock;

	if (!(bh = sb_bread(s, 1)))
		goto out_bad_sb;

	ms = (struct fs421_super_block *) bh->b_data;
	sbi->s_ms = ms;
	sbi->s_sbh = bh;
	/* Moved this up higher --jyp */
	if (ms->s_magic == FS421_SUPER_MAGIC) {
		sbi->s_version = FS421_V2;
	} else
		goto out_no_fs;

	sbi->s_mount_state = ms->s_state;
	sbi->s_ninodes = ms->s_ninodes;
	sbi->s_imap_blocks = ms->s_imap_blocks;
	sbi->s_zmap_blocks = ms->s_zmap_blocks;
	sbi->s_firstdatazone = ms->s_firstdatazone;
	sbi->s_log_zone_size = ms->s_log_zone_size;
	sbi->s_max_size = ms->s_max_size;
	s->s_magic = ms->s_magic;
	/* Following was specific to V2; Removed a bunch of V1 & V3 stuff
	 * --jyp
	 * "if (s->s_magic == FS421_SUPER_MAGIC) {"
	 */
	sbi->s_version = FS421_V2;
	sbi->s_nzones = ms->s_zones;
	sbi->s_dirsize = 16;
	sbi->s_namelen = 14;
	s->s_max_links = FS421_LINK_MAX;
	/* Following borrowed from V3; it didn't seem to hurt MINIX that
	 * this was (re)set down here  --jyp X */
	sb_set_blocksize(s, ms->s_blocksize);

	/*
	 * Allocate the buffer map to keep the superblock small.
	 */
	if (sbi->s_imap_blocks == 0 || sbi->s_zmap_blocks == 0)
		goto out_illegal_sb;
	i = (sbi->s_imap_blocks + sbi->s_zmap_blocks) * sizeof(bh);
	map = kzalloc(i, GFP_KERNEL);
	if (!map)
		goto out_no_map;
	sbi->s_imap = &map[0];
	sbi->s_zmap = &map[sbi->s_imap_blocks];

	block=2;
	for (i=0 ; i < sbi->s_imap_blocks ; i++) {
		if (!(sbi->s_imap[i]=sb_bread(s, block)))
			goto out_no_bitmap;
		block++;
	}
	for (i=0 ; i < sbi->s_zmap_blocks ; i++) {
		if (!(sbi->s_zmap[i]=sb_bread(s, block)))
			goto out_no_bitmap;
		block++;
	}

	/* Orig.; I think just for yucks  --jyp */
	__set_bit(0,(unsigned long *) sbi->s_imap[0]->b_data);

	/* Apparently minix can create filesystems that allocate more blocks for
	 * the bitmaps than needed.  We simply ignore that, but verify it didn't
	 * create one with not enough blocks and bail out if so.
	 */
	block = fs421_blocks_needed(sbi->s_ninodes, s->s_blocksize);
	if (sbi->s_imap_blocks < block) {
		printk("FS421-fs: file system does not have enough "
				"imap blocks allocated.  Refusing to mount\n");
		goto out_no_bitmap;
	}

	/*
	 * HINT-STUB:
	 * fs421_blocks_needed() is a trivial inline func that converts a
	 * bitmap size, in bits, into the number of blocks needed to hold
	 * that bitmap on the disk.  In the original MINIX fs code
	 * (pre-binary-buddy), it was just passed the number of data blocks:
	 *
	 *   block = fs421_blocks_needed(sbi->s_nzones - sbi->s_firstdatazone,
	 *		s->s_blocksize);
	 *
	 * and that gave us the size of the bitmap.
	 * This wil not work for our new BB free block map, which has
	 * many more levels of bitmaps. We must now calculate the sizes of
	 * all of the BB levels, as well as accounting for the 32-bit
	 * word-aligned padding for each level,
	 * THEN pass this total bitmap size in bits to fs421_blocks_needed()
	 * This is all done as a by-product of the calculations done in
	 * fs421_init_zmap_offsets()
	 */

	block = fs421_blocks_needed(fs421_init_zmap_offsets(sbi),
			s->s_blocksize);

	if (sbi->s_zmap_blocks < block) {
		printk("FS421-fs: file system does not have enough "
				"zmap blocks allocated.  Refusing to mount.\n");
		goto out_no_bitmap;
	}

	/* set up enough so that it can read an inode */
	s->s_op = &fs421_sops;
	root_inode = fs421_iget(s, FS421_ROOT_INO);
	if (IS_ERR(root_inode)) {
		ret = PTR_ERR(root_inode);
		goto out_no_root;
	}

	ret = -ENOMEM;
	s->s_root = d_make_root(root_inode);
	if (!s->s_root)
		goto out_no_root;

	if (!(s->s_flags & MS_RDONLY)) {
		ms->s_state &= ~FS421_VALID_FS;
		mark_buffer_dirty(bh);
	}
	if (!(sbi->s_mount_state & FS421_VALID_FS))
		printk("FS421-fs: mounting unchecked file system, "
			"running fsck is recommended\n");
	else if (sbi->s_mount_state & FS421_ERROR_FS)
		printk("FS421-fs: mounting file system with errors, "
			"running fsck is recommended\n");

	return 0;

out_no_root:
	if (!silent)
		printk("FS421-fs: get root inode failed\n");
	goto out_freemap;

out_no_bitmap:
	printk("FS421-fs: bad superblock or unable to read bitmaps\n");
out_freemap:
	for (i = 0; i < sbi->s_imap_blocks; i++)
		brelse(sbi->s_imap[i]);
	for (i = 0; i < sbi->s_zmap_blocks; i++)
		brelse(sbi->s_zmap[i]);
	kfree(sbi->s_imap);
	goto out_release;

out_no_map:
	ret = -ENOMEM;
	if (!silent)
		printk("FS421-fs: can't allocate map\n");
	goto out_release;

out_illegal_sb:
	if (!silent)
		printk("FS421-fs: bad superblock\n");
	goto out_release;

out_no_fs:
	if (!silent)
		printk("VFS: Can't find a Fs421 filesystem V1 | V2 | V3 "
		       "on device %s.\n", s->s_id);
out_release:
	brelse(bh);
	goto out;

out_bad_hblock:
	printk("FS421-fs: blocksize too small for device\n");
	goto out;

out_bad_sb:
	printk("FS421-fs: unable to read superblock\n");
out:
	s->s_fs_info = NULL;
	kfree(sbi);
	return ret;
}

static int fs421_statfs(struct dentry *dentry, struct kstatfs *buf)
{
	struct super_block *sb = dentry->d_sb;
	struct fs421_sb_info *sbi = fs421_sb(sb);
	u64 id = huge_encode_dev(sb->s_bdev->bd_dev);
	buf->f_type = sb->s_magic;
	buf->f_bsize = sb->s_blocksize;
	buf->f_blocks = (sbi->s_nzones - sbi->s_firstdatazone) << sbi->s_log_zone_size;
	buf->f_bfree = fs421_count_free_blocks(sb);
	buf->f_bavail = buf->f_bfree;
	buf->f_files = sbi->s_ninodes;
	buf->f_ffree = fs421_count_free_inodes(sb);
	buf->f_namelen = sbi->s_namelen;
	buf->f_fsid.val[0] = (u32)id;
	buf->f_fsid.val[1] = (u32)(id >> 32);

	return 0;
}

static int fs421_writepage(struct page *page, struct writeback_control *wbc)
{
	return block_write_full_page(page, fs421_get_block, wbc);
}

static int fs421_readpage(struct file *file, struct page *page)
{
	return block_read_full_page(page,fs421_get_block);
}

/*
 * The "chunk" here is from original MINIX, and does note refer to the
 * "chunks" concept from our binary buddy allocation scheme --jyp
 */
int fs421_prepare_chunk(struct page *page, loff_t pos, unsigned len)
{
	return __block_write_begin(page, pos, len, fs421_get_block);
}

static void fs421_write_failed(struct address_space *mapping, loff_t to)
{
	struct inode *inode = mapping->host;

	if (to > inode->i_size) {
		truncate_pagecache(inode, inode->i_size);
		fs421_truncate(inode);
	}
}

static int fs421_write_begin(struct file *file, struct address_space *mapping,
			loff_t pos, unsigned len, unsigned flags,
			struct page **pagep, void **fsdata)
{
	int ret;

	ret = block_write_begin(mapping, pos, len, flags, pagep,
				fs421_get_block);
	if (unlikely(ret))
		fs421_write_failed(mapping, pos + len);

	return ret;
}

static sector_t fs421_bmap(struct address_space *mapping, sector_t block)
{
	return generic_block_bmap(mapping,block,fs421_get_block);
}

static const struct address_space_operations fs421_aops = {
	.readpage = fs421_readpage,
	.writepage = fs421_writepage,
	.write_begin = fs421_write_begin,
	.write_end = generic_write_end,
	.bmap = fs421_bmap
};

static const struct inode_operations fs421_symlink_inode_operations = {
	.readlink	= generic_readlink,
	.follow_link	= page_follow_link_light,
	.put_link	= page_put_link,
	.getattr	= fs421_getattr,
};

void fs421_set_inode(struct inode *inode, dev_t rdev)
{
	if (S_ISREG(inode->i_mode)) {
		inode->i_op = &fs421_file_inode_operations;
		inode->i_fop = &fs421_file_operations;
		inode->i_mapping->a_ops = &fs421_aops;
	} else if (S_ISDIR(inode->i_mode)) {
		inode->i_op = &fs421_dir_inode_operations;
		inode->i_fop = &fs421_dir_operations;
		inode->i_mapping->a_ops = &fs421_aops;
	} else if (S_ISLNK(inode->i_mode)) {
		inode->i_op = &fs421_symlink_inode_operations;
		inode->i_mapping->a_ops = &fs421_aops;
	} else
		init_special_inode(inode, inode->i_mode, rdev);
}

/*
 * The global function to read an inode.
 * (Amalg. of fs421_iget() and V2_fs421_iget() --jyp
 */
struct inode *fs421_iget(struct super_block *sb, unsigned long ino)
{
	struct inode *inode;
	struct buffer_head * bh;
	struct fs421_inode * raw_inode;
	struct fs421_inode_info *fs421_inode;
	int i;

	inode = iget_locked(sb, ino);
	if (!inode)
		return ERR_PTR(-ENOMEM);
	if (!(inode->i_state & I_NEW))
		return inode;

	/* Remove V1|V2 cond --jyp */
	/* Remainder of function is from V2_fs421_iget() --jyp */
	fs421_inode = fs421_i(inode);
	raw_inode = fs421_raw_inode(inode->i_sb, inode->i_ino, &bh);
	if (!raw_inode) {
		iget_failed(inode);
		return ERR_PTR(-EIO);
	}
	inode->i_mode = raw_inode->i_mode;
	i_uid_write(inode, raw_inode->i_uid);
	i_gid_write(inode, raw_inode->i_gid);
	set_nlink(inode, raw_inode->i_nlinks);
	inode->i_size = raw_inode->i_size;
	inode->i_mtime.tv_sec = raw_inode->i_mtime;
	inode->i_atime.tv_sec = raw_inode->i_atime;
	inode->i_ctime.tv_sec = raw_inode->i_ctime;
	inode->i_mtime.tv_nsec = 0;
	inode->i_atime.tv_nsec = 0;
	inode->i_ctime.tv_nsec = 0;
	inode->i_blocks = 0;

	for (i = 0; i < 10; i++)
		fs421_inode->i2_data[i] = raw_inode->i_chunk[i];
	fs421_set_inode(inode, old_decode_dev(raw_inode->i_chunk[0]));

	brelse(bh);
	unlock_new_inode(inode);
	return inode;
}

static int fs421_write_inode(struct inode *inode, struct writeback_control *wbc)
{
	int err = 0;
	struct buffer_head *bh;
	struct fs421_inode * raw_inode;
	struct fs421_inode_info *fs421_inode = fs421_i(inode);
	int i;

	/* Remove V1|V2 cond --jyp */
	/* This part of function is from V2_fs421_iget() --jyp */
	raw_inode = fs421_raw_inode(inode->i_sb, inode->i_ino, &bh);
	if (!raw_inode)
		return EIO;
	raw_inode->i_mode = inode->i_mode;
	raw_inode->i_uid = fs_high2lowuid(i_uid_read(inode));
	raw_inode->i_gid = fs_high2lowgid(i_gid_read(inode));
	raw_inode->i_nlinks = inode->i_nlink;
	raw_inode->i_size = inode->i_size;
	raw_inode->i_mtime = inode->i_mtime.tv_sec;
	raw_inode->i_atime = inode->i_atime.tv_sec;
	raw_inode->i_ctime = inode->i_ctime.tv_sec;
	if (S_ISCHR(inode->i_mode) || S_ISBLK(inode->i_mode))
		raw_inode->i_chunk[0] = old_encode_dev(inode->i_rdev);

	else for (i = 0; i < 10; i++)
		raw_inode->i_chunk[i] = fs421_inode->i2_data[i];
	mark_buffer_dirty(bh);
	/* End of splice from V2_fs421_iget() */

	if (wbc->sync_mode == WB_SYNC_ALL && buffer_dirty(bh)) {
		sync_dirty_buffer(bh);
		if (buffer_req(bh) && !buffer_uptodate(bh)) {
			printk("IO error syncing fs421 inode [%s:%08lx]\n",
				inode->i_sb->s_id, inode->i_ino);
			err = -EIO;
		}
	}
	brelse (bh);
	return err;
}

int fs421_getattr(struct vfsmount *mnt, struct dentry *dentry, struct kstat *stat)
{
	struct super_block *sb = dentry->d_sb;
	generic_fillattr(dentry->d_inode, stat);
	/* Remove V1|V2 cond --jyp */
	stat->blocks = (sb->s_blocksize / 512) * fs421_blocks(stat->size, sb);
	stat->blksize = sb->s_blocksize;
	return 0;
}

static struct dentry *fs421_mount(struct file_system_type *fs_type,
	int flags, const char *dev_name, void *data)
{
	return mount_bdev(fs_type, flags, dev_name, data, fs421_fill_super);
}

static struct file_system_type fs421_fs_type = {
	.owner		= THIS_MODULE,
	.name		= "fs421",
	.mount		= fs421_mount,
	.kill_sb	= kill_block_super,
	.fs_flags	= FS_REQUIRES_DEV,
};
MODULE_ALIAS_FS("fs421");

static int __init init_fs421_fs(void)
{
	int err = init_inodecache();
	if (err)
		goto out1;
	err = register_filesystem(&fs421_fs_type);
	if (err)
		goto out;
	return 0;
out:
	destroy_inodecache();
out1:
	return err;
}

static void __exit exit_fs421_fs(void)
{
        unregister_filesystem(&fs421_fs_type);
	destroy_inodecache();
}

module_init(init_fs421_fs)
module_exit(exit_fs421_fs)
MODULE_LICENSE("GPL");

