/*
 *  linux/fs/fs421/file.c
 *
 *  Copyright (C) 1991, 1992 Linus Torvalds
 *
 *  Modified from minix fs for use as exercise for UMBC CMSC421 OS course
 *      by John Y. Park
 */

/*
 *
 *  fs421 regular file handling primitives
 */

#include "fs421.h"

/*
 * We have mostly NULLs here: the current defaults are OK for
 * the fs421 filesystem.
 */
const struct file_operations fs421_file_operations = {
	.llseek		= generic_file_llseek,
	.read		= do_sync_read,
	.aio_read	= generic_file_aio_read,
	.write		= do_sync_write,
	.aio_write	= generic_file_aio_write,
	.mmap		= generic_file_mmap,
	.fsync		= generic_file_fsync,
	.splice_read	= generic_file_splice_read,
#ifdef CANONICAL
	.release	= fs421_release,
#endif
};

static int fs421_setattr(struct dentry *dentry, struct iattr *attr)
{
	struct inode *inode = dentry->d_inode;
	int error;

	error = inode_change_ok(inode, attr);
	if (error)
		return error;

	if ((attr->ia_valid & ATTR_SIZE) &&
	    attr->ia_size != i_size_read(inode)) {
		error = inode_newsize_ok(inode, attr->ia_size);
		if (error)
			return error;

		truncate_setsize(inode, attr->ia_size);
		fs421_truncate(inode);
	}

	setattr_copy(inode, attr);
	mark_inode_dirty(inode);
	return 0;
}

const struct inode_operations fs421_file_inode_operations = {
	.setattr	= fs421_setattr,
	.getattr	= fs421_getattr,
};
