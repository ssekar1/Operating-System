/*
 *  linux/fs/fs421/bitmap.c
 *
 *  Copyright (C) 1991, 1992  Linus Torvalds
 */

/*
 * Modified for 680x0 by Hamish Macdonald
 * Fixed for 680x0 by Andreas Schwab
 *
 *  Modified from minix fs for use as exercise for UMBC CMSC421 OS course
 *      by John Y. Park
 */

/* bitmap.c contains the code that handles the inode and block bitmaps */

/*
 * Additional notes:
 * The inode bitmap starts from 0, i.e., inode x is represented by bit x.
 * However, the actual inode array starts from index 1, i.e., inode 1 is
 * inodes[0] in the first block of inodes.
 * Therefore, bit 0 in the inode bitmap is always set, because it doesn't
 * correspond to any actual inode structure.  --jyp
 */


#include "fs421.h"
#include <linux/buffer_head.h>
#include <linux/bitops.h>
#include <linux/sched.h>

static DEFINE_SPINLOCK(bitmap_lock);

/*
 * General comments:
 *   Bitmap consists of blocks filled with 32bit words (you can walk it as
 *   8-bit or 16-bit "words" if desired, but note that FS421 pads to 32-bit
 *   boundaries).  Note: bit set == busy, bit clear == free
 */

/*

additional function bit finder level offset, level, offset map, and bitmap

*/

 
/*
 * HINT-STUB:
 *
 * fs421_init_zmap_offsets():  NEW FUNCTION
 * Supplemental code to make dealing with bitmaps faster, by calculating
 * the size of each level of the Binary Buddy bitmap, and using that to
 * generate a precomputed cache of the offsets for each level of the
 * bitmaps, accounting for the 32-bit word-aligned padding for each level.
 * Note that this generates one extra "offset" value, i.e., where the
 * "n+1-th level" would start--this lets us calculate the size of the
 * nth level as a general case.
 * This table of offsets is used to conveniently go directly to any
 * desired level of the bitmap.
 * NB: the table is in units of bits.
 * Return: total number of bits across all the levels of BB bitmap hierarchy--
 *   used by fs421_fill_super() at mount time to validate that the
 *   bitmap size indicated in the superblock is of sufficient size.
 */

 unsigned B_Buddy( int level, __u32 *offset, struct buffer_head **adjust_block, int tot_bits){

		//block number as a linear ordering
	int B_BlockNum = offset[level];
	int endB_BlockNum = offset[level+1];
	int bits_left = endB_BlockNum - B_BlockNum;
	int D_BlockNum = B_BlockNum / tot_bits;
	int Begin_block = B_BlockNum % tot_bits;
	int bitsChecked = 0;
	unsigned words;
	//adjusted in block size
	adjust_block = adjust_block + D_BlockNum;

	//find buddy
	int found = 0;

	do {
		
	  int bits_to_test = tot_bits - Begin_block;
		
		if (bits_to_test > bits_left) {
			bits_to_test = bits_left;
		}
	//bits test /32
		words = bits_to_test / 32;

		// next block , finding lock
	unsigned   p = (u32 *)(*adjust_block++)->b_data + Begin_block / 32;

		
		while (words--){
			int firstZeroBit = 32 - ffz(p++);
			if (firstZeroBit){
				found = 1;
				bitsChecked += firstZeroBit;
			}
			else{
				bitsChecked += 32;
			}
		}
		
		bits_left -= bits_to_test;
		Begin_block = 0;

		
	} while (bits_left);

	int returnValue;

	if (found){
		returnValue = bitsChecked;
	}
	else{
	
		returnValue = 0;
	}
	return returnValue;
}

/*
 * General comments:
 *   Bitmap consists of blocks filled with 32bit words (you can walk it as
 *   8-bit or 16-bit "words" if desired, but note that FS421 pads to 32-bit
 *   boundaries).  Note: bit set == busy, bit clear == free
 */

 
 
/*
  Additional operation log2 operation. 
 */
				
 int find(int blockNum)
{
	int returnValue;

	if (blockNum % 2 == 0){
		returnValue = blockNum + 1;
	}
	else{
		returnValue = blockNum - 1;
	}
	// return value. set block num
	return returnValue;
}



/*
 * General comments:
 *   Bitmap consists of blocks filled with 32bit words (you can walk it as
 *   8-bit or 16-bit "words" if desired, but note that FS421 pads to 32-bit
 *   boundaries).  Note: bit set == busy, bit clear == free
 */

 
/*
 * HINT-STUB:
 *
 * fs421_init_zmap_offsets():  NEW FUNCTION
 * Supplemental code to make dealing with bitmaps faster, by calculating
 * the size of each level of the Binary Buddy bitmap, and using that to
 * generate a precomputed cache of the offsets for each level of the
 * bitmaps, accounting for the 32-bit word-aligned padding for each level.
 * Note that this generates one extra "offset" value, i.e., where the
 * "n+1-th level" would start--this lets us calculate the size of the
 * nth level as a general case.
 * This table of offsets is used to conveniently go directly to any
 * desired level of the bitmap.
 * NB: the table is in units of bits.
 * Return: total number of bits across all the levels of BB bitmap hierarchy--
 *   used by fs421_fill_super() at mount time to validate that the
 *   bitmap size indicated in the superblock is of sufficient size.
 */
unsigned long fs421_init_zmap_offsets(struct fs421_sb_info *sbi) {
	__u32 bits = sbi->s_nzones - sbi->s_firstdatazone;
	__u32 total_bits = 0;
	int level = 0;

	sbi->s_zmap_offset[0] = 0;
	while (bits) {
		total_bits += DIV_ROUND_UP(bits, 32) * 32;
		sbi->s_zmap_offset[++level] = total_bits;
		bits /= 2;
	}
	sbi->s_zmap_nlevels = level;
	return total_bits;
}


/*
 * General comments:
 *   Bitmap consists of blocks filled with 32bit words (you can walk it as
 *   8-bit or 16-bit "words" if desired, but note that FS421 pads to 32-bit
 *   boundaries).  Note: bit set == busy, bit clear == free
 */

 
/*
  Additional operation log2 operation. 
 */
				
 // int find(int blockNum)
// {
	// int returnValue;
	// int findValue;
	// if (blockNum % 5 == 0){
		// findValue = blockNum + 1;
	// }
	// else{
		// returnValue = blockNum - 1;

	// if (blockNum % 3 == 1){
		// returnValue = blockNum + 1;
	// }
	// else{
		// returnValue = blockNum - 1;
	// }
	// // return value. set block num
	// return returnValue;
// }




/*

additional function clear level offset, level, offset map, and bitmap

*/


/*
 * General comments:
 *   Bitmap consists of blocks filled with 32bit words (you can walk it as
 *   8-bit or 16-bit "words" if desired, but note that FS421 pads to 32-bit
 *   boundaries).  Note: bit set == busy, bit clear == free
 */
 void clear_Offsets(int Block_lvel_offset, int level, __u32 *offset, struct buffer_head **adjust_block, int tot_bits){

	
	//clear block number 
	int B_BlockNum = offset[level] + Block_lvel_offset;
	int D_BlockNum = B_BlockNum / tot_bits;
	int Begin_block = B_BlockNum % tot_bits;
	int W_block = Begin_block / 32;
	int End_block = Begin_block % 32;

	//adjust in block size
	adjust_block = adjust_block + D_BlockNum;

	//size 32 pointer for test bit
	__u32 *p = (__u32 *)(*adjust_block)->b_data + W_block;

	clear_bit_le(End_block, p);
	mark_buffer_dirty(*adjust_block);

}


/*
 * General comments:
 *   Bitmap consists of blocks filled with 32bit words (you can walk it as
 *   8-bit or 16-bit "words" if desired, but note that FS421 pads to 32-bit
 *   boundaries).  Note: bit set == busy, bit clear == free
 */

/*
 * HINT-STUB:
 *
 * fs421_init_zmap_offsets():  NEW FUNCTION
 * Supplemental code to make dealing with bitmaps faster, by calculating
 * the size of each level of the Binary Buddy bitmap, and using that to
 * generate a precomputed cache of the offsets for each level of the
 * bitmaps, accounting for the 32-bit word-aligned padding for each level.
 * Note that this generates one extra "offset" value, i.e., where the
 * "n+1-th level" would start--this lets us calculate the size of the
 * nth level as a general case.
 * This table of offsets is used to conveniently go directly to any
 * desired level of the bitmap.
 * NB: the table is in units of bits.
 * Return: total number of bits across all the levels of BB bitmap hierarchy--
 *   used by fs421_fill_super() at mount time to validate that the
 *   bitmap size indicated in the superblock is of sufficient size.
 */

unsigned long fs421_init_zmap_offsets(struct fs421_sb_info *sbi) {
	__u32 bits = sbi->s_nzones - sbi->s_firstdatazone;
	__u32 total_bits = 0;
	int level = 0;

	sbi->s_zmap_offset[0] = 0;
	while (bits) {
		total_bits += DIV_ROUND_UP(bits, 32) * 32;
		sbi->s_zmap_offset[++level] = total_bits;
		bits /= 2;
	}
	sbi->s_zmap_nlevels = level;
	return total_bits;
}

/*
 * General comments:
 *   Bitmap consists of blocks filled with 32bit words (you can walk it as
 *   8-bit or 16-bit "words" if desired, but note that FS421 pads to 32-bit
 *   boundaries).  Note: bit set == busy, bit clear == free
 */
/*
  additional function test level offset, level, offset map, and bitmap
*/

 int test_Offsets(int Block_lvel_offset, int level, __u32 *offset, struct buffer_head **adjust_block, int tot_bits){

	// asking for blocks
	// sorting block num
	int B_BlockNum = offset[level] + Block_lvel_offset;  
	// data num
	int D_BlockNum = B_BlockNum / tot_bits;
	
	int Begin_block = B_BlockNum % tot_bits;
	int W_block = Begin_block / 32;
	int End_block = Begin_block % 32;

	//adjst block size
	adjust_block = adjust_block + D_BlockNum;

	//size 32 pointer for test bit
	__u32 *p = (__u32 *)(*adjust_block)->b_data + W_block;

	int returnValue = test_bit_le(End_block, p);

	return returnValue;
}


/*
 * get_bh_and_bit():  NEW FUNCTION
 */
static inline void get_bh_and_bit(struct super_block *sb, int level, __u32 zchunk,
			     struct buffer_head **bh, __u32 *bit) {
	struct fs421_sb_info *sbi = fs421_sb(sb);
	int k = sb->s_blocksize_bits + 3;

	/* Calculate the absolute position of the bit we are
	 * interested in, from start of entire bitmap set */
	__u32 abs_bit = sbi->s_zmap_offset[level] + (zchunk >> level);
	*bh = sbi->s_zmap[abs_bit >> k];
	*bit = abs_bit & ((1<<k) - 1);
}

/*
 * Tallies the number of free (zero) bits at one level of the BB bitmap set.
 * Param "start" is the bit offset in bitmap hierarchy for requested level
 *
 * HINT-STUB:
 * You should study this function very carefully--it provides a wealth
 * of information that you can reuse in other bitmap functions,
 * particularly how you span multiple blocks, while dealing with the
 * Linux buffer cache.
 */
static __u32 count_free(struct buffer_head *map[], unsigned blocksize,
			__u32 start, __u32 num_bits)
{
  struct buffer_head **adjust_block;
  __u32 tot_bits = 8 * blocksize;
  __u32 bits_left = num_bits;
  unsigned start_in_blk, bits_to_test;
	unsigned words;
	__u32 sum = 0;
	__u16 *p;
	
	/* Should following also test if num_bits % 32? */
	if (start % 32) {
	  printk("Non-word-aligned bitmap start or len: %u, %u\n",
		 start, num_bits);
	  return 0;
	}
	
	adjust_block = map + start / tot_bits;
	start_in_blk = start % tot_bits;
	
	do {
	  bits_to_test = tot_bits - start_in_blk;
	  if (bits_to_test > bits_left) {
	    bits_to_test = bits_left;
	  }
	  /* Now, convert to words */
	  words = bits_to_test / 16;
	  p = (__u16 *)(*adjust_block++)->b_data + start_in_blk / 16;
	  while (words--)
	    sum += 16 - hweight16(*p++);
	  bits_left -= bits_to_test;
	  start_in_blk = 0;	/* for all subsequent blocks */
	} while (bits_left);
	
	return sum;
}


/*
 * General comments:
 *   Bitmap consists of blocks filled with 32bit words (you can walk it as
 *   8-bit or 16-bit "words" if desired, but note that FS421 pads to 32-bit
 *   boundaries).  Note: bit set == busy, bit clear == free
 */

/*
additional function set level offset, level, offset map, and bitmap
*/

void set_Offsets(int Block_lvel_offset, int level, __u32 *offset, struct buffer_head **adjust_block, int tot_bits){

	
	//initialize block number 
	int B_BlockNum = offset[level] + Block_lvel_offset;
	int D_BlockNum = B_BlockNum / tot_bits;
	int Begin_block = B_BlockNum % tot_bits;
	int W_block = Begin_block / 32;
	int End_block = Begin_block % 32;

	//adjust in block size
	adjust_block = adjust_block + D_BlockNum;

	//size 32 pointer for test bit
	__u32 *p = (__u32 *)(*adjust_block)->b_data + W_block;

	set_bit_le(End_block, p);
	mark_buffer_dirty(*adjust_block);

}


/*
 * HINT-STUB:
 * This is a required function, which you must write most of.
 *
 * It receives a request to free up a chunk of the specified size
 * (units is log2(num_blocks_to_free)) starting at the specified disk
 * block number, and puts it back into the binary buddy bitmap.
 * Note it must handle mapping disk block# to bitmap level and position,
 * and then also handle coalescing.
 *
 * IMPORTANT: be very consistent in the units you are passing around: is it
 * bits, bytes, words, or blocks??
 */
void fs421_free_block(struct inode *inode, unsigned long block, unsigned size_log2)
{
  struct super_block *sb = inode->i_sb;
  struct fs421_sb_info *sbi = fs421_sb(sb);
  struct buffer_head *adjust_block;
  __u32 zchunk, bit, bbit;
  int level;

  unsigned blocksize = sb->s_blocksize;
  __u32 *offsetMap = sbi->s_zmap_offset;  
  int tot_bits = 8*blocksize;

  unsigned long Block_Bud_Bin = block - sbi->s_firstdatazone;
  unsigned long numBlocks = sbi->s_nzones; 
  
    

  int Block_lvel_offset = Block_Bud_Bin / (2 << (size_log2 - 1));
  
  //CHECK THAT BLOCK IS IN LEGAL RANGE FOR DISK;
  if(block >= sbi->s_firstdatazone && block < numBlocks) {
    
    //CHECK THAT BLOCK IS PROPERLY ALIGNED FOR ITS BINARY SIZE;
    if(ffs(Block_Bud_Bin)- 1 >= size_log2){
      
      spin_lock(&bitmap_lock);  /* LEAVE THIS ALONE! */
      
      
      //CHECK THAT BIT ISN NOT ALREADY FREE AT LEVEL;
      if (!test_Offsets(Block_lvel_offset, size_log2, offsetMap, adjust_block, tot_bits)){
	
		
	//FREE UP BIT AT LEVEL; COALESCE IF NECESSARY;
	clear_Offsets(Block_lvel_offset, size_log2, offsetMap, adjust_block, tot_bits);
	
	int buddyBlock_lvel_offset = find(Block_lvel_offset);
	
	//CHECK BUDDY FOR COALESCING;
	while (!test_Offsets(buddyBlock_lvel_offset, size_log2, offsetMap, adjust_block, tot_bits)){
	    
	  
	  set_Offsets(Block_lvel_offset, size_log2, offsetMap, adjust_block, tot_bits);
	  set_Offsets(buddyBlock_lvel_offset, size_log2, offsetMap, adjust_block, tot_bits);
	  	  
	  size_log2++;
	  Block_lvel_offset /= 2;
	  
	  buddyBlock_lvel_offset = find(Block_lvel_offset);
	  	  
	  clear_Offsets(Block_lvel_offset, size_log2, offsetMap, adjust_block, tot_bits);
	}
      }
      
      spin_unlock(&bitmap_lock);  /* LEAVE THIS ALONE! */
      
    }
     
}

/*
 * HINT-STUB:
 * This is a required function, which you must write most of.
 *
 * It implements the allocation side of the binary buddy allocator.
 * It receives a request for a chunk of a specified size
 * (units is log2(num_blocks_to_free)), and tries to allocate it from
 * the binary buddy bitmap.
 * Note it must handle mapping bitmap level and position to  disk block#.
 *
 * Return: disk-relative block# of first disk block of contiguous chunk
 *   of size 2**size_log2,
 *   0 on error (e.g., no more space big enough)
 *
 * IMPORTANT: be very consistent in the units you are working with at any
 * given time: is it bits, bytes, words, or blocks??
 */
int fs421_new_block(struct inode * inode, unsigned size_log2)
{
	
  struct super_block *sb = inode->i_sb;
  struct fs421_sb_info *sbi = fs421_sb(sb);
  struct buffer_head *bh;
  int Bit_first;
  int level = size_log2;
  int maxLevel = sbi->s_zmap_nlevels;
  // int level;
  __u32 bit, zchunk;
  
  
  /*
  CALCULATE LEVEL BASED ON SIZE: STORE IN size_log2;

	SCAN REQUESETD LEVEL;

	IF NECESSARY: SCAN HIGHER LEVELS {
	    IF FOUND AT HIGHER LEVEL, SPLIT INTO PIECES
		BACK DOWN TO REQUESTED LEVEL;
	}

	MARK BIT AS USED (BIT SET);

	IMPORTANT--EVERY TIME YOU CHANGE A BIT, MUST USE FOLLOWING
	    ON CONTAINING BUFFER''S buffer_head:
	    mark_buffer_dirty(bh);


	spin_unlock(&bitmap_lock);

	RETURN DISK-RELATIVE OFFSET OF CHUNK, OR 0 FOR ERROR;
  
  */
    
  struct buffer_head **s_imap = sbi->s_imap;
  struct buffer_head **adjust_block = sbi->s_zmap;
  unsigned blocksize = sb->s_blocksize;
  int tot_bits = 8 * blocksize;
  __u32 *offsetMap = sbi->s_zmap_offset;
   unsigned long numBlocks = sbi->s_nzones;
  int FFB_bit = 0;//free bit
  spin_lock(&bitmap_lock);

  
  //SCAN REQUESTED LEVEL;
  //IF NECESSARY: SCAN HIGHER LEVELS 
  /*
  
  
	IF NECESSARY: SCAN HIGHER LEVELS {
	    IF FOUND AT HIGHER LEVEL, SPLIT INTO PIECES
		BACK DOWN TO REQUESTED LEVEL;
	}
	*/
  
  do{
    //IF NECESSARY: SCAN HIGHER LEVELS
     Bit_first = B_Buddy(level, offsetMap, adjust_block, tot_bits);
    //IF FOUND AT HIGHER LEVEL, SPLIT INTO PIECES
	//	BACK DOWN TO REQUESTED LEVEL;
  
    if (Bit_first ){
      FFB_bit = 1;
 
      if (level == size_log2){
	
	/*
	MARK BIT AS USED (BIT SET);

	IMPORTANT--EVERY TIME YOU CHANGE A BIT, MUST USE FOLLOWING
	    ON CONTAINING BUFFER''S buffer_head:
	    mark_buffer_dirty(bh);
	
	*/
	set_Offsets(Bit_first, level, offsetMap, adjust_block, tot_bits);
	break;
      }
      
      else{

	 //RETURN DISK-RELATIVE OFFSET OF CHUNK, OR 0 FOR ERROR;
	while (level != size_log2){
  
	  set_Offsets(Bit_first, level, offsetMap, adjust_block, tot_bits);
	  

	  clear_Offsets(0, level - 1, offsetMap, adjust_block, tot_bits);
	  clear_Offsets(1, level - 1, offsetMap, adjust_block, tot_bits);
	  

	  Bit_first = 0;
	  level--;
	}

	set_Offsets(Bit_first, level, offsetMap, adjust_block, tot_bits);
	break;
      }
      
    }
    level++;
    
    
  } while (level != sbi->s_zmap_nlevels);
  
  spin_unlock(&bitmap_lock);
  
  int returnValue;
  //RETURN DISK-RELATIVE OFFSET OF CHUNK, OR 0 FOR ERROR;
 
  if (FFB_bit){
    
    int diskRelativeOffset = offsetMap[level] + Bit_first + sbi->s_firstdatazone;
      returnValue = diskRelativeOffset;
  }
  else{
   
    returnValue = 0;
  }
 
 }


unsigned long fs421_count_free_blocks(struct super_block *sb)
{
  __u32 sum = 0;
  struct fs421_sb_info *sbi = fs421_sb(sb);
  __u32 *offset = sbi->s_zmap_offset;
  int i;
  
  
  for (i = 0; i < sbi->s_zmap_nlevels; i++) {
    /* count_free() uses a bitcount for use on inode bitmap,
     * so keep it that way
     */
    sum += count_free(sbi->s_zmap, sb->s_blocksize,
		      offset[i], offset[i + 1] - offset[i])
      << i;
  }
  
  return (sum << sbi->s_log_zone_size);
}

/*
 * Was originally "struct minix2_inode minix_V2_raw_inode()" --jyp
 */

struct fs421_inode *
fs421_raw_inode(struct super_block *sb, ino_t ino, struct buffer_head **bh)
{
  int block;
  struct fs421_sb_info *sbi = fs421_sb(sb);
  struct fs421_inode *p;
  int fs421_inodes_per_block = sb->s_blocksize / sizeof(struct fs421_inode);
  
  *bh = NULL;
  if (!ino || ino > sbi->s_ninodes) {
    printk("Bad inode number on dev %s: %ld is out of range\n",
	   sb->s_id, (long)ino);
    return NULL;
  }
  ino--;
  block = 2 + sbi->s_imap_blocks + sbi->s_zmap_blocks +
    ino / fs421_inodes_per_block;
  *bh = sb_bread(sb, block);
  if (!*bh) {
    printk("Unable to read inode block\n");
    return NULL;
  }
  p = (void *)(*bh)->b_data;
  return p + ino % fs421_inodes_per_block;
}

/* Clear the link count and mode of a deleted inode on disk. */

static void fs421_clear_inode(struct inode *inode)
{
  struct buffer_head *bh = NULL;
  
  /* Following was formerly V1|V2-specific */
  {
    struct fs421_inode *raw_inode;
    raw_inode = fs421_raw_inode(inode->i_sb, inode->i_ino, &bh);
    if (raw_inode) {
      raw_inode->i_nlinks = 0;
      raw_inode->i_mode = 0;
    }
  }
  if (bh) {
    mark_buffer_dirty(bh);
    brelse (bh);
  }
}

void fs421_free_inode(struct inode * inode)
{
  struct super_block *sb = inode->i_sb;
  struct fs421_sb_info *sbi = fs421_sb(inode->i_sb);
  struct buffer_head *bh;
  int k = sb->s_blocksize_bits + 3;
  unsigned long ino, bit;
  
  ino = inode->i_ino;
  if (ino < 1 || ino > sbi->s_ninodes) {
    printk("fs421_free_inode: inode 0 or nonexistent inode\n");
    return;
  }
  bit = ino & ((1<<k) - 1);
  ino >>= k;
  if (ino >= sbi->s_imap_blocks) {
    printk("fs421_free_inode: nonexistent imap in superblock\n");
    return;
  }
  
  fs421_clear_inode(inode);	/* clear on-disk copy */
  
  bh = sbi->s_imap[ino];
  spin_lock(&bitmap_lock);
  if (!__test_and_clear_bit_le(bit, bh->b_data))
    printk("fs421_free_inode: bit %lu already cleared\n", bit);
  spin_unlock(&bitmap_lock);
  mark_buffer_dirty(bh);
}

struct inode *fs421_new_inode(const struct inode *dir, umode_t mode, int *error)
{
  struct super_block *sb = dir->i_sb;
  struct fs421_sb_info *sbi = fs421_sb(sb);
  struct inode *inode = new_inode(sb);
  struct buffer_head * bh;
  int tot_bits = 8 * sb->s_blocksize;
  unsigned long j;
  int i;
  
  if (!inode) {
    *error = -ENOMEM;
    return NULL;
  }
  j = tot_bits;
  bh = NULL;
  *error = -ENOSPC;
  spin_lock(&bitmap_lock);
  for (i = 0; i < sbi->s_imap_blocks; i++) {
    bh = sbi->s_imap[i];
    j = find_first_zero_bit_le(bh->b_data, tot_bits);
    if (j < tot_bits)
      break;
  }
  if (!bh || j >= tot_bits) {
    spin_unlock(&bitmap_lock);
    iput(inode);
    return NULL;
  }
  if (__test_and_set_bit_le(j, bh->b_data)) {	/* shouldn't happen */
    spin_unlock(&bitmap_lock);
    printk("fs421_new_inode: bit already set\n");
    iput(inode);
    return NULL;
  }
  spin_unlock(&bitmap_lock);
  mark_buffer_dirty(bh);
  j += i * tot_bits;
  if (!j || j > sbi->s_ninodes) {
    iput(inode);
    return NULL;
  }
  inode_init_owner(inode, dir, mode);
  inode->i_ino = j;
  inode->i_mtime = inode->i_atime = inode->i_ctime = CURRENT_TIME_SEC;
  inode->i_blocks = 0;
  memset(&fs421_i(inode)->i2_data, 0, sizeof(fs421_i(inode)->i2_data));
  insert_inode_hash(inode);
  mark_inode_dirty(inode);
  
  *error = 0;
  return inode;
}

unsigned long fs421_count_free_inodes(struct super_block *sb)
{
  struct fs421_sb_info *sbi = fs421_sb(sb);
  u32 bits = sbi->s_ninodes + 1;
  
  return count_free(sbi->s_imap, sb->s_blocksize, 0, bits);
}
