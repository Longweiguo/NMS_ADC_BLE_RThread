#include "heyos_zlib.h"
#include <heyos.h>
#include <heyos_utils.h>
#include "zlib.h"
#include "heyos_fs.h"

#define CHUNK 4096


/*******************************************************************
 * add by lixueliang
 * heyos_zlib_decompress_data2data
 * declare: only used for OTA,because it is not safety
 * ****************************************************************/
int heyos_zlib_decompress_data2data(uint8_t * src, uint8_t * dest, uint32_t src_len)
{
    int ret;
    uint32_t have;
    z_stream strm;
    //uint8_t* in = (uint8_t *)heyos_malloc(CHUNK);
    //uint8_t* out = (uint8_t *)heyos_malloc(CHUNK);
    uint32_t read_size = 0;
    uint32_t write_size = 0;

    /* allocate inflate state */
    strm.zalloc = Z_NULL;
    strm.zfree = Z_NULL;
    strm.opaque = Z_NULL;
    strm.avail_in = 0;
    strm.next_in = Z_NULL;
	ret = inflateInit(&strm);
    if (ret != Z_OK)
        goto exit;

    /* decompress until deflate stream ends or end of file */
    do {
        uint32_t cur_size = ((src_len - read_size) > CHUNK)?(CHUNK):(src_len - read_size);
        strm.avail_in = cur_size;
        strm.next_in = &src[read_size];

        /* run inflate() on input until output buffer not full */
        do {
            strm.avail_out = CHUNK;
            strm.next_out = &dest[write_size];
            ret = inflate(&strm, Z_NO_FLUSH);

            switch (ret) {
            case Z_NEED_DICT:
                ret = Z_DATA_ERROR;     /* and fall through */
            case Z_DATA_ERROR:
            case Z_MEM_ERROR:
                (void)inflateEnd(&strm);
                goto exit;
            }
            have = CHUNK - strm.avail_out;
            write_size += have;
            /*if (fwrite(out, 1, have, dest) != have || ferror(dest)) {
                (void)inflateEnd(&strm);
                return Z_ERRNO;
            }*/
        } while (strm.avail_out == 0);

        read_size += cur_size;

        if(read_size >= src_len){
            break;
        }

        /* done when inflate() says it's done */
    } while (ret != Z_STREAM_END);



    /* clean up and return */
    (void)inflateEnd(&strm);

exit:
    //heyos_free(in);
    //heyos_free(out);
    return ret == Z_STREAM_END ? Z_OK : Z_DATA_ERROR;
}

/*******************************************************************
 * add by lixueliang
 * heyos_zlib_decompress_data2file
 *
 * ****************************************************************/
int heyos_zlib_decompress_data2file(uint8_t * src, char *des_file_name, uint32_t src_len)
{
    int ret;
    uint32_t have;
    z_stream strm;
    //uint8_t* in = (uint8_t *)heyos_malloc(CHUNK);
    uint8_t* out = (uint8_t *)heyos_malloc(CHUNK);
    uint32_t read_size = 0;
    FILE * dest = fopen(des_file_name, "w+");
    if (dest == NULL){
        goto exit;
    }

    /* allocate inflate state */
    strm.zalloc = Z_NULL;
    strm.zfree = Z_NULL;
    strm.opaque = Z_NULL;
    strm.avail_in = 0;
    strm.next_in = Z_NULL;
	ret = inflateInit(&strm);
    if (ret != Z_OK)
        goto exit;

    /* decompress until deflate stream ends or end of file */
    do {
        uint32_t cur_size = ((src_len - read_size) > CHUNK)?(CHUNK):(src_len - read_size);
        strm.avail_in = cur_size;
        strm.next_in = &src[read_size];

        /* run inflate() on input until output buffer not full */
        do {
            strm.avail_out = CHUNK;
            strm.next_out = out;
            ret = inflate(&strm, Z_NO_FLUSH);

            switch (ret) {
            case Z_NEED_DICT:
                ret = Z_DATA_ERROR;     /* and fall through */
            case Z_DATA_ERROR:
            case Z_MEM_ERROR:
                (void)inflateEnd(&strm);
                goto exit;
            }
            have = CHUNK - strm.avail_out;

            if (fwrite(out, 1, have, dest) != have || ferror(dest)) {
                (void)inflateEnd(&strm);
                goto exit;
            }
        } while (strm.avail_out == 0);

        read_size += cur_size;

        if(read_size >= src_len){
            break;
        }

        /* done when inflate() says it's done */
    } while (ret != Z_STREAM_END);



    /* clean up and return */
    (void)inflateEnd(&strm);

exit:
    //heyos_free(in);
    heyos_free(out);
    fclose(dest);
    return ret == Z_STREAM_END ? Z_OK : Z_DATA_ERROR;
}

int heyos_zlib_decompress_file2file(char *src_file_name, char *des_file_name)
{
    int ret;
    uint32_t have;
    z_stream strm;
    uint8_t* in = (uint8_t *)heyos_malloc(CHUNK);
    uint8_t* out = (uint8_t *)heyos_malloc(CHUNK);
    uint32_t read_size = 0;
    uint32_t f_size = 0;
    FILE * f_src = NULL;
    FILE * f_des = fopen(des_file_name, "w+");
    if (f_des == NULL){
        goto exit;
    }

    f_src = fopen(src_file_name, "r+");
    if (f_src == NULL){
        goto exit;
    }
    f_size = heyos_file_size(src_file_name);
    /* allocate inflate state */
    strm.zalloc = Z_NULL;
    strm.zfree = Z_NULL;
    strm.opaque = Z_NULL;
    strm.avail_in = 0;
    strm.next_in = Z_NULL;
	ret = inflateInit(&strm);
    if (ret != Z_OK)
        goto exit;

    /* decompress until deflate stream ends or end of file */
    do {
        uint32_t cur_size = ((f_size - read_size) > CHUNK)?(CHUNK):(f_size - read_size);
        cur_size = fread(in, 1, cur_size,  f_src);

        strm.avail_in = cur_size;
        strm.next_in = in;

        /* run inflate() on input until output buffer not full */
        do {
            strm.avail_out = CHUNK;
            strm.next_out = out;
            ret = inflate(&strm, Z_NO_FLUSH);

            switch (ret) {
            case Z_NEED_DICT:
                ret = Z_DATA_ERROR;     /* and fall through */
            case Z_DATA_ERROR:
            case Z_MEM_ERROR:
                (void)inflateEnd(&strm);
                goto exit;
            }
            have = CHUNK - strm.avail_out;

            if (fwrite(out, 1, have, f_des) != have || ferror(f_des)) {
                (void)inflateEnd(&strm);
                goto exit;
            }
        } while (strm.avail_out == 0);

        read_size += cur_size;

        if(read_size >= f_size){
            break;
        }

        /* done when inflate() says it's done */
    } while (ret != Z_STREAM_END);



    /* clean up and return */
    (void)inflateEnd(&strm);

exit:
    heyos_free(in);
    heyos_free(out);
    fclose(f_des);
    fclose(f_src);
    return ret == Z_STREAM_END ? Z_OK : Z_DATA_ERROR;
}

bool is_gzipfile(char* des)
{
    int gzip_fd = -1;
    bool ret = false;
    gzip_fd = open(des, O_RDONLY, 0);
    if(gzip_fd < 0) {
        LOG_ERROR("%s is not tar file\n",des);
        return false;
    }

    char* buff = heyos_psram_malloc(128);
    if(buff == NULL) {
        LOG_ERROR("[%s] no mem\n",__FUNCTION__);
        close(gzip_fd);
        return false;
    }

    read(gzip_fd, buff, 10);
    // check ID1 ID2
    if(buff[0] == 0x1F && buff[1] == 0x8B) {
        ret = true;
    }

    heyos_free(buff);
    close(gzip_fd);
    return ret;
}

static void is_gzip(int argc, char** argv)
{
    if(is_gzipfile(argv[1])) {
        LOG_DEBUG("%s is gzipfile\n",argv[1]);
    } else {
        LOG_DEBUG("%s is no gzipfile\n",argv[1]);
    }
}
MSH_CMD_EXPORT(is_gzip, check file is gzip);

void ungzip(int argc, char** argv)
{
    char out_name[10] = {0};
    if(argc < 1) {
        return;
    }
    if(argc == 1) {
        int len = strlen(argv[1]);
        strncpy(out_name,argv[1], len -3);
    } else {
        strcpy(out_name, argv[2]);
    }
    heyos_zlib_decompress_file2file(argv[1], out_name);
}
MSH_CMD_EXPORT(ungzip, unpack gzip file);


