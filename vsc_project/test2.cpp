#include <vector>
#include <stack>
#include <iostream>
#include <queue>
#include <map>
#include <functional>
#include <set>
#include <stdio.h>


using p64=std::pair<int,int>;
using MatrixInt=std::vector<std::vector<int>>;

template <typename T>
using Matrix=std::vector<std::vector<T>>;

bool is_valid_pixel(int x,int y,int h,int w)
{
    return (x>=0&&x<h&&y>=0&&y<w);
}

typedef struct 
{
    uint16_t Num;
    int16_t M;
    int16_t N;
}single_point_struct;

typedef struct 
{
    int point_count;
    single_point_struct* point;
}point_struct;

typedef struct 
{
    single_point_struct point;
    int16_t Jump;
    int16_t Jump1;
    bool IS_Door;
}single_link_struct;


typedef struct 
{
    uint16_t id;
    uint16_t num;
}orphan_struct;


typedef struct 
{
    int16_t first;
    int16_t second;
}p16_struct;

typedef struct 
{
    int16_t size;
    p16_struct* p16;
}p16_array;

typedef struct 
{
    uint16_t id;

    int isoland;
    int cleaned;
    int inside;

    int16_t max_x;
    int16_t max_y;
    int16_t min_x;
    int16_t min_y;

    int32_t square;

    int16_t link_count;
    single_link_struct* link;

    int8_t obstacle_count;
    p16_array* obstacle;

    int8_t near_count;
    uint8_t* near;

    int8_t door_count;
    uint8_t* door;

    int8_t orphan_count;
    orphan_struct* orphan;
}link_struct;


typedef struct 
{
    bool Parent;
    uint16_t link_id;
}indicate_struct;


typedef struct
{
    uint16_t id;
    int16_t M;
    int16_t N;
    uint16_t relative_count;
    uint16_t parent_count;
    indicate_struct* relatives;
}orphans_struct;


typedef struct 
{
    uint8_t area_count;
    int16_t convert_x;
    int16_t convert_y;
    float convert_theta;
    link_struct* area_links;

    uint8_t orphan_count;
    orphans_struct* orphan_links;
}area_struct;




typedef struct am_data_t
{
    /* data */
    char name[128];
    FILE *fp;
    pthread_mutex_t mutex;
}am_data;
typedef am_data* am_data_ptr;

void* malloc_space(uint32_t n,const char* s)
{
    void* dest=NULL;
    if(n>0)
    {
        dest=malloc(n);
        if(dest!=NULL)
        {
            memset(dest,0,n);
        }
    }
    return dest;
}

uint32_t hal_data_read(am_data_ptr dp,uint32_t offset,uint32_t len,uint8_t* buff)
{
    uint32_t r_len;
    int ret;
    if(NULL==dp)
    {
        return -1;
    }

    pthread_mutex_lock(&dp->mutex);

    ret=fseek(dp->fp,offset,SEEK_SET);
    if(ret<0)
    {
        pthread_mutex_unlock(&dp->mutex);
        return ret;
    }

    r_len=fread(buff,1,len,dp->fp);

    fflush(dp->fp);
    pthread_mutex_unlock(&dp->mutex);
    return r_len;
}

//从完整路径中返回文件名
const char *get_rpfile_name(const char *apfile_name)
{
    int ch='/';
    char *p2;

    char *p1=strrchr((char*)apfile_name,ch);
    if(p1==NULL)
    {
        return apfile_name;
    }
    p2=p1+1;
    return p2;
}

char *strsep(char **str,const char *delims)
{
    char *token;
    if(!*str)
    {
        return NULL;
    }
    token=*str;
    while(**str!='\0')
    {
        if(strchr(delims,**str))
        {
            **str='\0';
            (*str)++;
            return token;
        }
        (*str)++;
    }
    *str=NULL;
    return token;
}

int mkdirs(char *folder_path)
{
    if(!_access(folder_path,0))
    {
        return 1;
    }

    char path[256];
    char *path_buf;
    char temp_path[256];
    char *temp;
    int temp_len;

    memset(path,0,sizeof(path));
    memset(temp_path,0,sizeof(temp_path));
    strcat(path,folder_path);
    path_buf=path;
    int first_dir=1;
    while((temp=strsep(&path_buf,"/"))!=NULL)
    {
        temp_len=strlen(temp);
        if(0==temp_len)
        {
            continue;
        }
        if(!first_dir)
        {
            strcat(temp_path,"/");
        }
        first_dir=0;
        strcat(temp_path,temp);
        if(-1==_access(temp_path,0))
        {
            if(-1==_mkdir(temp_path))
            {
                return 2;
            }
        }
    }

    return 1;
}

am_data_ptr hal_data_init(const char*node)
{
    int ret;
    char temp_path[256]={0};
    am_data_ptr dp=(am_data_ptr)malloc(sizeof(am_data));

    if(!dp)
    {
        return NULL;
    }

    if(node==NULL)
    {
        goto free_data;
    }

    //只获取不包含文件名的路径
    memcpy(temp_path,node,(uint32_t)get_rpfile_name(node)-(uint32_t)node);
    mkdirs(temp_path);

    ret=pthread_mutex_init(&dp->mutex,NULL);
    if(ret) goto free_data;

    if(_access(node,0)==0)
    {
        dp->fp=fopen((const char *)node,"rb+");
    }
    else
    {
        dp->fp=fopen((const char *)node,"wb+");
    }
    if(!dp->fp)
    {
        goto free_mutex;
    }
    strncpy(dp->name,node,sizeof(dp->name));
    return dp;
    free_mutex:
    pthread_mutex_destroy(&dp->mutex);
    free_data:
    free(dp);
    return NULL;
}

uint32_t hal_data_write(am_data_ptr dp,uint32_t offset,uint32_t len,uint8_t* buff)
{
    uint32_t w_len;
    pthread_mutex_lock(&dp->mutex);

    fseek(dp->fp,offset,SEEK_SET);
    w_len=fwrite(buff,1,len,dp->fp);
    fflush(dp->fp);
    pthread_mutex_unlock(&dp->mutex);
    return w_len;
}



typedef enum 
{
    VERSION_A, 
    VERSION_B, 
    VERSION_C, 

    VERSION_UNKNOWN 
} file_version_enum;


typedef struct
{
    char version_id[8]; // 假设版本标识是一个4字符的字符串，例如"V1.0"
    uint32_t struct_size; // 结构体数据的大小
    time_t timestamp;
} file_header;

// 函数来比较版本字符串和枚举
file_version_enum get_version_enum(const char* version_id) 
{
    if (strncmp(version_id, "V1.0", 4) == 0) 
    {
        return VERSION_A;
    } 
    else if (strncmp(version_id, "V2.0", 4) == 0)
    {
        return VERSION_B;
    } 
    else if (strncmp(version_id, "V3.0", 4) == 0) 
    {
        return VERSION_C;
    } 
    else
    {
        return VERSION_UNKNOWN;
    }
}

// 使用版本枚举来读取数据
uint32_t read_data_with_version_enum(am_data_ptr dp, uint32_t offset, void* buff, size_t buff_size) 
{
    file_header header;
    uint32_t read_bytes;

    // 首先读取文件头部
    read_bytes = hal_data_read(dp, offset, sizeof(file_header), (uint8_t*)&header);
    if (read_bytes < sizeof(file_header)) 
    {
        // 处理错误或文件大小不符合的情况
        return -1;
    }

    // 获取版本枚举
    file_version_enum version = get_version_enum(header.version_id);

    switch (version) 
    {
        case VERSION_A:
            // 处理版本A的读取方式
            break;
        case VERSION_B:
            // 处理版本B的读取方式
            break;
        case VERSION_C:
            // 处理版本C的读取方式
            break;
        default:
            // 处理未知版本或者进行错误处理
            break;
    }


    return read_bytes;
}





int his_segment_read(area_struct* area_s)
{
    char path[128]="/root/vslam/history_map_save/history_first/room1";

    am_data_ptr segment_ori_addr=hal_data_init(path);

    uint32_t devia=0;
    uint32_t check_sum=0;
    uint32_t orphan_sum=0;

    hal_data_read(segment_ori_addr,devia,sizeof(uint8_t),(uint8_t*)&area_s->area_count);
    devia+=sizeof(uint8_t);
    hal_data_read(segment_ori_addr,devia,sizeof(int16_t),(uint8_t*)&area_s->convert_x);
    devia+=sizeof(int16_t);
    hal_data_read(segment_ori_addr,devia,sizeof(int16_t),(uint8_t*)&area_s->convert_y);
    devia+=sizeof(int16_t);
    hal_data_read(segment_ori_addr,devia,sizeof(float),(uint8_t*)&area_s->convert_theta);
    devia+=sizeof(float);
    hal_data_read(segment_ori_addr,devia,sizeof(uint8_t),(uint8_t*)&area_s->orphan_count);
    devia+=sizeof(uint8_t);

    area_s->area_links=(link_struct*)malloc_space(area_s->area_count*sizeof(link_struct),"ms-hsr-0");
    area_s->orphan_links=(orphans_struct*)malloc_space(area_s->orphan_count*sizeof(orphans_struct),"ms-hsr-1");

    for(int i=0;i<area_s->area_count;i++)
    {
        hal_data_read(segment_ori_addr,devia,sizeof(uint16_t),(uint8_t*)&area_s->area_links[i].id);
        devia+=sizeof(uint16_t);
        hal_data_read(segment_ori_addr,devia,sizeof(int),(uint8_t*)&area_s->area_links[i].isoland);
        devia+=sizeof(int);
        hal_data_read(segment_ori_addr,devia,sizeof(int16_t),(uint8_t*)&area_s->area_links[i].max_x);
        devia+=sizeof(int16_t);
        hal_data_read(segment_ori_addr,devia,sizeof(int16_t),(uint8_t*)&area_s->area_links[i].max_y);
        devia+=sizeof(int16_t);
        hal_data_read(segment_ori_addr,devia,sizeof(int16_t),(uint8_t*)&area_s->area_links[i].min_x);
        devia+=sizeof(int16_t);
        hal_data_read(segment_ori_addr,devia,sizeof(int16_t),(uint8_t*)&area_s->area_links[i].min_y);
        devia+=sizeof(int16_t);
        hal_data_read(segment_ori_addr,devia,sizeof(int32_t),(uint8_t*)&area_s->area_links[i].square);
        devia+=sizeof(int32_t);
        hal_data_read(segment_ori_addr,devia,sizeof(int16_t),(uint8_t*)&area_s->area_links[i].link_count);
        devia+=sizeof(int16_t);
        hal_data_read(segment_ori_addr,devia,sizeof(int8_t),(uint8_t*)&area_s->area_links[i].obstacle_count);
        devia+=sizeof(int8_t);
        hal_data_read(segment_ori_addr,devia,sizeof(int8_t),(uint8_t*)&area_s->area_links[i].near_count);
        devia+=sizeof(int8_t);
        hal_data_read(segment_ori_addr,devia,sizeof(int8_t),(uint8_t*)&area_s->area_links[i].door_count);
        devia+=sizeof(int8_t);
        hal_data_read(segment_ori_addr,devia,sizeof(int8_t),(uint8_t*)&area_s->area_links[i].orphan_count);
        devia+=sizeof(int8_t);


        area_s->area_links[i].link=(single_link_struct*)malloc_space(area_s->area_links[i].link_count*sizeof(single_link_struct),"ms-hsr-1");
        for(int j=0;j<area_s->area_links[i].link_count;j++)
        {
            hal_data_read(segment_ori_addr,devia,sizeof(uint16_t),(uint8_t*)&area_s->area_links[i].link[j].point.Num);
            devia+=sizeof(uint16_t);
            hal_data_read(segment_ori_addr,devia,sizeof(int16_t),(uint8_t*)&area_s->area_links[i].link[j].point.M);
            devia+=sizeof(int16_t);
            hal_data_read(segment_ori_addr,devia,sizeof(int16_t),(uint8_t*)&area_s->area_links[i].link[j].point.N);
            devia+=sizeof(int16_t);
            hal_data_read(segment_ori_addr,devia,sizeof(int16_t),(uint8_t*)&area_s->area_links[i].link[j].Jump);
            devia+=sizeof(int16_t);
            hal_data_read(segment_ori_addr,devia,sizeof(bool),(uint8_t*)&area_s->area_links[i].link[j].IS_Door);
            devia+=sizeof(bool);

            check_sum++;
        }

        area_s->area_links[i].obstacle=(p16_array*)malloc_space(area_s->area_links[i].obstacle_count*sizeof(p16_array),"ms-hsr-6");

        for(int j=0;j<area_s->area_links[i].obstacle_count;j++)
        {
            hal_data_read(segment_ori_addr,devia,sizeof(int16_t),(uint8_t*)&area_s->area_links[i].obstacle[j].size);
            devia+=sizeof(int16_t);

            area_s->area_links[i].obstacle[j].p16=(p16_struct*)malloc_space(area_s->area_links[i].obstacle[j].size*sizeof(p16_struct),"ms-hsr-7");

            for(int k=0;k<area_s->area_links[i].obstacle[j].size;k++)
            {
                hal_data_read(segment_ori_addr,devia,sizeof(int16_t),(uint8_t*)&area_s->area_links[i].obstacle[j].p16[k].first);
                devia+=sizeof(int16_t);
                hal_data_read(segment_ori_addr,devia,sizeof(int16_t),(uint8_t*)&area_s->area_links[i].obstacle[j].p16[k].second);
                devia+=sizeof(int16_t);

                check_sum++;

            }
        }

        area_s->area_links[i].near=(uint8_t*)malloc_space(area_s->area_links[i].near_count*sizeof(uint8_t),"ms-hsr-2");
        for(int j=0;j<area_s->area_links[i].near_count;j++)
        {
            hal_data_read(segment_ori_addr,devia,sizeof(uint8_t),(uint8_t*)&area_s->area_links[i].near[j]);
            devia+=sizeof(uint8_t);
            check_sum++;
        }

        area_s->area_links[i].door=(uint8_t*)malloc_space(area_s->area_links[i].door_count*sizeof(uint8_t),"ms-hsr-3");
        for(int j=0;j<area_s->area_links[i].door_count;j++)
        {
            hal_data_read(segment_ori_addr,devia,sizeof(uint8_t),(uint8_t*)&area_s->area_links[i].door[j]);
            devia+=sizeof(uint8_t);
            check_sum++;
        }

        area_s->area_links[i].orphan=(orphan_struct*)malloc_space(area_s->area_links[i].orphan_count*sizeof(orphan_struct),"ms-hsr-4");
        for(int j=0;j<area_s->area_links[i].orphan_count;j++)
        {
            hal_data_read(segment_ori_addr,devia,sizeof(uint16_t),(uint8_t*)&area_s->area_links[i].orphan[j].id);
            devia+=sizeof(uint16_t);
            hal_data_read(segment_ori_addr,devia,sizeof(uint16_t),(uint8_t*)&area_s->area_links[i].orphan[j].num);
            devia+=sizeof(uint16_t);
            check_sum++;
        }

    }

    for(int i=0;i<area_s->orphan_count;i++)
    {
        hal_data_read(segment_ori_addr,devia,sizeof(uint16_t),(uint8_t*)&area_s->orphan_links[i].id);
        devia+=sizeof(uint16_t);
        hal_data_read(segment_ori_addr,devia,sizeof(int16_t),(uint8_t*)&area_s->orphan_links[i].M);
        devia+=sizeof(int16_t);
        hal_data_read(segment_ori_addr,devia,sizeof(int16_t),(uint8_t*)&area_s->orphan_links[i].N);
        devia+=sizeof(int16_t);
        hal_data_read(segment_ori_addr,devia,sizeof(uint16_t),(uint8_t*)&area_s->orphan_links[i].relative_count);
        devia+=sizeof(uint16_t);
        hal_data_read(segment_ori_addr,devia,sizeof(uint16_t),(uint8_t*)&area_s->orphan_links[i].parent_count);
        devia+=sizeof(uint16_t);


        area_s->orphan_links[i].relatives=(indicate_struct*)malloc_space(area_s->orphan_links[i].relative_count*sizeof(indicate_struct),"ms-hsr-5");

        for(int j=0;j<area_s->orphan_links[i].relative_count;j++)
        {
            hal_data_read(segment_ori_addr,devia,sizeof(bool),(uint8_t*)&area_s->orphan_links[i].relatives[j].Parent);
            devia+=sizeof(bool);
            hal_data_read(segment_ori_addr,devia,sizeof(uint16_t),(uint8_t*)&area_s->orphan_links[i].relatives[j].link_id);
            devia+=sizeof(uint16_t);
            orphan_sum++;
        }

    }

    uint32_t checkread=0;
    uint32_t checkorphan=0;

    hal_data_read(segment_ori_addr,devia,sizeof(uint32_t),(uint8_t*)&checkread);
    devia+=sizeof(uint32_t);
    hal_data_read(segment_ori_addr,devia,sizeof(uint32_t),(uint8_t*)&checkorphan);
    devia+=sizeof(uint32_t);

    hal_data_deinit(segment_ori_addr);

    if(area_s->area_count<=0)
    {
        return -1;
    }

    if(checkread!=check_sum)
    {
        return -1;
    }

    return 1;

}





