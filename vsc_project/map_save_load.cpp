#include <string>
#include <fstream>
#include <mutex>
#include <nlohmann/json.hpp>
#include <iostream>

#include <stdio.h>
#include <direct.h> // 对于 Windows
#include <errno.h>  // 错误码

using json=nlohmann::json;

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

//area_struct的转换
namespace nlohmann
{
    template<>
    struct adl_serializer<single_point_struct>
    {
        static void to_json(json& j,const single_point_struct& point)
        {
            j=json{
                {"Num",point.Num},
                {"M",point.M},
                {"N",point.N}
            };
        }

        static void from_json(const json& j,single_point_struct& point)
        {
            j.at("Num").get_to(point.Num);
            j.at("M").get_to(point.M);
            j.at("N").get_to(point.N);
        }
    };

    template<>
    struct adl_serializer<single_link_struct>
    {
        static void to_json(json& j,const single_link_struct& link)
        {
            j=json{
                {"point",link.point},
                {"Jump",link.Jump},
                {"IS_Door",link.IS_Door}
            };
        }

        static void from_json(const json& j,single_link_struct& link)
        {
            j.at("point").get_to(link.point);
            j.at("Jump").get_to(link.Jump);
            j.at("IS_Door").get_to(link.IS_Door);
        }
    };

    template<>
    struct adl_serializer<orphan_struct>
    {
        static void to_json(json& j,const orphan_struct& orphan)
        {
            j=json{
                {"id",orphan.id},
                {"num",orphan.num}
            };
        }

        static void from_json(const json& j,orphan_struct& orphan)
        {
            j.at("id").get_to(orphan.id);
            j.at("num").get_to(orphan.num);
        }
    };

    template<>
    struct adl_serializer<p16_struct>
    {
        static void to_json(json& j,const p16_struct& p16)
        {
            j=json{
                {"first",p16.first},
                {"second",p16.second}
            };
        }

        static void from_json(const json& j,p16_struct& p16)
        {
            j.at("first").get_to(p16.first);
            j.at("second").get_to(p16.second);
        }
    };

    template<>
    struct adl_serializer<p16_array>
    {
        static void to_json(json& j,const p16_array& obstacle)
        {
            std::vector<json> p16s;
            for(int i=0;i<obstacle.size;i++)
            {
                p16s.push_back(obstacle.p16[i]);
            }
            j["p16"]=p16s;
        }

        static void from_json(const json& j,p16_array& obstacle)
        {
            obstacle.size=j.at("p16").size();

            if(obstacle.size>0)
            {
                obstacle.p16=static_cast<p16_struct*>(std::malloc(obstacle.size*sizeof(p16_struct)));

                auto& jsonArray = j.at("p16");
                for(int i=0;i<obstacle.size;i++)
                {
                    jsonArray[i].get_to(obstacle.p16[i]);
                }
            }
        }
    };

    template<>
    struct adl_serializer<link_struct>
    {
        static void to_json(json& j,const link_struct& area_links)
        {
            j=json{
                {"id",area_links.id},
                {"isoland",area_links.isoland},
                {"cleaned",area_links.cleaned},
                {"inside",area_links.inside},
                {"max_x",area_links.max_x},
                {"max_y",area_links.max_y},
                {"min_x",area_links.min_x},
                {"min_y",area_links.min_y},
                {"square",area_links.square}
            };

            std::vector<json> links;
            for(int i=0;i<area_links.link_count;i++)
            {
                links.push_back(area_links.link[i]);
            }
            j["link"]=links;

            std::vector<json> obstacles;
            for(int i=0;i<area_links.obstacle_count;i++)
            {
                links.push_back(area_links.obstacle[i]);
            }
            j["obstacle"]=obstacles;

            std::vector<uint8_t> nears;
            for(int i=0;i<area_links.near_count;i++)
            {
                nears.push_back(area_links.near[i]);
            }
            j["near"]=nears;

            std::vector<uint8_t> doors;
            for(int i=0;i<area_links.door_count;i++)
            {
                doors.push_back(area_links.door[i]);
            }
            j["door"]=doors;

            std::vector<json> orphans;
            for(int i=0;i<area_links.orphan_count;i++)
            {
                orphans.push_back(area_links.orphan[i]);
            }
            j["orphan"]=orphans;
        }

        static void from_json(const json& j,link_struct& area_links)
        {
            j.at("id").get_to(area_links.id);
            j.at("isoland").get_to(area_links.isoland);
            j.at("cleaned").get_to(area_links.cleaned);
            j.at("inside").get_to(area_links.inside);
            j.at("max_x").get_to(area_links.max_x);
            j.at("max_y").get_to(area_links.max_y);
            j.at("min_x").get_to(area_links.min_x);
            j.at("min_y").get_to(area_links.min_y);
            j.at("square").get_to(area_links.square);
        }
    };

    template<>
    struct adl_serializer<indicate_struct>
    {
        static void to_json(json& j,const indicate_struct& relatives)
        {
            j=json{
                {"Parent",relatives.Parent},
                {"link_id",relatives.link_id}
            };
        }
    };

    template<>
    struct adl_serializer<orphans_struct>
    {
        static void to_json(json& j,const orphans_struct& orphan_links)
        {
            j=json{
                {"id",orphan_links.id},
                {"M",orphan_links.M},
                {"N",orphan_links.N},
                {"parent_count",orphan_links.parent_count}
            };

            std::vector<json> relativess;
            for(int i=0;i<orphan_links.relative_count;i++)
            {
                relativess.push_back(orphan_links.relatives[i]);
            }
            j["relatives"]=relativess;
        }
    };

    template<>
    struct adl_serializer<area_struct>
    {
        static void to_json(json& j,const area_struct& area)
        {
            j=json{
                {"convert_x",area.convert_x},
                {"convert_y",area.convert_y},
                {"convert_theta",area.convert_theta}
            };

            std::vector<json> area_linkss;
            for(int i=0;i<area.area_count;i++)
            {
                area_linkss.push_back(area.area_links[i]);
            }
            j["area_links"]=area_linkss;

            std::vector<json> orphan_linkss;
            for(int i=0;i<area.orphan_count;i++)
            {
                orphan_linkss.push_back(area.orphan_links[i]);
            }
            j["orphan_links"]=orphan_linkss;
        }
    };
}


bool createDirectory(const std::string& path) {
    if (_mkdir(path.c_str()) == 0 || errno == EEXIST) {
        return true;
    }
    return false;
}

std::mutex file_mutex;





