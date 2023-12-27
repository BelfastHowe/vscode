#include <string>
#include <fstream>
#include <mutex>
#include <nlohmann/json.hpp>
#include <iostream>

#include <direct.h> // 对于 Windows
#include <errno.h>  // 错误码

struct Student {
    std::string name;
    int age;
    float gpa;
};

namespace nlohmann {
    template <>
    struct adl_serializer<Student> {
        static void to_json(json& j, const Student& s) {
            j = json{{"name", s.name}, {"age", s.age}, {"gpa", s.gpa}};
        }

        static void from_json(const json& j, Student& s) {
            s.name = j.at("name").get<std::string>();
            s.age = j.at("age").get<int>();
            s.gpa = j.at("gpa").get<float>();
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

int writeToJson(const Student& student, const std::string& filename) {
    std::lock_guard<std::mutex> lock(file_mutex); // 使用锁保护文件操作

    std::string directory = filename.substr(0, filename.find_last_of("\\/"));

    if (!createDirectory(directory)) 
    {
        std::cerr << "Failed to create directory: " << directory << std::endl;
        return -1;
    }

    std::ofstream file(filename);
    if (file.is_open()) {
        nlohmann::json j = student;
        file << j.dump(4);
        file.close();
        return 0;
    }
    else
    {
        return -1;
    }
}

int readFromJson(Student& student, const std::string& filename) {
    std::lock_guard<std::mutex> lock(file_mutex); // 使用锁保护文件操作
    std::ifstream file(filename);
    if (file.is_open()) {
        nlohmann::json j;
        file >> j;
        student = j.get<Student>();
        file.close();
        return 0;
    }
    else
    {
        return -1;
    }
}



int main() 
{
std::string json_filename="C:\\Users\\13012\\Desktop\\json\\student.json";

    Student student{"John Doe", 20, 3.5};

    // 写入到文件
    if(writeToJson(student, json_filename)!=0)
    {
        std::cerr<<"write err!"<<std::endl;
        return -1;
    }

    // 从文件读取
    Student loaded_student;
    if(readFromJson(loaded_student, json_filename)!=0)
    {
        std::cerr<<"read err!"<<std::endl;
        return -1;
    }

    // 输出读取的数据
    std::cout << "Name: " << loaded_student.name 
              << ", Age: " << loaded_student.age 
              << ", GPA: " << loaded_student.gpa << std::endl;

    return 0;
}



