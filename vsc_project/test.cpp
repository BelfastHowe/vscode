#include <vector>
#include <stack>
#include <iostream>
#include <queue>
#include <map>
#include <functional>
#include <opencv2/opencv.hpp>
#include <random>
#include <set>



using p64=std::pair<int,int>;
using MatrixInt=std::vector<std::vector<int>>;

template <typename T>
using Matrix=std::vector<std::vector<T>>;


class Room {
private:
    int room_id;//房间编号
    std::vector<std::pair<int, int>> pixels;//房间内的像素点

    std::vector<std::pair<int, std::pair<std::pair<int, int>, std::pair<int, int>>>> connected_rooms;//房间关于门的连通信息
    std::vector<std::pair<int,p64>> connection_info;

    std::vector<std::pair<int, int>> outline_pixels;//外轮廓像素点列表

public:
    Room(int room_id=0);//构造函数

    void add_pixel(std::pair<int, int> pixel);//添加像素点

    int get_pixel_count() const;//获取像素点数量

    void clear_pixels(){pixels.clear();}

    std::string to_string() const;//输出房间信息

    int get_room_id() const;//获取房间编号

    const std::vector<std::pair<int, int>>& get_pixels() const;//获取房间内的像素点

    void add_connected_room(int room_id, const std::pair<std::pair<int, int>, std::pair<int, int>>& door);//添加房间关于门的连通信息

    void print_connected_rooms() const;//输出房间关于门的连通信息

    void calculate_outline(const std::vector<std::vector<int>>& matrix);//计算并保存外轮廓像素点

    const std::vector<std::pair<int, int>>& get_outline_pixels() const;//获取外轮廓像素点列表

    void add_connection_info(int room_id,p64 door_id);

    const std::vector<std::pair<int,p64>>& get_connection_info() const;

    void delete_connection_info(int id);

    void clear_connection_info(){connection_info.clear();}

};

Room::Room(int room_id):room_id(room_id){}

const std::vector<std::pair<int,p64>>& Room::get_connection_info() const
{
    return connection_info;
}

void Room::add_connection_info(int room_id,p64 door_id)
{
    connection_info.push_back(std::make_pair(room_id,door_id));
}

void Room::delete_connection_info(int id) 
{    
    for (auto it = connection_info.begin(); it != connection_info.end(); ) 
    {
        if (it->first == id) 
        {
            it = connection_info.erase(it);
        } 
        else 
        {
            ++it;
        }
    }
}

class Area:public Room
{
private:
    int area_id;
    int h;
    int w;
    std::map<int,std::pair<Room,Room>> rooms;


    


};

struct Door 
{
    std::pair<int, int> startPoint;
    std::pair<int, int> endPoint;
    std::vector<std::pair<int, int>> path;

    // 带有默认值的构造函数
    Door(std::pair<int, int> p1 = {-1, -1}, std::pair<int, int> p2 = {-1, -1}, std::vector<std::pair<int, int>> l = {})
    : startPoint(p1), endPoint(p2), path(l)
    {}
};




bool is_valid_pixel(int x, int y, int rows, int cols) 
{
    return (x >= 0 && x < rows && y >= 0 && y < cols);
}

std::vector<p64> bresenham4(int x0, int y0, int x1, int y1)
{
    std::vector<p64> points;
    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);
    int sgnX = x0 < x1 ? 1 : -1;
    int sgnY = y0 < y1 ? 1 : -1;
    int e = 0;
    for (int i = 0; i < dx + dy; ++i)
    {
        points.push_back(std::make_pair(x0, y0));
        int e1 = e + dy;
        int e2 = e - dx;
        if (abs(e1) < abs(e2))
        {
            x0 += sgnX;
            e = e1;
        }
        else
        {
            y0 += sgnY;
            e = e2;
        }
    }
    return points;
}


std::pair<Matrix<int>, std::map<int, Room>> expand_rooms_simple(const Matrix<int>& segmented_matrix, const std::map<int, Room>& rooms)
{
    //凹角膨胀存在运行过慢和边缘紧贴房间图的情况，这里采用通用膨胀方法。

    //创建副本，默认染色
    std::map<int, Room> expanded_rooms = rooms;
    Matrix<int> expanded_matrix = segmented_matrix;

    int h = segmented_matrix.size();
    int w = segmented_matrix[0].size();

    //首先确定膨胀边界
    struct Boundary
    {
        int min_x;
        int min_y;
        int max_x;
        int max_y;

        Boundary(int x0=-1,int y0=-1,int x1=-1,int y1=-1)
        :min_x(x0),min_y(y0),max_x(x1),max_y(y1)
        {}
    };

    std::map<int,Boundary> all_boundary;

    for(const auto&room:rooms)
    {
        int room_id=room.first;
        
        int min_x=h,min_y=w,max_x=0,max_y=0;
        for(const auto&p:room.second.get_pixels())
        {
            min_x=std::min(min_x,p.first);
            min_y=std::min(min_y,p.second);
            max_x=std::max(max_x,p.first);
            max_y=std::max(max_y,p.second);
        }
        min_x-=10;
        min_y-=10;
        max_x+=10;
        max_y+=10;

        Boundary boundary(min_x,min_y,max_x,max_y);

        all_boundary.insert(std::map<int,Boundary>::value_type(room_id,boundary));

    }

    std::function<bool(int,int,int)> is_in_boundary=[&](int i,int j,int roomid)->bool
    {
        //找到当前边界
        Boundary curb=all_boundary[roomid];

        bool tar=i>=curb.min_x&&i<=curb.max_x&&j>=curb.min_y&&j<=curb.max_y;

        return tar;
    };

    bool expansion_occurred = true;

    while(expansion_occurred)
    {
        expansion_occurred=false;

        //待膨胀栈
        std::stack<std::pair<p64,int>> expansion;

        for (int i = 1; i < h-1; i++)
        {
            for (int j = 1; j < w-1; j++)
            {
                if (expanded_matrix[i][j] == 0)
                {
                    //获取周围八个点的像素值
                    int p[9];
                    p[0] = expanded_matrix[i][j];     // P1
                    p[1] = expanded_matrix[i - 1][j];   // P2
                    p[2] = expanded_matrix[i - 1][j + 1]; // P3
                    p[3] = expanded_matrix[i][j + 1];   // P4
                    p[4] = expanded_matrix[i + 1][j + 1]; // P5
                    p[5] = expanded_matrix[i + 1][j];   // P6
                    p[6] = expanded_matrix[i + 1][j - 1]; // P7
                    p[7] = expanded_matrix[i][j - 1];   // P8
                    p[8] = expanded_matrix[i - 1][j - 1]; // P9

                    //四连通的值
                    std::set<int> con4={p[1],p[3],p[5],p[7]};

                    if(con4.size()==2&&*con4.begin()==0)
                    {
                        int trid=*con4.rbegin();

                        //d邻域的值
                        std::set<int> cond={p[2],p[4],p[6],p[8]};

                        if(std::all_of(cond.begin(),cond.end(),[trid](int pn)
                        {
                            return pn==0||pn==trid;
                        }))
                        {
                            if(is_in_boundary(i,j,trid))
                            {
                                expansion.push(std::make_pair(std::make_pair(i,j),trid));

                                expansion_occurred=true;
                            }
                            
                        }

                    }



                }
            }
        }

        while(!expansion.empty())
        {
            std::pair<p64, int> pending = expansion.top();
            expansion.pop();

            p64 p = pending.first;
            int id = pending.second;

            std::set<int> judgment;
            for(int u=-1;u<=1;u++)
            {
                for(int v=-1;v<=1;v++)
                {
                    int nx=p.first+u;
                    int ny=p.second+v;
                    judgment.insert(expanded_matrix[nx][ny]);
                }
            }

            if(judgment.size()==2&&*judgment.begin()==0&&*judgment.rbegin()==id)
            {
                expanded_matrix[p.first][p.second]=id;
                expanded_rooms[id].add_pixel(p);
            }


        }
    }


    //计算拓展后的房间轮廓
    for (auto& room : expanded_rooms)
    {
        room.second.calculate_outline(expanded_matrix);
    }

    return std::make_pair(expanded_matrix, expanded_rooms);
}



std::pair<Matrix<int>, std::map<int, Room>> expand_rooms_queue(const Matrix<int>& segmented_matrix, const std::map<int, Room>& rooms)
{
    //经典凹角膨胀，队列或者堆栈版，期望优化速度
    //别的方法都很一般，还得是初版

    //创建双副本
    std::map<int, Room> expanded_rooms = rooms;
    Matrix<int> expanded_matrix = segmented_matrix;

    //四邻域
    std::vector<p64> directions4={{-1,0},{1,0},{0,-1},{0,1}};


    int h = segmented_matrix.size();
    int w = segmented_matrix[0].size();

    //膨胀判断函数
    std::function<int(int,int,Matrix<int>&)> isValidExpansionPoint=[&h,&w](int i,int j,Matrix<int>&src)->int
    {
        //四邻域和八邻域中的值
        std::vector<int> four_neighbours;
        std::vector<int> eight_neighbours;

        for(int dx=-1;dx<=1;dx++)
        {
            for(int dy=-1;dy<=1;dy++)
            {
                if(dx==0&&dy==0) continue;
                int nx=i+dx;
                int ny=j+dy;

                if(is_valid_pixel(nx,ny,h,w))
                {
                    int neighbour_value=src[nx][ny];
                    if(dx==0||dy==0)
                    {
                        four_neighbours.push_back(neighbour_value);
                    }
                    eight_neighbours.push_back(neighbour_value);
                }
            }
        }

        int non_zero_value = 0;  // 用于保存四邻域内的非零值
        for (int value : four_neighbours) 
        {
            if (value != 0) 
            {
                if (non_zero_value == 0)
                {
                    non_zero_value = value;
                } 
                else if (non_zero_value != value) 
                {
                    return 0;  // 说明四邻域内有两种不同的非零值，不满足条件
                }
            }
        }

        if(non_zero_value==0) return 0;

        if (std::count(four_neighbours.begin(), four_neighbours.end(), non_zero_value) < 2) 
        {
            return 0;  // 四邻域内非零值的数量不足2
        }

        for (int value : eight_neighbours) 
        {
            if (value != 0 && value != non_zero_value) 
            {
                return 0;  // 八邻域内存在与四邻域非零值不同的其他非零值
            }
        }

        return non_zero_value;  // 所有条件都满足，返回true



    };


    std::queue<std::pair<p64,int>> expansion;

    //找到膨胀种子
    for(int i=0;i<h;i++)
    {
        for(int j=0;j<w;j++)
        {
            if(expanded_matrix[i][j]==0)
            {
                int room_id=isValidExpansionPoint(i,j,expanded_matrix);

                if(room_id==0) continue;

                expansion.push(std::make_pair(std::make_pair(i,j),room_id));
            }
        }
    }

    //开始依序膨胀
    while(!expansion.empty())
    {
        std::pair<p64,int> pending=expansion.front();
        expansion.pop();

        p64 p=pending.first;
        int id=pending.second;

        //二次判定待膨胀点，防止不同房间的两个点同时膨胀让房间相连通
        std::vector<int> judgment2;
        for (int d = -1; d <= 1; d++)
        {
            for (int f = -1; f <= 1; f++)
            {
                int u = p.first + d;
                int v = p.second + f;
                if (is_valid_pixel(u, v, h, w))
                {
                    judgment2.push_back(expanded_matrix[u][v]);
                }
            }
        }

        if (std::all_of(judgment2.begin(), judgment2.end(), [id](int pixel) {return pixel == id || pixel == 0; }))
        {
            expanded_matrix[p.first][p.second] = id;
            expanded_rooms[id].add_pixel(p);
        


            //从成功膨胀的点中拓展下线
            for(const auto&d:directions4)
            {
                int nx=p.first+d.first;
                int ny=p.second+d.second;

                if(expanded_matrix[nx][ny]==0)
                {
                    int room_id=isValidExpansionPoint(nx,ny,expanded_matrix);

                    if(room_id==0) continue;

                    expansion.push(std::make_pair(std::make_pair(nx,ny),room_id));
                }
            }

        }


    }


    //计算拓展后的房间轮廓
    for (auto& room : expanded_rooms)
    {
        room.second.calculate_outline(expanded_matrix);
    }

    return std::make_pair(expanded_matrix, expanded_rooms);


}


int getDirection(p64 p1,p64 p2)
{

}

int gap_enlargement(Matrix<int>&expanded_matrix,std::map<int,Room>&expanded_rooms)
{
    int h=expanded_matrix.size();
    int w=expanded_matrix[0].size();

    //构建全房间转折点列表
    std::vector<std::pair<int,std::vector<p64>>> fpc;

    for(auto&room:expanded_rooms)
    {
        int rd=room.first;

        std::vector<p64> allPoints=room.second.get_outline_pixels();
        std::vector<p64> turnPoints;

        int n=allPoints.size();
        for(int i=0;i<n;i++)
        {
            p64 p1=allPoints[i];
            p64 p2=allPoints[(i+1)%n];
            p64 p3=allPoints[(i+2)%n];

            int dir1=getDirection(p1,p2);
            int dir2=getDirection(p2,p3);

            if(dir1!=dir2)
            {
                turnPoints.push_back(p2);
            }
        }
        fpc.push_back(std::make_pair(rd,turnPoints));

    }

    //逐级变形十次
    for(int getime=0;getime<10;getime++)
    {
        //单个房间
        for(int fpcn=0;fpcn<fpc.size();fpcn++)
        {
            int room_id=fpc[fpcn].first;
            std::vector<p64> cnt=fpc[fpcn].second;

            Matrix<int> mask(h,std::vector<int>(w,0));

            //背景限制矩阵
            for(auto&mask_cnt:fpc)
            {
                if(mask_cnt.first==room_id) continue;

                
            }
        }
    }


}


std::pair<MatrixInt, std::map<int, Room>> segment_rooms(MatrixInt& src, std::map<p64, Door>& doorMap, MatrixInt& bgmask)
{
    std::cout << "开始分割" << std::endl;

    int h = src.size();
    int w = src[0].size();

    MatrixInt segmented_matrix = src; //创建副本
    Matrix<bool> visited(h, std::vector<bool>(w, false));//创建与矩阵大小相同的访问标记矩阵

    std::map<int, Room> rooms;
    int room_id = 1;

    std::vector<p64> directions = { {-1, 0}, {1, 0}, {0, -1}, {0, 1} };//定义DFS算法四个方向的偏移量


    std::function<void(int, int, Room&)> dfs = [&](int x, int y, Room& room)
    {
        std::stack<std::pair<int, int>> stack;
        stack.push(std::make_pair(x, y));//将当前像素点压入栈中

        while (!stack.empty())
        {
            std::pair<int, int> current = stack.top();
            stack.pop();//将当前像素点弹出栈

            int current_x = current.first;
            int current_y = current.second;
            room.add_pixel(current);//将当前像素点添加到房间中

            visited[current_x][current_y] = true;//将当前像素点标记为已访问

            for (const auto& direction : directions)
            {
                int next_x = current_x + direction.first;
                int next_y = current_y + direction.second;

                if (is_valid_pixel(next_x, next_y, h, w) && segmented_matrix[next_x][next_y] == 1 && !visited[next_x][next_y])
                {
                    stack.push(std::make_pair(next_x, next_y));//将下一个点压入栈中
                }
            }
        }
    };


    //门重叠点置为背景
    for (int i = 0; i < h; i++)
    {
        for (int j = 0; j < w; j++)
        {
            if (bgmask[i][j] == 1)
            {
                segmented_matrix[i][j] = 0;
            }
        }
    }


    // 通过门像素进行分割
    for (const auto& door : doorMap)
    {
        auto& door_path = door.second.path;
        for (const auto& p : door_path)
        {
            int x = p.first;
            int y = p.second;
            if (is_valid_pixel(x, y, h, w))
            {
                segmented_matrix[x][y] = 0;
            }
        }
    }

    for (int i = 0; i < h; i++)
    {
        for (int j = 0; j < w; j++)
        {
            if (segmented_matrix[i][j] == 1 && !visited[i][j])
            {
                Room room(room_id);
                dfs(i, j, room);
                rooms.insert(std::map<int, Room>::value_type(room_id, room));
                room_id++;
            }
        }
    }

    std::cout << "分割完成" << std::endl;

    return std::make_pair(segmented_matrix, rooms);

}






