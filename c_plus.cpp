#include<iostream>
#include<algorithm>
#include<vector>
using namespace std;
void print_ivec(vector<int>::iterator begin, vector<int>::iterator end)
{
    for(;begin != end; ++begin)
        cout << *begin << '\t';
    cout << endl;
}

#include <iostream>
#include <vector>
#include <queue>

using namespace  std;

struct Point
{
    int r;
    int c;
    Point(int r_, int c_) : r(r_), c(c_){}
    Point(const Point& p) : r(p.r), c(p.c){}
};

class Solution
{
public:
    int m;
    int n;
    
	bool isvalid(int i, int j, vector<vector<int>>& matrix, vector<vector<bool>>& mask)
    {
        return i>=0 && i<m && j>=0 && j<n && !mask[i][j] && matrix[i][j]==1;
    }
    
	void add(int i, int j, vector<vector<int>>& matrix, queue<Point>& q, vector<vector<bool>>& mask)
    {
        if(isvalid(i, j, matrix, mask))
        {
            q.push(Point(i,j));
            mask[i][j]=true;
        }
    }
    
	vector<vector<Point>> bwlabel(vector<vector<int>> &matrix)
    {
        m=matrix.size(); 
        n=matrix[0].size();
        vector<vector<Point>> res;
        vector<Point> tmp;
        vector<vector<bool>> mask(m, vector<bool>(n,false));
        for(int i=0; i<m;i++)
        {
            for(int j=0; j<n; j++)
            {
                if(mask[i][j] || matrix[i][j] == 0)
                    continue;
                tmp.clear();
                queue<Point> q;
                q.push(Point(i,j));
                mask[i][j] = true;
                while(!q.empty())
                {
                    Point t = q.front();
                    q.pop();
                    tmp.push_back(t);

                    add(t.r-1, t.c, matrix, q, mask);
                    add(t.r+1, t.c, matrix, q, mask);
                    add(t.r, t.c-1, matrix, q, mask);
                    add(t.r, t.c+1, matrix, q, mask);

					
                    /*add(t.r-1, t.c-1, matrix, q, mask);
                    add(t.r+1, t.c+1, matrix, q, mask);
                    add(t.r+1, t.c-1, matrix, q, mask);
                    add(t.r-1, t.c+1, matrix, q, mask);*/
                }
                res.push_back(tmp);
            }
        }
        return res;
    }
};


int main(int argc, char* argv[])
{
    /*int a[] = {1, 12, 15, 20, 30};
    vector<int> ivec(a, a + sizeof(a) / sizeof(a[0]));
    print_ivec(ivec.begin(), ivec.end());
    make_heap(ivec.begin(), ivec.end(), greater<int>());
    print_ivec(ivec.begin(), ivec.end());
    pop_heap(ivec.begin(), ivec.end(),greater<int>());
    ivec.pop_back();
    print_ivec(ivec.begin(), ivec.end());
    ivec.push_back(99);
    print_ivec(ivec.begin(), ivec.end());
    push_heap(ivec.begin(), ivec.end(),greater<int>());
    print_ivec(ivec.begin(), ivec.end());
    sort_heap(ivec.begin(), ivec.end(),greater<int>());
    print_ivec(ivec.begin(), ivec.end());*/

#if defined(__PX4_POSIX_SITL_SIMULATE)
	std::cout << "__PX4_POSIX_SITL_SIMULATE" << std::endl;
#else
	std::cout << "__Hello_World" << std::endl;
#endif
	
	/*uint8_t left_ = 0;
	uint8_t front_left_ = 1;
	uint8_t right_ = 4;

	uint32_t cur_signal = ((left_ & 5) << 8) + ((front_left_ & 5) << 4) + (right_ & 5);

	std::cout << "cur_signal:" << hex << cur_signal << std::endl;
	
	std::cout << "test(!(3&4)):" << !(3&4) << std::endl;
	
	std::cout << "test(!(5&4)):" << !(5&4) << std::endl;

	std::cout << "test((3&4)):" << (3&4) << std::endl;
	
	std::cout << "test((5&4)):" << (5&4) << std::endl;*/

	vector<vector<int>> m = {
        {1,1,0,0,0},
        {1,0,1,0,0},
        {0,1,1,0,0},
        {0,0,0,1,0},
        {0,0,0,0,1},
        {0,0,0,0,0}
    };
    
	vector<vector<int>> n = {{}};
    Solution s;
    vector<vector<Point>> res = s.bwlabel(m);
	
	std::cout << "res_size_:" << res.size() << std::endl;
	
    vector<vector<Point>> rss = s.bwlabel(n);

	std::cout << "rss_size_:" << rss.size() << std::endl;
	std::cout << "rss_size_:" << rss.size() << std::endl;

    return 0;
}