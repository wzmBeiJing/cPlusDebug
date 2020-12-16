#include<iostream>
#include<algorithm>
#include<vector>
#include "CChain.h"
#include "stdlib.h"
#include "string.h"
#define M_PI 3.1415926

using namespace std;
using namespace flight;

CChainExtraction chain_extraction_;

int checkChain(pose_s &pose)
	{
		//Update Chain List & Detect Loop
		//-1- Detect Loop
		//-2- Check_Chain not empty, detect section
		if (chain_extraction_.generateChain(pose) || chain_extraction_.recheckChain()) {
			std::cout << "<----------- chain detect -------------->" << std::endl;

			if (chain_extraction_.detectSectionChain()) {
				std::cout << "--------the section chain detect----------!!" << std::endl;

				if (chain_extraction_.isRealChain(pose)) {
					// PX4_WARN("the section chain detect!!");
					chain_extraction_.clearChainList();
					return 2;  // section chain
				}

			} else {
				//model_->stop();
				// chain_extraction_.clearChainList();
				chain_extraction_.clearCChain();
				std::cout << "the island chian detect..." << std::endl;
				return 0;
			}
		}

		return 1;
	}


void print_ivec(vector<int>::iterator begin, vector<int>::iterator end)
{
    for(;begin != end; ++begin)
        cout << *begin << '\t';
    cout << endl;
}

double wrapAngle(double a)
{
	a = fmod(a + M_PI, 2 * M_PI);

	if (a < 0.0) {
		a += 2.0 * M_PI;
	}

	return a - M_PI;
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

void updateVecPose(std::vector<pose_s> &vec){
		vec.clear();
		vec.push_back(pose_s(1,1,0));
		vec.push_back(pose_s(2,2,0));
		vec.push_back(pose_s(3,3,0));
		vec.push_back(pose_s(4,4,0));
}


/*class A{
	public:

	void * operator new(size_t size)
	{
		if(size == 0)  
		size = 1;  
		void *res;  
		for(;;)  
		{  
		//allocate memory block  
		res = malloc(size);  
		//if successful allocation, return pointer to memory  
		if(res)  
			break;  
		//call installed new handler  
		}  
		printf("new Override A size %d\r\n", size);
		return res;  
	}

	void operator delete (void * pointer)
	{
		printf("delete %x\r\n", pointer);
		
		free(&(((A*)pointer)->a));
		free(pointer);
	}
	
	void operator delete [](void * pointer)
	{
		printf("delete []%x\r\n", pointer);
		free(pointer);
	}

	int a;
	
	std::vector<int> m[3];	
	A(int a_){
		a = a_;
		std::cout << "A constructor" << std::endl;
	}
	~A(){std::cout << "A deconstructor" << std::endl;}

};*/
/*class B : A{
	

}*/



#include <iostream>
#include <vector>
#include <ctime>
using namespace std;


/*void *operator new(size_t size)
{
	std::cout << "global size:" << size << std::endl;
	return malloc(size);
}
void *operator new[](size_t size) //数组版本
{
	std::cout << "global size:" << size << std::endl;
	return malloc(size);
}
void operator delete(void *phead)
{
	free(phead);
}
void operator delete[](void *phead)
{
	free(phead);
}*/

namespace _nmsp1 //命名空间
{	
	//一：重载全局operator new和operator delete函数
	   //重载全局operator new[]和operator delete[]函数
	class A
	{
	public:
		int m_i;
		int m_j;
		A()
		{
			cout << "A::A()" << endl;
		}
		~A()
		{
			cout << "A::~A()" << endl;
		}


		void *operator new(size_t size)
		{
			A *ppoint = (A*)malloc(size);
			std::cout << "local 1size:" << size << std::endl;
			return ppoint;
		}
		void *operator new[](size_t size) //数组版本
		{
			A *ppoint = (A*)malloc(size);
			std::cout << "local 1size:" << size << std::endl;
			return ppoint;
		}
		void operator delete(void *phead)
		{
			free(phead);
		}
		void operator delete[](void *phead)
		{
			free(phead);
		}
	};

	void func()
	{
		/*int *pint = new int(12); 
		delete pint;

		char *parr = new char[15];
		delete[] parr;
*/
		A *p = new A();
		delete p;

		A *pa = new A[1]();
		delete[] pa;
		
		
	}
}
namespace _nmsp2
{
	//二：定位new（placement new）
	//有placement new，但是没有对应的placement delete
	//功能：在已经分配的原始内存中初始化一个对象；
	 //a)已经分配，定位new并不分配内存，你需要提前将这个定位new要使用的内存分配出来
	 //b)初始化一个对象（初始化一个对象的内存），我们就理解成调用这个对象的构造函数；
	//说白了，定位new就是能够在一个预先分配好的内存地址中构造一个对象；
	//格式：
	//new (地址) 类类型()

	class A
	{
	public:
		int m_a;
		A() :m_a(0)
		{
			cout << "A::A()_0" << endl;
			int test;
			test = 1;
		}
		A(int tempvalue) :m_a(tempvalue)
		{
			cout << "A::A()_1" << endl;
			int test;
			test = 1;
		}
		~A()
		{
			cout << "A::~A()" << endl;
			int abc;
			abc = 1;
		}

		//传统new操作符重载
		void *operator new(size_t size)
		{
			std::cout << "new_0" << std::endl;
			A *ppoint = (A*)malloc(size);
			return ppoint;
		}

		//定位new操作符的重载
		void *operator new(size_t size,void *phead)
		{
			//A *ppoint = (A*)malloc(size);
			//return ppoint;
			std::cout << "new_1" << std::endl;
			A *ppoint = (A*)malloc(size);
			return phead; //收到内存开始地址，只需要原路返回
		}

	};
	   	 
	void func()
	{
		void *mymemPoint = (void *)new char[sizeof(A)]; //内存必须事先分配出来
		A *pmyAobj1 = new (mymemPoint) A(); //调用无参构造函数，这里并不会额外分配内存
		
		printf("mymemPoint:%0x\r\n",mymemPoint);
		printf("pmyAobj1:%0x\r\n",pmyAobj1);
		//A *pmyAobj3 = new A();

		/*void *mymemInt = (void *)new int; 
		int  *pmyint = new (mymemPoint) int();
		*/
		
		void *mymemPoint2 = (void *)new char[sizeof(A)];
		A *pmyAobj2 = new (mymemPoint2) A(12); //调用带一个参数的构造函数，这里并不会额外分配内存

		//delete pmyAobj1;
		//delete pmyAobj2;

		pmyAobj1->~A(); //手工调用析构函数是可以的，但手工调用构造函数一般不可以
		pmyAobj2->~A();
		delete[](void *)pmyAobj1;
		delete[](void *)pmyAobj2;

	}
}
namespace _nmsp3
{
	//三：多种版本的operator new重载
	//可以重载很多版本的operator new，只要每个版本参数不同就行，但是第一个参数是固定的，
	//都是size_t，表示你要new对象的sizeof值

	class A
	{
	public:
		void *operator new(size_t size, int tvp1, int tvp2)
		{
			return NULL;
		}
		A()
		{
			int test;
			test = 1;
		}
	};
	void func()
	{
		A *pa =  new (1234, 56) A(); //自定义 new 不调用构造函数

	}
}

/*int main()
{	
	//_nmsp1::func();		
	//_nmsp2::func();
	_nmsp3::func();
	return 1;
}*/


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

	/*vector<vector<int>> m = {
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

	std::vector<int> a;
	a.push_back(1);
	a.pop_back();

	std::cout << "a.size:" << a.size() << std::endl;*/
	
	std::vector<pose_s> v_pose_;
	//v_pose_.push_back(pose_s(-1.0,1.5,0));
	//v_pose_.push_back(pose_s(-0.5,1.5,0));
	//v_pose_.push_back(pose_s(0.0,1.5,0));
	//v_pose_.push_back(pose_s(0.5,1.5,0));
	//v_pose_.push_back(pose_s(1.0,1.5,0));
	v_pose_.push_back(pose_s(0.5,0.5,0));
	v_pose_.push_back(pose_s(0.4,0.5,0));
	v_pose_.push_back(pose_s(0.3,0.5,0));
	v_pose_.push_back(pose_s(0.2,0.5,0));
	v_pose_.push_back(pose_s(0.1,0.5,0));
	v_pose_.push_back(pose_s(0.0,0.5,0));

	//v_pose_.push_back(pose_s(1.5,1.0,0));
	//v_pose_.push_back(pose_s(1.5,0.5,0));
	//v_pose_.push_back(pose_s(1.5,0.0,0));
	//v_pose_.push_back(pose_s(1.5,-0.5,0));
	//v_pose_.push_back(pose_s(1.5,-1.0,0));
	v_pose_.push_back(pose_s(-0.1,0.5,0));
	v_pose_.push_back(pose_s(-0.2,0.5,0));
	v_pose_.push_back(pose_s(-0.3,0.5,0));
	v_pose_.push_back(pose_s(-0.4,0.5,0));
	v_pose_.push_back(pose_s(-0.5,0.5,0));

	//v_pose_.push_back(pose_s(1.0,-1.5,0));
	//v_pose_.push_back(pose_s(0.5,-1.5,0));
	//v_pose_.push_back(pose_s(0.0,-1.5,0));
	//v_pose_.push_back(pose_s(-0.5,-1.5,0));
	//v_pose_.push_back(pose_s(-1.0,-1.5,0));
	v_pose_.push_back(pose_s(-0.5,0.4,0));
	v_pose_.push_back(pose_s(-0.5,0.3,0));
	v_pose_.push_back(pose_s(-0.5,0.2,0));
	v_pose_.push_back(pose_s(-0.5,0.1,0));
	v_pose_.push_back(pose_s(-0.5,0.0,0));

	//v_pose_.push_back(pose_s(-1.5,-1.0,0));
	//v_pose_.push_back(pose_s(-1.5,-0.5,0));
	//v_pose_.push_back(pose_s(-1.5,0.0,0));
	//v_pose_.push_back(pose_s(-1.5,0.5,0));
	//v_pose_.push_back(pose_s(-1.5,1.0,0));
	v_pose_.push_back(pose_s(-0.5,-0.1,0));
	v_pose_.push_back(pose_s(-0.5,-0.2,0));
	v_pose_.push_back(pose_s(-0.5,-0.3,0));
	v_pose_.push_back(pose_s(-0.5,-0.4,0));
	v_pose_.push_back(pose_s(-0.5,-0.5,0));
	//v_pose_.push_back(pose_s(-1.2,1.5,0));
	//v_pose_.push_back(pose_s(-0.8,2.0,0));
	//v_pose_.push_back(pose_s(-0.8,0.9,0));
	//v_pose_.push_back(pose_s(0.9,1.3,0));
	//updateVecPose(v_pose_);
	//std::cout << "v_pose_size:" << v_pose_.size() << std::endl;
	//v_pose_.push_back(pose_s(0.5,0.5,0));
	v_pose_.push_back(pose_s(-0.4,-0.5,0));
	v_pose_.push_back(pose_s(-0.3,-0.5,0));
	v_pose_.push_back(pose_s(-0.2,-0.5,0));
	v_pose_.push_back(pose_s(-0.1,-0.5,0));
	v_pose_.push_back(pose_s(-0.0,-0.5,0));

	v_pose_.push_back(pose_s(0.1,-0.5,0));
	v_pose_.push_back(pose_s(0.2,-0.5,0));
	v_pose_.push_back(pose_s(0.3,-0.5,0));
	v_pose_.push_back(pose_s(0.4,-0.5,0));
	v_pose_.push_back(pose_s(0.5,-0.5,0));
	
	v_pose_.push_back(pose_s(0.5,-0.4,0));
	v_pose_.push_back(pose_s(0.5,-0.3,0));
	v_pose_.push_back(pose_s(0.5,-0.2,0));
	v_pose_.push_back(pose_s(0.5,-0.1,0));
	v_pose_.push_back(pose_s(0.5,-0.0,0));
	v_pose_.push_back(pose_s(0.5,0.1,0));
	v_pose_.push_back(pose_s(0.5,0.2,0));
	v_pose_.push_back(pose_s(0.5,0.3,0));
	v_pose_.push_back(pose_s(0.5,0.4,0));
	v_pose_.push_back(pose_s(0.5,0.5,0));
	v_pose_.push_back(pose_s(0.4,0.5,0));
	v_pose_.push_back(pose_s(0.3,0.5,0));
	v_pose_.push_back(pose_s(0.2,0.5,0));
	v_pose_.push_back(pose_s(0.1,0.5,0));
	v_pose_.push_back(pose_s(0.0,0.5,0));
	for(int i = 0;i < v_pose_.size();i++){
		std::cout << "i:" << i << std::endl;
		int res = checkChain(v_pose_[i]);	
		std::cout << "res:" << res << std::endl;
	}


	/*if(true){
		int a = 1;
		std::cout << "ai_1:" << a << std::endl;
		if(true){
			int a = 2;
			std::cout << "a_2:" << a << std::endl;
			if(true){
				int a = 3;
				std::cout << "a_3:" << a << std::endl;
			}
		}
	}*/

#define a

#if defined (a)

	//std::cout << "a" << std::endl;
#endif

<<<<<<< HEAD
double res = wrapAngle(1.56);	

printf("res:%f\n",res);
printf("VERSION:%s\n",VERSION);		    
printf("VERSION:%s\n",VERSION);		    

=======
//	double res = wrapAngle(1.56);	
//
//	printf("res:%f\n",res);
//	printf("VERSION:%s\n",VERSION);		    
//
//	std::vector<int> l_obj;
//
//	std::cout << "l_obj.capacity:" << l_obj.capacity() << std::endl;
//		
//	A *obj_a = new A(1);
//
//	int **vec_text = new int*[3];
//	for(int i = 0;i < 3;i++){
//		vec_text[i] = new int[3];
//		memset(vec_text[i],(uint8_t)0,3*4);	
//	}
//
//	int vector_ii[3];
//	std::cout << "vector_ii:" << sizeof(vector_ii) << std::endl;
//
//	std::cout << "vec_text[0][0]:" << vec_text[0][0] << std::endl;	
//	delete obj_a;
	//_nmsp1::func();

	//_nmsp2::func();
>>>>>>> e871654
	return 0;
}
