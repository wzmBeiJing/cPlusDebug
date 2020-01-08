/*
 *
 */

//#include "model.h"
#include <deque>
#include <vector>
//#include <uORB/topics/occupancy_grid.h>

using namespace std;

namespace flight
{

struct pose_s{
	double x;
	double y;
	double z;
	double theta;
	double confidence;
	int timestamp;
	
	pose_s(){
		x = 0;
		y = 0;
		theta = 0;
	}

	pose_s(double x_,double y_,double theta_){
		x = x_;
		y = y_;
		theta = theta_;
	}
};

class CChainCell
{
public:
	CChainCell()
	{
	}

	~CChainCell()
	{
	}

	CChainCell(const int idx, const int idy, const pose_s &pose) : idx_(idx), idy_(idy), pose_(pose)
	{
	}

	int idx_;       //index[x, y] in list
	int idy_;
	pose_s pose_;   //robot pose
};

class CChainExtraction
{
public:
	CChainExtraction()
		: last_chain_size_(0)
		, recheck_count_(0)
		, recheck_index_(-1)
		, left_chain_size_(0)
		, recheck_flag_(false)
		, row_(51)
		, col_(51)
		, resolution_(0.05)
		, min_x_(0)
		, min_y_(0)
		, max_x_(0)
		, max_y_(0)
	{
		chain_list_.clear();
		check_chain_.clear();
		map_.clear();
		//row_ = grid_map_.MAX_WIDTH;
		//col_ = grid_map_.MAX_HEIGHT;
	}

	~CChainExtraction()
	{
	}
	/*
	 * Clear map_ and chain_list_
	 */
	void clearCChain();
	/*
	 *  Add pose to Chain list & update Chain Index
	 *  return true if check loop close, else return false
	 */
	bool generateChain(pose_s &pose);

	/*
	*   Add Pose To Chain List, if Chain list empty , initial the map_
	*   1-If Pose Cell Same as last one,Not Add
	*   2-If Robot Out Of Map, Expend Map, Update Value of Chain List Index from input index to last one(Value in Map)
	*   3-Add Pose To Deque
	*   This function add pose but not set value, except when index out of map,
	*   it will expent map, and change all index value in map
	*/

	void addPoseToChainList(pose_s &pose);
	/*
	 * Update Chain Index, if check loop close ,
	 * return true,alse return false
	 *
	 */
	bool updateChain();

	/*
	 * Check If Map Obstacle Loop Close and Set Value for New add index
	 *
	 */
	bool updateChainAndMap(int father_number, int index);

	/*
	 * Update cell value from current Chain List index to last Chain List index
	 *
	 */
	void drawChainToMap(int index);

	/*
	 * Get Father Cell Index of Some Index
	 * Search 8-Neighbour of Index, Get The Smallest Index, Return Its Value
	 * This Value Correspond to Index in Chain list
	 */
	int getFatherCell(int index);

	/*
	* add for detect real chain
	*
	*/
	bool recheckChain();

	void clearChainList();

	bool isRealChain(const pose_s &pose);

	// For the chain
	bool detectSectionChain();

	void findChainBounds(int &min_x, int &min_y, int &max_x, int &max_y);

	void findChainBounds(int &min_x, int &min_y, int &max_x, int &max_y, vector<CChainCell> &chain);

	void getPoseBounds(float &min_x, float &min_y, float &max_x, float &max_y);

	void addUniqueElement(pose_s &pose);

	void addUniqueElement(CChainCell cell);

	/*
	* Add Pose Cell To Deque
	* 1-If List Empty, Just Add Pose Inpute
	* 2-Not Empty, Add Cell Between Last Pose And Current Pose
	*/
	void addChainCellDeque(pose_s &pose);

	// For the map
	int x2idx(const float x);

	int y2idx(const float y);

	CChainCell xy2idx(pose_s &pose);

	double idx2x(const int idx);

	double idx2y(const int idy);

	bool isOutOfBounds(int idx, int idy);

	bool isPoseOutOfBounds(pose_s &pose);

	void setCell(const int idx, const int idy, const int value);

	int getCellValue(const int idx, const int idy);

	void expandMap();

	void resizeMap(int row, int col, float resolution);

	/*
	 * Initial the map, size equal to costmap, value set to "-1"
	 */
	void initialMap();

	int getChainSize();
	
	double distance2D(pose_s a,pose_s b);

	// For the chain
	int last_chain_size_;
	vector<CChainCell> chain_list_; //save all follow CChainCell
	vector<CChainCell> check_chain_;

	int recheck_count_;
	int recheck_index_;   // recheck this loop from this index
	int left_chain_size_; // left element in list
	bool recheck_flag_;

	// For the map to save follow point,
	int row_;
	int col_;

	pose_s standard_pose_; // start follow wall pose

	float resolution_;
	float min_x_;
	float min_y_;
	float max_x_;
	float max_y_;

	std::vector<int> map_;  //save value of CChainCell
	//occupancy_grid_s grid_map_;
};
}
