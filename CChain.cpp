#include "CChain.h"
#include "limits.h"
#include <iostream>
#include <math.h>


namespace flight
{
/*
 *  clear map_ & chain_list_ & check_chain_
 */
void CChainExtraction::clearCChain()
{
	initialMap();
	chain_list_.clear();
	check_chain_.clear();
}

/*
 *  Add pose to Chain list & update Chain Index
 *  return true if check loop close, else return false
 */
bool CChainExtraction::generateChain(pose_s &pose)
{
	last_chain_size_ = chain_list_.size();
	std::cout  << "last_chain_size_:" << last_chain_size_ << std::endl;
	//-1- Add Pose To Chain
	addPoseToChainList(pose);
	//-2-
	int current_chain_size = chain_list_.size();
	
	std::cout << "recheck_index_:" << recheck_index_ << std::endl;

	// for recheck
	if (recheck_index_ >= 0 && current_chain_size > left_chain_size_) {
		std::cout << "CChain-Need-Recheck!" << std::endl;
		left_chain_size_ = current_chain_size;
		recheck_flag_ = true;

	} else if (recheck_index_ >= 0) {
		std::cout << "Recheck index:" << recheck_index_ << std::endl;
		std::cout << "But not meet loop condition Current size:" << current_chain_size << std::endl;
		std::cout << "Left size:" << left_chain_size_ << std::endl;
	}

	return updateChain();
}
/*
*   Add Pose To Chain List, if Chain list empty , initial the map_
*   1-If Pose Cell Same as last one,Not Add
*   2-If Robot Out Of Map, Expend Map, Update Value of Chain List Index from input index to last one(Value in Map)
*   3-Add Pose To Deque
*   This function add pose but not set value, except when index out of map,
*   it will expent map, and change all index value in map
*/
void CChainExtraction::addPoseToChainList(pose_s &pose)
{
	// PX4_WARN("add pose to chain list");
	if (chain_list_.empty()) {
		std::cout << "------chain list empty-------add first" << std::endl;
		standard_pose_ = pose;
		
		std::cout << "standard_pose_x:" << standard_pose_.x << std::endl;
		std::cout << "standard_pose_y:" << standard_pose_.y << std::endl;

		initialMap();
		addUniqueElement(pose);

	} else {
		if (isPoseOutOfBounds(pose)) {
			std::cout << "START to expand map " << std::endl;

			while (isPoseOutOfBounds(pose)) {
				// TODO: resize the map and update.
				std::cout << "Follow Out Of Map, Expend Map" << std::endl;
				expandMap();
			}

			drawChainToMap(0);
			// PX4_INFO("finish expand map ");
		}

		addChainCellDeque(pose);
	}
}
/*
 * Update Chain Index, if check loop close ,
 * return true,alse return false
 *
 */
bool CChainExtraction::updateChain()
{
	std::cout << "updateChain" << std::endl;
	// 1- Update Element Value(Index) In Chain List
	drawChainToMap(last_chain_size_);
	
	std::cout << "chain_list_size:" << chain_list_.size() << std::endl;
	// PX4_WARN("drawChainToMap finish");
	// 2- Check If Chain List Loop
	if (chain_list_.size() > 2) {
		std::cout << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%" << std::endl;
		int father_number = 0;
		int index = last_chain_size_; //set start index
		
		//std::cout  << "index:" << index << std::endl;
		// PX4_INFO(" Chain List Size [%d] - last chain size %d",
		//          last_chain_size_, chain_list_.size());
		while (index < chain_list_.size()) {
			father_number = getFatherCell(index);

			// PX4_INFO(" get father_number %d ", father_number);
			std::cout  << "index:" << index << std::endl;
			std::cout << "get father_number:" << father_number << std::endl;

			while ((father_number + 1 == index) && (++index < chain_list_.size())) {
				father_number = getFatherCell(index);
				
				std::cout  << "In_index:" << index << std::endl;
				std::cout << "In_get father_number:" << father_number << std::endl;
			}

			// last_chain_size_ = father_number + 2;
			// when Index >= chain_list_.size(), did not find father
			if (index >= chain_list_.size()) {
				std::cout << "index >= chain_list_.size()" << std::endl;
				break;
			}

			//PX4_INFO(" get father_number %d index %d", father_number, index);
			std::cout << "Close get father_number:" << father_number << std::endl;
			std::cout << "Close index:" << index << std::endl;
			std::cout << "Close chain list size:" << chain_list_.size() << std::endl;

			// check if close..
			if (updateChainAndMap(father_number, index)) {
				recheck_index_ = father_number;
				left_chain_size_ = chain_list_.size();  // if judge finish, size = 1
				//PX4_INFO(" Chain List Size [%d] - last chain size %d",
				//		 last_chain_size_, chain_list_.size());
				//PX4_WARN("Check Loop Close, Recheck Index = %d, left_chain_size_ = %d", recheck_index_, left_chain_size_);

				std::cout << "last chain size:" << last_chain_size_ << std::endl;
				std::cout << "Chain List Size:" << chain_list_.size() << std::endl;
				std::cout << "Check Loop Close, Recheck Index:" << recheck_index_ << std::endl;
				std::cout << "left_chain_size:" << left_chain_size_ << std::endl;
				
				return true;
			}

			// else
			// {
			//     PX4_WARN("updateChainAndMap Failed !!!");
			// }

			index = father_number + 2;
			// PX4_INFO("INDEX %d ", index);
		}
	}

	return false;
}

/*
 * Check If Map Obstacle Loop Close and Set Value for New add index
 *
 */
bool CChainExtraction::updateChainAndMap(int father_number, int index)
{
	// PX4_INFO("updateChainAndMap, index, father %d, %d, size %d", index, father_number, chain_list_.size());
	if (father_number < 0) {
		return false;
	}

	// 1- Finish Process: Judge if Loop Closs
	//    0 1 2
	//  11     3
	// 10       4
	//  9      5
	//    8 7 6
	if ((index - father_number) > 7) { //10
		std::cout << "index:" << index << "father_number:" << father_number << std::endl;
		check_chain_.clear();

		for (int i = father_number; i <= index; i++) {
			check_chain_.push_back(chain_list_[i]);
			int idx = chain_list_[i].idx_;
			int idy = chain_list_[i].idy_;
			setCell(idx, idy, -1);
		}

		// close condition..
		if (father_number == 0) {
			// 1-1 Full Follow Close
			//PX4_WARN("start closed");
			std::cout << "start closed " << std::endl;

			CChainCell last_cell = chain_list_[index];
			chain_list_.clear();
			chain_list_.push_back(last_cell);

		} else {
			// 1-2 Island Follow Close
			//PX4_WARN("middle closed, father number: %d", father_number);
			
			std::cout << "middle closed,father number:" << father_number << std::endl;
			std::vector<CChainCell> temp = chain_list_;
			chain_list_.clear();

			for (int i = 0; i < father_number; i++) {
				chain_list_.push_back(temp[i]);
			}
		}

		return true;
	}

	// 2-Normal Process: updtae chain_list_
	std::vector<CChainCell> temp = chain_list_;
	chain_list_.clear();

	for (int i = 0; i <= father_number; i++) {
		chain_list_.push_back(temp[i]);
	}

	// set father_number + 1 ---> Index to '-1' (default)
	for (int i = father_number + 1; i < index; i++) {
		int idx = temp[i].idx_;
		int idy = temp[i].idy_;
		setCell(idx, idy, -1);
	}

	for (int i = index; i < temp.size(); i++) {
		chain_list_.push_back(temp[i]);
	}

	// set father_number + 1 ---> Index to new value (default)
	for (int i = father_number + 1; i < chain_list_.size(); i++) {
		int idx = chain_list_[i].idx_;
		int idy = chain_list_[i].idy_;
		// PX4_WARN("%d / %d -get cell in update %d, %d", i, chain_list_.size(), idx, idy);
		int value = getCellValue(idx, idy);

		if (value == -2) { //-2 means out of map, update failed
			return false;

		} else if (value == -1) {
			value = i;

		} else {
			value = value < i ? value : i;
		}

		setCell(idx, idy, value);
	}

	return false;
}
/*
 * Update cell value from current Chain List index to last Chain List index
 *
 */
void CChainExtraction::drawChainToMap(int index)
{
	int cell_x, cell_y;

	// Update Chain Index
	for (int i = index; i < chain_list_.size(); i++) {
		cell_x = chain_list_[i].idx_;
		cell_y = chain_list_[i].idy_;

		// if (isOutOfBounds(cell_x, cell_y))
		// {
		//     PX4_WARN("wrong condition.... out of map when drawChainToMap");
		//     expandMap();
		// }
		// PX4_WARN("cell_x, cell_y: %d, %d", cell_x, cell_y);
		int value = getCellValue(cell_x, cell_y);

		//value
		// -2: out of map
		// -1: init value
		if (value == -2) {
			return;

		} else if (value == -1) {
			value = i;

		} else {
			value = value < i ? value : i;
		}

		setCell(cell_x, cell_y, value);
	}
}
/*
 * Get Father Cell Index of Some Index
 * Search 8-Neighbour of Index, Get The Smallest Index, Return Its Value
 * This Value Correspond to Index in Chain list
 */
int CChainExtraction::getFatherCell(int index)
{
	int father_number = -1;
	CChainCell search_cell = chain_list_[index];

	// search 24-neighbour
	for (int i = search_cell.idx_ - 1; i <= search_cell.idx_ + 1; i++) {
		for (int j = search_cell.idy_ - 1; j <= search_cell.idy_ + 1; j++) {
			if (isOutOfBounds(i, j)) {
				continue;
			}

			int mark_point = getCellValue(i, j);

			// PX4_WARN("cell value: %d, %d, - %d", i, j, mark_point);
			if (mark_point != -1) { // default value
				if (father_number == -1) {
					father_number = mark_point;

				} else { //find smaller index value
					father_number = father_number < mark_point ? father_number : mark_point;
				}
			}
		}
	}

	return father_number;
}

// add for detect real chain
bool CChainExtraction::recheckChain()
{
	std::cout << "recheckChain:" << !check_chain_.empty() << std::endl;
	return !check_chain_.empty();
}

void CChainExtraction::clearChainList()
{
	chain_list_.clear();
}

double CChainExtraction::distance2D(pose_s a,pose_s b){
	return sqrt((a.x-b.x)*(a.x-b.x) + (a.y-b.y)*(a.y-b.y));
}

bool CChainExtraction::isRealChain(const pose_s &pose)
{
	//PX4_INFO("enter real chain detect, %d, %d", recheck_index_, recheck_count_);
	std::cout << "enter real chain detect recheck_index_:" << recheck_index_ << std::endl;
	std::cout << "enter real chain detect recheck_count_:" << recheck_count_ << std::endl;

	if (check_chain_.empty()) {
		//PX4_INFO("Check Chain List Is Empty");
		std::cout << "Check Chain List Is Empty" << std::endl;
	}

	if (recheck_index_ < 0) {
		//PX4_ERR("wrong condition.... recheck_index_ = %d", recheck_index_);
		
		std::cout << "wrong condition.... recheck_index_:"  << recheck_index_ << std::endl;
		recheck_index_ = -1;
		recheck_count_ = 0;
		check_chain_.clear();
		return false;
	}

	if (!recheck_flag_) {
		//PX4_WARN("Recheck flag is false, loop not exist");
		std::cout << "Recheck flag is false, loop not exist" << std::endl;
		return false;
	}

	recheck_flag_ = false;

	// recheck_index_ is father_number, check_chain_ is from father_number to end
	double dis, distance_current, distance_left, distance_right;
	// 1-get dis: distance between current pose and father_number
	distance_current = distance2D(pose, check_chain_[recheck_index_].pose_);
	dis = distance_current;

	// 2-get near dis left: get neighour index of recheck_index_, compute this distance
	int index;

	if (recheck_index_ == 0) {
		index = check_chain_.size() - 1;

	} else {
		index = recheck_index_ - 1;
	}

	distance_left = distance2D(pose, check_chain_[index].pose_);
	dis = dis < distance_left ? dis : distance_left;

	// 3-get near dis Right: get neighour index of recheck_index_, compute this distance
	index = recheck_index_ + 1;

	if (index >= check_chain_.size()) {
		index %= check_chain_.size();
	}

	distance_right = distance2D(pose, check_chain_[index].pose_);
	dis = dis < distance_right ? dis : distance_right;

	//PX4_WARN("is real chain : dis = %f | c_l_r [%f, %f, %f]", dis, distance_current, distance_left, distance_right);
	
	std::cout << "is real chain dis :" << dis << std::endl;
	std::cout << "is real chain dis_c: :" << distance_current << std::endl;
	std::cout << "is real chain dis_l:" << distance_left << std::endl;
	std::cout << "is real chain dis_r:" << distance_right << std::endl;


	if (dis < resolution_ * 7) { // 0.1 * 2 = 0.2
		recheck_count_++;
		recheck_index_++;

		if (recheck_index_ >= check_chain_.size()) {
			recheck_index_ %= check_chain_.size();
		}

	} else {
		recheck_index_ = -1;
		recheck_count_ = 0;
		check_chain_.clear();
	}

	if (recheck_count_ > 2) {
		recheck_index_ = -1;
		recheck_count_ = 0;
		check_chain_.clear();
		return true;
	}

	return false;
}

// For the chain
bool CChainExtraction::detectSectionChain()
{
	int min_idx, min_idy, max_idx, max_idy;
	
	std::cout << "check_chain_size:" << check_chain_.size() << std::endl;
	for(int i = 0;i < check_chain_.size();i++){
		std::cout << "idx:" << check_chain_[i].idx_ << " idy:" << check_chain_[i].idy_ << std::endl;
	} 

	findChainBounds(min_idx, min_idy, max_idx, max_idy, check_chain_);
	
	std::cout << "min_idx:" << min_idx << std::endl;
	std::cout << "min_idy:" << min_idy << std::endl;
	
	std::cout << "max_idx:" << max_idx << std::endl;
	std::cout << "max_idy:" << max_idy << std::endl;

	// initial
	int threshold = 2;
	int left = -1, right = -1, up = -1, down = -1;
	int min_cell_idx = INT_MAX;
	int min_cell_idy = INT_MAX;
	int max_cell_idx = INT_MIN;
	int max_cell_idy = INT_MIN;

	if (max_idx - min_idx > threshold) {
		int middle = (min_idx + max_idx) / 2;

		// find le
		for (int i = 0; i < check_chain_.size(); i++) {
			int cell_x = check_chain_[i].idx_;
			int cell_y = check_chain_[i].idy_;

			//first left
			if (left == -1) {
				if (cell_x == min_idx) {
					left = i;
				}
			}

			//first right
			if (right == -1) {
				if (cell_x == max_idx) {
					right = i;
				}
			}

			if (middle == cell_x) {
				if (cell_y > max_cell_idy) {
					max_cell_idy = cell_y;
					up = i;
				}

				if (cell_y < min_cell_idy) {
					min_cell_idy = cell_y;
					down = i;
				}
			}
		}

	}else {
		int middle = (min_idy + max_idy) / 2;

		for (int i = 0; i < check_chain_.size(); i++) {
			int cell_x = check_chain_[i].idx_;
			int cell_y = check_chain_[i].idy_;

			if (down == -1) {
				if (cell_y == min_idy) {
					down = i;
				}
			}

			if (up == -1) {
				if (cell_y == max_idy) {
					up = i;
				}
			}

			if (middle == cell_y) {
				if (cell_x > max_cell_idx) {
					max_cell_idx = cell_x;
					right = i;
				}

				if (cell_x < min_cell_idx) {
					min_cell_idx = cell_x;
					left = i;
				}
			}
		}
	}

	int direc_count = 0;

	if (down - left > 0) {
		direc_count++;
	}

	if (right - down > 0) {
		direc_count++;
	}

	if (up - right > 0) {
		direc_count++;
	}

	if (left - up > 0) {
		direc_count++;
	}

	//PX4_INFO("direc_count:%d",direc_count);
	std::cout << "left:" << left << "right:" << right << "up:" << up << "down:" << down << "direc_count:" << direc_count << std::endl;

	if (direc_count == 3)
	{
		return true;

	} else {
		return false;
	}
}

void CChainExtraction::findChainBounds(int &min_x, int &min_y, int &max_x, int &max_y)
{
	findChainBounds(min_x, min_y, max_x, max_y, chain_list_);
}

void CChainExtraction::findChainBounds(int &min_x, int &min_y, int &max_x, int &max_y, vector<CChainCell> &chain)
{
	min_x = INT_MAX;
	min_y = INT_MAX;
	max_x = INT_MIN;
	max_y = INT_MIN;

	for (int i = 0; i < chain.size(); i++) {
		int cell_idx = chain[i].idx_;
		int cell_idy = chain[i].idy_;

		min_x = cell_idx < min_x ? cell_idx : min_x;
		min_y = cell_idy < min_y ? cell_idy : min_y;
		max_x = cell_idx > max_x ? cell_idx : max_x;
		max_y = cell_idy > max_y ? cell_idy : max_y;
	}
}

void CChainExtraction::getPoseBounds(float &min_x, float &min_y, float &max_x, float &max_y)
{
	min_x = 10e6;
	min_y = 10e6;
	max_x = -10e6;
	max_y = -10e6;

	for (int i = 0; i < chain_list_.size(); i++) {
		float cell_x = chain_list_[i].pose_.x;
		float cell_y = chain_list_[i].pose_.y;

		min_x = cell_x < min_x ? cell_x : min_x;
		min_y = cell_y < min_y ? cell_y : min_y;
		max_x = cell_x > max_x ? cell_x : max_x;
		max_y = cell_y > max_y ? cell_y : max_y;
	}
}

void CChainExtraction::addUniqueElement(pose_s &pose)
{
	std::cout << "addUniqueElement" << std::endl;
	int idx = x2idx(pose.x);
	int idy = y2idx(pose.y);
	std::cout << "idx:" << idx << "idy:" << idy << std::endl;
	CChainCell cell(idx, idy, pose);
	addUniqueElement(cell);
}

void CChainExtraction::addUniqueElement(CChainCell cell)
{
	if (chain_list_.empty()) {
		chain_list_.push_back(cell);
	}

	//Check if Cell Unique
	else {
		CChainCell last_cell = chain_list_.back();

		if (last_cell.idx_ != cell.idx_ || last_cell.idy_ != cell.idy_) {
			chain_list_.push_back(cell);
		}
	}
}
/*
* Add Pose Cell To Deque
* 1-If List Empty, Just Add Pose Inpute
* 2-Not Empty, Add Cell Between Last Pose And Current Pose
*/
void CChainExtraction::addChainCellDeque(pose_s &pose)
{
	std::cout << "addChainCellDeque" << std::endl;

	int idx = x2idx(pose.x);
	int idy = y2idx(pose.y);
	CChainCell cell(idx, idy, pose);

	if (chain_list_.empty()) {
		chain_list_.push_back(cell);

	} else {
		CChainCell last_cell = chain_list_.back();
		// Add the cell deque
		float distance = distance2D(last_cell.pose_, pose);
		
		std::cout << "distance:" << distance << std::endl;

		float setp = resolution_ / 4;

		for (float i = 0; i < distance; i += setp) {
			float x = ((distance - i) * last_cell.pose_.x + i * pose.x) / distance;
			float y = ((distance - i) * last_cell.pose_.y + i * pose.y) / distance;
			float phi = ((distance - i) * last_cell.pose_.theta + i * pose.theta) / distance;

			pose_s temp = pose;
			temp.x = x;
			temp.y = y;
			temp.theta = phi;

			CChainCell cell = xy2idx(temp);
			
			std::cout << "temp.x:" << temp.x << " temp.y:" << temp.y << "cell.idx:" << cell.idx_ << " cell.idy:" << cell.idy_ << std::endl;
			
			addUniqueElement(cell);
		}

		addUniqueElement(pose);
	}
}

// For the map
int CChainExtraction::x2idx(const float x)
{
	return floor((x - min_x_) / resolution_);
}
int CChainExtraction::y2idx(const float y)
{
	return floor((y - min_y_) / resolution_);
}
CChainCell CChainExtraction::xy2idx(pose_s &pose)
{
	int idx = x2idx(pose.x);
	int idy = y2idx(pose.y);
	CChainCell cell(idx, idy, pose);
	return cell;
}

double CChainExtraction::idx2x(const int idx)
{
	return (float)((min_x_ + (idx + 0.5f) * resolution_));
}
double CChainExtraction::idx2y(const int idy)
{
	return (float)((min_y_ + (idy + 0.5f) * resolution_));
}

bool CChainExtraction::isOutOfBounds(int idx, int idy)
{
	return (idx < 0 || idy < 0 || idx >= row_ || idy >= col_);
}

bool CChainExtraction::isPoseOutOfBounds(pose_s &pose)
{
	int idx = x2idx(pose.x);
	int idy = y2idx(pose.y);

	return isOutOfBounds(idx, idy);
}

void CChainExtraction::setCell(const int idx, const int idy, const int value)
{
	if (isOutOfBounds(idx, idy)) {
		//PX4_WARN("setCellValue err, out of bounds:-- idx, idy: %d,%d;;; row_, col_ = %d, %d", idx, idy, row_, col_);
		std::cout << "getCellValue err idx:" << idx << std::endl;
		std::cout << "getCellValue err idy:" << idy << std::endl;
		std::cout << "getCellValue err row_:" << row_ << std::endl;
		std::cout << "getCellValue err col_:" << col_ << std::endl;

	} else {
		map_[idx * col_ + idy] = value;
	}
}

int CChainExtraction::getCellValue(const int idx, const int idy)
{
	if (isOutOfBounds(idx, idy)) {
		//PX4_WARN("getCellValue err, out of bounds.. idx, idy: %d,%d;;; row_, col_ = %d, %d", idx, idy, row_, col_);
		std::cout << "getCellValue err idx:" << idx << std::endl;
		std::cout << "getCellValue err idy:" << idy << std::endl;
		std::cout << "getCellValue err row_:" << row_ << std::endl;
		std::cout << "getCellValue err col_:" << col_ << std::endl;

		return -2;
	}

	return map_[idx * col_ + idy];
}

void CChainExtraction::expandMap()
{
	resizeMap(row_ + 50, col_ + 50, resolution_);

	// offset the chain list
	for (int i = 0; i < chain_list_.size(); i++) {
		chain_list_[i].idx_ += 25;
		chain_list_[i].idy_ += 25;
	}

	for (int i = 0; i < check_chain_.size(); i++) {
		check_chain_[i].idx_ += 25;
		check_chain_[i].idy_ += 25;
	}
}

void CChainExtraction::resizeMap(int row, int col, float resolution)
{
	row_ = row;
	col_ = col;
	resolution_ = resolution;
	initialMap();
}
/*
 * Initial the map, size equal to costmap, value set to "-1"
 */
void CChainExtraction::initialMap()
{
	float delta_x = ((row_ - 1) / 2 + 0.5f) * resolution_;
	float delta_y = ((col_ - 1) / 2 + 0.5f) * resolution_;
	// PX4_WARN("standard pose x,y : %f, %f", standard_pose_.x, standard_pose_.y);
	min_x_ = (standard_pose_.x - delta_x);
	min_y_ = (standard_pose_.y - delta_y);
	max_x_ = row_ * resolution_ + min_x_;
	max_y_ = col_ * resolution_ + min_y_;
	
	//PX4_WARN("min_x_, min_y_, max_x_, max_y_, : %f, %f, %f, %f", min_x_, min_y_, max_x_, max_y_);
	std::cout << "min_x_:" << min_x_ << "min_y_:" << min_y_ << "max_x_:" << max_x_ << "max_y_:" << max_y_ << std::endl;  	
	map_.clear();
	map_.resize(row_ * col_, -1);
}

int CChainExtraction::getChainSize()
{
	return chain_list_.size();
}


}
