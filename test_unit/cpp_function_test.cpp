#include <vector>
#include <set>
#include <iostream>
#include <algorithm> 

// TODO: Add return format
void findVecDiffInt(std::vector<int> prev, std::vector<int> curr){
    std::set<int> set_prev(prev.begin(), prev.end());
    std::set<int> set_curr(curr.begin(), curr.end());

    std::set<int> diff;

    // 找出在 prev 中但不在 curr 中的元素
    std::set_difference(set_prev.begin(), set_prev.end(),
                        set_curr.begin(), set_curr.end(),
                        std::inserter(diff, diff.begin()));

    std::cout << "Elements in prev not in curr:" << std::endl;
    for (int elem : diff) {
        std::cout << elem << " ";
    }
    std::cout << std::endl;

    // 清空 diff 集合并找出在 curr 中但不在 prev 中的元素
    diff.clear();
    std::set_difference(set_curr.begin(), set_curr.end(),
                        set_prev.begin(), set_prev.end(),
                        std::inserter(diff, diff.begin()));

    std::cout << "Elements in curr not in prev:" << std::endl;
    for (int elem : diff) {
        std::cout << elem << " ";
    }
    std::cout << std::endl;
}

int main(){

    std::vector<int> prev, curr;
    // prev.push_back(4);
    // prev.push_back(2);
    // prev.push_back(0);
    // prev.push_back(1);
    // prev.push_back(3);

    curr.push_back(7);
    curr.push_back(4);
    curr.push_back(5);
    curr.push_back(6);
    curr.push_back(2);
    curr.push_back(3);

    findVecDiffInt(prev, curr);
    return 0;
}