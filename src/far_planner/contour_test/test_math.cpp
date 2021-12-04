#include <iostream>
#include <math.h>       /* acos */
#include <unordered_set>
#include <set>
#include <bitset>         // std::bitset
#include <vector>

#include "time_measure.h"
#include "intersection.h"

static TimeMeasure TIMER;

using namespace std;


typedef std::bitset<2> BitVoxel;
typedef std::pair<size_t, size_t> PAIR_IDX;


int main() {
    const int N = 1e7;
    const BitVoxel temp_bitvoxel;
    TIMER.start_time();
    std::vector<char> processed_list(N, (char)0);
    TIMER.end_time("Initialize Time");
    TIMER.start_time();
    const char ibit = char(0);
    const char bit0 = char(1);
    const char bit1 = char(2);
    const char c_both = char(3);
    std::fill(processed_list.begin(), processed_list.end(), c_both);
    // for (int i=0; i<N; i++) {
    //     processed_list[i] = ibit;
    //     // processed_list[i] = processed_list[i] | bit0;
    //     // const int ind = i % 2;
    //     // if (ind == 0) {
    //     //     processed_list[i] = processed_list[i] | bit1;
    //     // }
    // }
    TIMER.end_time("Extraction Pairs");
    int both = 0;
    for (int i=0; i<N; i++) {
        if (processed_list[i] == c_both) both ++;
    }
    // connect_pairs.resize(idx_c);
    std::cout<<"Size of both: "<<both<<endl;
    // for (const auto& pair : connect_pairs) {
    //     std::cout<<pair.first<<", "<<pair.second<<endl;
    // }
}