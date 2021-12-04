#include <iostream>
#include <math.h>       /* acos */
#include <vector>
#include <queue>

#include "time_measure.h"

static TimeMeasure TIMER;

using namespace std;

struct Node {
    float id;
    float fscore;
};

struct node_fcomp
{
  bool operator()(const Node& n1, const Node& n2) const
  {
    return n1.fscore > n2.fscore;
  }
};

int main() {
    std::priority_queue<Node, std::vector<Node>, node_fcomp> open_set;
    const int N = 1e3;
    for (int i=0; i<N; i++) {
        Node node;
        node.id = i, node.fscore = ((i+3) * 36) % 2000;
        open_set.push(node);
    }
    while(!open_set.empty()) {
        std::cout << (open_set.top()).fscore << ' ';
        open_set.pop();
    }
    std::cout << '\n';
}