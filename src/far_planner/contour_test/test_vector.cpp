#include <iostream>
#include <vector>


using namespace std;

void PrintVec(const vector<int>& vec) {
    cout<<"Vec: ";
    for (const auto& e : vec) {
        cout<<e<<", ";
    }
    cout<<"\n";
}

int main() {

    const int N  = 10;
    vector<int> vec;
    for (int i=0; i<N; i++) {
        vec.push_back(i);
    }
    PrintVec(vec);

    vec.erase(vec.begin(), vec.begin()+1);

    PrintVec(vec);

}


