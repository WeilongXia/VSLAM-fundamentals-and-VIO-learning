#include <iostream>
#include <vector>
#include <algorithm>

using namespace std;

class A
{
public:
    A(const int& i) : index(i) {}
    int index = 0;
};

int main()
{
    A a1(7), a2(5), a3(9);
    vector<A> avec{a1, a2, a3};
    std::sort(avec.begin(), avec.end(), [](const A&a1, const A&a2){return a1.index < a2.index;});
    for(auto& a: avec) cout << a.index << " ";
    cout << endl;
    return 0;
}