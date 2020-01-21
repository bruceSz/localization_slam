#include <cstdio>
#include <iostream>
#include <chrono>

using std::cout;
using std::endl;


class TicToc
{
  public:
    TicToc()
    {
        tic();
    }

    void tic()
    {
        start = std::chrono::system_clock::now();
    }

    double toc()
    {
        end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;
        return elapsed_seconds.count() * 1000;
    }

  private:
    std::chrono::time_point<std::chrono::system_clock> start, end;
};

void singleHello() {
    #pragma omp parallel
    {
        cout << "Hello world" << endl;
    }
}

void forPrint() {
    TicToc tk;
    int size=1000;
    double t[size];
    //#pragma omp parallel
    for(int i=0;i< size;i++) {
        //cout << i << endl;
        t[i] = 0.1;
    }
    cout << "raw time cost copy: " << tk.toc() << endl;
    
    TicToc tk2;
    #pragma omp parallel
    for(int i=0;i< size;i++) {
        //cout << i << endl;
        t[i] = 0.1;
    }
    cout << "omp parallel time cost copy: " << tk2.toc() << endl;

    TicToc tk3;
    #pragma omp simd
    for(int i=0;i< size;i++) {
        //cout << i << endl;
        t[i] = 0.1;
    }
    cout << "simd: time cost copy: " << tk3.toc() << endl;

}
int main() {
    //singleHello();
    forPrint();
}