#include <iostream>

using std::cout ;
using std::endl;

int main(int argc, char** argv) {
    if (argc != 2) {
        cout << "Usage: " << argv[0] << " bal_data.txt" << endl;
        return 1;
    }

    BALProblem bal(argv[1]);
    bal.Normalize();
    bal.Perturb(0.1, 0.5, 0.5);
    bal.WriteToPLYFile("initial.ply");
    SolveBA(bal);
    bal.WriteToPLYFile("final.ply");
    return 0;
}