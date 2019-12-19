#include "kdl_class.h"


#include <stdio.h>
#include <stdlib.h>
#include <fstream>

int main() {
    int Seed = time(NULL);
	srand( Seed );
	cout << "Seed in testing: " << Seed << endl;

    int n = 4;

    kdl fourBar(n, 1.5);

    std::ofstream f;
	f.open("samples.txt");//, ios::app);
    for (int i=0; i < 500000; i++) {

        State q = fourBar.sample_q();

        fourBar.printVector(q);
    
        for (int j = 0; j < q.size(); j++)
            f << q[j] << " ";
        f << "\n";
    }
    f.close();
    
}
