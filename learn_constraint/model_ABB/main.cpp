#include "kdl_class.h"


#include <stdio.h>
#include <stdlib.h>
#include <fstream>

int main() {
    int Seed = time(NULL);
	srand( Seed );
	cout << "Seed in testing: " << Seed << endl;

    int n = 4;

    kdl abb(900.0, 300.0);

    std::ofstream f;
	f.open("samples.txt");//, ios::app);
    for (int i=0; i < 500000; i++) {

        State q = abb.sample_q();

        // abb.printVector(q);
    
        f << i << " ";
        for (int j = 0; j < q.size(); j++)
            f << q[j] << " ";
        f << "\n";
    }
    f.close();
    
}
