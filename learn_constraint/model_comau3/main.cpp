#include "kdl_class_comau3.h"


#include <stdio.h>
#include <stdlib.h>
#include <fstream>

int main() {
    int Seed = time(NULL);
	srand( Seed );
	cout << "Seed in testing: " << Seed << endl;

    int n = 4;

    kdl D();

    std::ofstream f;
	f.open("samples_comau3.txt");//, ios::app);
    for (int i=0; i < 50000; i++) {

        State q = D.sample_q();

        // D.printVector(q);
    
        f << i << " ";
        for (int j = 0; j < q.size(); j++)
            f << q[j] << " ";
        f << "\n";
    }
    f.close();
    
}
