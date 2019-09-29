#include "param_manager.h"
#include <iostream>


using namespace std;

int main(int argc, char** argv)
{
    if(argc<2)
    {
        cout<<"usage: test_load_params calibration.txt"<<endl;
        return -1;
    }
    ParamManager manager(argv[1]);
    manager.Save("copy.txt");
}