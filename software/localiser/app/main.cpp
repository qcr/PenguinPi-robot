#include <iostream>
#include <unistd.h>
#include <stdint.h>

#include <penguinpi/localiser.h>

#define STREAM_PORT     ("udp://0.0.0.0:5000")

using namespace std;

int main(int argc, char * argv[]){

    PenguinPi::Localiser localiser();

    cout << localiser << endl;

    // TODO connect to web server

    while(1){    

        break;
    }
    return 0;
}


