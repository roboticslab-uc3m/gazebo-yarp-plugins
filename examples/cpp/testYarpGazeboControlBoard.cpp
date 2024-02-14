#include <stdio.h>

#include <yarp/os/Network.h>
#include <yarp/dev/all.h>

int main(int argc, char *argv[]) {

    yarp::os::Network yarp;
    if ( ! yarp::os::Network::checkNetwork() )
    {
        printf("Please start a yarp name server first\n");
        return 1;
    }    

    yarp::os::Property options;
    options.put("device","YarpGazeboControlBoard");

    yarp::dev::PolyDriver dd(options);
    if( ! dd.isValid() )
    {
        printf("Device not available.\n");
        dd.close();
        yarp::os::Network::fini();
        return 1;
    }
    else
    {
        printf("Device available.\n");
    }

    dd.close();

    return 0;

}