#include "YarpGazeboControlBoard.hpp"

#include <cstddef>

#include <algorithm>
#include <string>
#include <vector>

#include <yarp/os/LogStream.h>
#include <yarp/os/Value.h>

#include <boost/smart_ptr/shared_ptr.hpp>

#include "LogComponent.hpp"

using namespace roboticslab;

constexpr auto DEFAULT_GEN_REF_SPEED = 7.5;
constexpr auto NOT_SET = -1;


bool YarpGazeboControlBoard::open(yarp::os::Searchable& config)
{
    return true;
}

bool YarpGazeboControlBoard::close()
{
    return true;
}
