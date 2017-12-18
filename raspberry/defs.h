#include <iostream>
#include <thread>
#include <boost/bind.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/asio.hpp>
#include <boost/shared_ptr.hpp>
#include <condition_variable>
#include <mutex>
#include <string>
#include <sstream>
#include <vector>
#include <chrono>
#include <cstdlib>
#include "data.h"

#define N_inos 2
