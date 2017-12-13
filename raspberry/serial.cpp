//ASYNC_SERIAL.CPP
#include <iostream>
#include <string>
#include <sstream>
#include <boost/asio.hpp>
using namespace boost::system;
using namespace boost::asio;
//GLOBALS
io_service io;
serial_port sp(io);
deadline_timer tim(io);
int counter = 0;

//HANDLERS FOR ASYNC CALLBACKS
//forward declaration of write_handler to timer_handler 
void write_handler(const error_code &ec, size_t nbytes);
//timer_handler 
void timer_handler(const error_code &ec) {
   //timer expired – launch new write operation
   std::ostringstream os;
   os << "Counter = " << ++counter;
   async_write(sp, buffer(os.str()), write_handler);
}
void write_handler(const error_code &ec, size_t nbytes) {
    std::cout << counter << "\n";
   //writer done – program new deadline
    tim.expires_from_now(boost::posix_time::seconds(5));
    tim.async_wait(timer_handler);
}

int main() 
try{
    sp.open("/dev/ttyACM0");    //connect to port
    sp.set_option(serial_port_base::baud_rate(9600));
    //program timer for write operations
    tim.expires_from_now(boost::posix_time::seconds(2));
    tim.async_wait(timer_handler);
    io.run(); 
}
catch(std::exception &e){
    std::cout << e.what() << std::endl;
}