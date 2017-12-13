#include <iostream>
#include <thread>
#include <boost/bind.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/asio.hpp>
#include <boost/shared_ptr.hpp>
#include <condition_variable>
#include <mutex>

using namespace boost; 
using namespace boost::asio; 
using ip::tcp;

std::mutex mtx;
std::condition_variable cv_serial;

float data = 0;
bool data_ready = 0;
bool data_available() {return data_ready!=0;}

class conn : public enable_shared_from_this<conn> {
    private:  
        tcp::socket sock_;
        std::string msg_;
        boost::asio::streambuf input_buffer_;

        conn(io_service& io) :  sock_(io)  {}
        
        void handle_client()   {
            // maybe use a deadline here???? (check dat)

            // Start an asynchronous operation to read a newline-delimited message.
            boost::asio::async_read_until(sock_, input_buffer_, '\n',
                boost::bind(&conn::handle_request, shared_from_this(), _1));
        }

        void handle_request(const boost::system::error_code& ec) {
            if (!ec) {
                // Extract the newline-delimited message from the buffer.
                std::string line;
                std::istream is(&input_buffer_);
                std::getline(is, line);

                // Empty messages are heartbeats and so ignored.
                if (!line.empty()) {
                    std::cout << "Received: " << line << "\n";
                    if(line == "set") {
                        std::unique_lock < std::mutex > lck(mtx); 
                        data = 1; 
                        data_ready = 1; 
                        cv_serial.notify_one(); 
                    }

                    // Decide what to do with the request            
                    respond_client();
                }
                else {
                    handle_client();
                }
            }
            else {
                std::cout << "Error on receive: " << ec.message() << "\n";
                // handle this!!!!!
            }
        }
        void respond_client() {
            boost::asio::async_write(sock_, buffer("Hello!!\n"),
                boost::bind(&conn::handle_client, shared_from_this()));
        }
    public: 
        static shared_ptr<conn> create(io_service& io) {
            return shared_ptr<conn>(new conn(io));
        }
        tcp::socket& socket() {return sock_;}
        void start() {
            // Receives client requests
            std::cout << "Waiting requests from client" << std::endl;
            async_write(sock_,buffer("Ready for client\n"),
              boost::bind(&conn::handle_client, shared_from_this())); 
        }
};

class tcp_server {
    private:  
        tcp::acceptor acceptor_;
    public:  
        tcp_server(io_service& io)
        : acceptor_(io, tcp::endpoint(tcp::v4(), 10000))  {
            start_accept();
        }
    private:  
        void start_accept() {
            shared_ptr<conn> new_conn =  conn::create(acceptor_.get_io_service());
            acceptor_.async_accept(new_conn->socket(),
               boost::bind(&tcp_server::handle_accept, this, new_conn));
        }
        void handle_accept(shared_ptr<conn> new_conn)  {
            new_conn->start();
            start_accept();
        }
};

class serialClass {
    private:
        serial_port sp;
        deadline_timer tim;
        int counter;
        std::ostringstream os;
    public:
        serialClass(io_service& io) 
        : sp(io), tim(io) {
            counter = 0;
            sp.open("/dev/ttyACM0");    
            sp.set_option(serial_port_base::baud_rate(9600));
            request_handler();
        }

    private:
        void request_handler() {
            std::unique_lock < std::mutex > lck(mtx);
            cv_serial.wait(lck, data_available);
            std::cout << "received data: " << data << "\n";
            async_write(sp, buffer(os.str()), boost::bind(&serialClass::request_handler, this));
            data_ready = 0;
        }
};

int main() {
    io_service io_main, io_sniff, io_serial;

    // Sniffing thread
    std::thread thread_sniff {[&io_sniff](){
        io_sniff.run();
    }};

    // Serial communication thread
    std::thread thread_serial {[&io_serial](){
        serialClass serial(io_serial);
        io_serial.run();
    }};

    // Main thread  
    tcp_server server(io_main);
    io_main.run();

    thread_sniff.join(); 
    thread_serial.join(); 
}
