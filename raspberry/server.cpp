#include <iostream>
#include <boost/bind.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/asio.hpp>
#include <boost/shared_ptr.hpp>

using namespace boost; 
using namespace boost::asio; 
using ip::tcp;

class conn : public enable_shared_from_this<conn> {
    private:  
        tcp::socket sock_;
        std::string msg_;
        boost::asio::streambuf input_buffer_;

        conn(io_service& io) :  sock_(io)  {}
        
        void handle_client()   {
            // Receives client requests
             std::cout << "Waiting requests from client" << std::endl;

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
                    // Execute the command with I2C sniffer
                }
                handle_client();
            }
            else {
                std::cout << "Error on receive: " << ec.message() << "\n";
                // handle this!!!!!
            }
        }
        void respond_client() {

        }
    public: 
        static shared_ptr<conn> create(io_service& io) {
            return shared_ptr<conn>(new conn(io));
        }
        tcp::socket& socket() {return sock_;}
        void start() {
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
            shared_ptr<conn> new_conn =
            conn::create(acceptor_.get_io_service());
            acceptor_.async_accept(new_conn->socket(),
               boost::bind(&tcp_server::handle_accept, this, new_conn));
        }
        void handle_accept(shared_ptr<conn> new_conn)  {
            new_conn->start();
            start_accept();
        }
};

int main() {
    io_service io;
    tcp_server server(io);
    io.run();
}
