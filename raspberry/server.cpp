#include <iostream>
#include <boost/bind.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/asio.hpp>
#include <boost/shared_ptr.hpp>

using namespace boost; 
using namespace boost::asio; 
using ip::tcp;

class tcp_conn : public enable_shared_from_this<tcp_conn> {
  private:  
     	tcp::socket sock_;
     	std::string msg_;
     	tcp_conn(io_service& io) : sock_(io) {}
  
      void handle_write(const boost::system::error_code& error,
        size_t bytes_transferred)
      {
        // !! Here we handle the client requests !!
        //std::cout << "Ready for client requests?";
      }

  public: 
    static shared_ptr<tcp_conn> create(io_service& io) {
      return shared_ptr<tcp_conn>(new tcp_conn(io));
    }

    tcp::socket& socket() {return sock_;}

    void start() {
      msg_ = "Hello World\n";
      async_write(sock_,buffer(msg_),
      		boost::bind(&tcp_conn::handle_write, shared_from_this(),
            boost::asio::placeholders::error,
            boost::asio::placeholders::bytes_transferred)); 
    }
};

class tcp_server {
  private:  
      tcp::acceptor acceptor_;
  public:  
      // when calling tcp_server constructor
      // it instantiates the class acceptor to listen on TCP port 10000
      // and runs start_accept
      tcp_server(io_service& io)
       : acceptor_(io, tcp::endpoint(tcp::v4(), 10000))  {
          start_accept();
       }
  private:  
    void start_accept() {
      // gets the shared pointer 
      shared_ptr<tcp_conn> new_tcp_conn =
        tcp_conn::create(acceptor_.get_io_service());
      // creates a socket and initiates an async accept
      // when its tcp_connected handle_accept is triggered
      acceptor_.async_accept(new_tcp_conn->socket(),
        boost::bind(&tcp_server::handle_accept, this, new_tcp_conn));
     }
    void handle_accept(shared_ptr<tcp_conn> new_tcp_conn)  {
      new_tcp_conn->start();

      //std::cout << "Handling client connection!";

      // gets ready for next tcp_connection
      start_accept();
     }
};

int main()  {
    // read this http://think-async.com/Asio/asio-1.4.1/doc/asio/overview/core/basics.html
    io_service io; // IO service linked to the OS
    tcp_server server(io);
    io.run(); //
}
