using namespace boost; 
using namespace boost::asio; 
using ip::tcp;

class conn : public enable_shared_from_this<conn> {
    private:  
        tcp::socket sock_;
        boost::asio::streambuf input_buffer_;
        serial_port sp;
        std::ostringstream os;

        conn(io_service& io) 
        :  sock_(io), sp(io)  {
        	sp.open("/dev/ttyACM0");    
            sp.set_option(serial_port_base::baud_rate(115200));
        }
        
        void handle_client()   {
            // maybe use a deadline here????

    		boost::asio::async_read_until(sock_, input_buffer_, '\n',
            	boost::bind(&conn::handle_request, shared_from_this(), _1));
            
        }

        void handle_request(const boost::system::error_code& ec) {
		
			char c;
            if (!ec) {
                
                std::string line;
                std::istream is(&input_buffer_);
                std::getline(is, line);
                int data;

                if (!line.empty()) {
                    std::cout << "Received: " << line << "\n";
                    c = line.at(0);
                    // restart
                   	if(c == 114) {
                   		data = 2*N_inos+1;
                   		os << data;
                   		std::cout << os.str() << "\n";
            			async_write(sp, buffer(os.str()), boost::bind(&conn::ack_command, shared_from_this(), _1));
                   	}
                   	// set
                   	if(c == 115) {
                   		if (line.at(5) == 0){
							data = (line.at(3)-1)*2+1;
						}
						else if (line.at(5) == 1){
							data = (line.at(3)-1)*2+2;
						}
                   		os << data << "\n";
            			async_write(sp, buffer(os.str()), boost::bind(&conn::ack_command, shared_from_this(), _1));                   		
                   	}
                   	// get
                   	if(c == 103) {
                   		resolve_get();
                   	}
                   	if(c == 98) {
                   		
                   	}
                   	if(c == 99) {
                   		
                   	}                   	         
                   	if(c == 100) {
                   		
                   	}                    
                }
                else {
                    handle_client();
                }
            }
            else {
                std::cout << "Error on receive: " << ec.message() << "\n";
            }
        }
        void ack_command(const boost::system::error_code& ec) {
        	if(!ec) {
        		async_write(sock_, buffer("ack"), boost::bind(&conn::handle_client, shared_from_this()));	
        	}
        	else {
        		std::cout << "Error on acknowledge: " << ec.message() << "\n";
        		exit(0);
        	}
        }

    public: 
        static shared_ptr<conn> create(io_service& io) {
            return shared_ptr<conn>(new conn(io));
        }

        tcp::socket& socket() {return sock_;}
        
        void start() {
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

int main() {
    io_service io_main, io_sniff;

    // Sniffing thread
    std::thread thread_sniff {[&io_sniff](){
        io_sniff.run();
    }};

    // Main thread  
    try	{
    	tcp_server server(io_main);
	    io_main.run();
	}
	catch ( std::exception& e ) {
	    std::cout << "Exception: " << e.what() << std::endl;
	}
    

    thread_sniff.join(); 
}
