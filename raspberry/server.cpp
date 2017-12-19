#include "resolve_get.h"
using namespace boost; 
using namespace boost::asio; 
using ip::tcp;

std::vector<Data> inoData;
//bool stream_on;

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
          os.str(std::string());
    		  boost::asio::async_read_until(sock_, input_buffer_, '\n',
            	boost::bind(&conn::handle_request, shared_from_this(), _1));
            
        }

        void handle_request(const boost::system::error_code& ec) {
		
          char c1;
          if (!ec) {

            std::string line;
            std::istream is(&input_buffer_);
            std::getline(is, line);
            int data = 0;

            if (!line.empty()) {
              std::cout << "Received: " << line << "\n";
              c1 = line.at(0);

              // restart
              if(c1 == 114) {
                data = 2*N_inos+1;
                os << data;
                for(int i=0;i<N_inos;++i){
                  inoData[i].SetRestartTime();
                }
                async_write(sp, buffer(os.str()), boost::bind(&conn::ack_command, shared_from_this(), _1));
              }
              // set
              if(c1 == 115) {
                int space = line.find_last_of(" ");
                std::string c2 = line.substr(2, space-2);
                // turn on/off consensus
                if(!c2.compare("c")) {
                  if (line.at(space+1) == '0'){  
                    data = 2*N_inos+2;
                  }
                  else if (line.at(space+1) == '1'){
                    data = 2*N_inos+3;
                  }
                }
                // spin window
                else if(!c2.compare("w")) {
                  if (line.at(space+1) == 0){  
                    data = 2*N_inos+4;
                  }
                  else if (line.at(space+1) == 90){
                    data = 2*N_inos+5;
                  }
                  else if(line.at(space+1) == 180) {
                    data = 2*N_inos+6;
                  }
                }
                else {
                  if (line.at(space+1) == '0'){  
                    data = (stoi(c2)-1)*2+1;
                  }
                  else if (line.at(space+1) == '1'){
                    data = (stoi(c2)-1)*2+2;
                  }
                }
                os << data;
                async_write(sp, buffer(os.str()), boost::bind(&conn::ack_command, shared_from_this(), _1));                   		
              }
              // get
              if(c1 == 103) {
                std::string comm = line.substr(2);
                std::string ret = resolve_get(comm);
                async_write(sock_, buffer(ret), boost::bind(&conn::handle_client, shared_from_this()));  
              }
              //get last minute buffer
              if(c1 == 98) {
                std::string comm=line.substr(2);
                std::string ret = resolve_buffer(comm);
                async_write(sock_, buffer(ret), boost::bind(&conn::handle_client, shared_from_this()));
              }
              if(c1 == 99) {
                /*stream_on = 1;
                if (line.at(2) == 108){

                }
                if (line.at(2) == 100){
                  
                }*/
              }                   	         
              if(c1 == 100) {

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
        		async_write(sock_, buffer("ack\n"), boost::bind(&conn::handle_client, shared_from_this()));	
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

class sniff {
  private:
    boost::asio::posix::stream_descriptor fifo;
    boost::asio::streambuf buffer;
    std::chrono::steady_clock::time_point begin, end; 
    int index, index_prev, val_a;
    int timestamp;
    float val_d, val_l, val_ref, val_ie;
    bool val_o;
    std::string line;
  public:
    sniff(io_service& io, int fifo_d)
    : fifo(io, fifo_d) {
      begin = std::chrono::steady_clock::now();
      for(int i=0; i<N_inos; i++) {
        inoData.push_back(Data(i+1));
      }
      start_sniff();
    }
  private:
    void start_sniff() {
      boost::asio::async_read_until(fifo, buffer, '\n',
              boost::bind(&sniff::assign_vec, this, _1));
    }
    void assign_vec(const boost::system::error_code& ec) {
      if(!ec) {
        std::istream is(&buffer);
        std::getline(is, line);
        end = std::chrono::steady_clock::now();
        timestamp = (int)std::chrono::duration_cast<std::chrono::seconds>(end - begin).count();

        // a
        if(line.at(1) == 97 && line.at(2) != 0) {

          index = line.find("l");
          if(index != std::string::npos) {
            val_a = std::stoi(line.substr(2, index-1));
            index_prev = index;
          }

          index = line.find("d");
          if(index != std::string::npos) {
            val_l = (float)std::stoi(line.substr(index_prev+1, index-index_prev-1));
            index_prev = index;
          }

          index = line.find("o");
          if(index != std::string::npos) {
            val_d = (float)std::stoi(line.substr(index_prev+1, index-index_prev-1));
            val_o = (bool)std::stoi(line.substr(index+1, 1));
            inoData[val_a-1].StoreNewData(timestamp, val_l, val_d, val_o);
            //std::cout << inoData[val_a-1].GetIlluminance() << "\n";
          }
        }
        // informçao do consensus/calibracao
      	else if(line.at(1) == 105 && line.at(2) != 0) {
      	  index = line.find("t");
          if(index != std::string::npos) {
            val_a = std::stoi(line.substr(2, index-1)) ;
            index_prev = index;
          }

      	  index = line.find("x");
          if(index != std::string::npos) {
            val_ref = (float)std::stoi(line.substr(index_prev+1, index-index_prev-1));
      	    val_ie = (float)std::stoi(line.substr(index+1));
            std::cout << val_ref << " " << val_ie << std::endl;
        	  inoData[val_a-1].SetExternalIlluminance(val_ie);
        	  inoData[val_a-1].SetReference(val_ref);
          }  	  
      	}
      }
      start_sniff();
    }
};


int main() {
  io_service io_main, io_sniff, io_stream;

  // Sniffing thread
  std::thread thread_sniff {[&io_sniff](){
    try {
      int fifo_d = open("myfifo", O_RDONLY);
      sniff sniffer(io_sniff, fifo_d);
      io_sniff.run();
    }
    catch (std::exception& e) {
      std::cout << "Exception sniff thread: " << e.what() << std::endl;
    }
  }};

  // Main thread  
  try	{
    	tcp_server server(io_main);
	    io_main.run();
	}
	catch (std::exception& e) {
	    std::cout << "Exception main thread: " << e.what() << std::endl;
	}
 
  thread_sniff.join(); 
}
