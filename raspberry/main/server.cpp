/** server.cpp 
 *
 *  Summary:     Generates 2 threads, the main thread receives requests from the client and sends the respective responses; 
 *               the other thread (sniff thread) receives the sniffed information from the FIFO and saves it.
 *  Last Edited: December 29, 2017
 *  Authors:     Mariana Martins (mrrvm@hotmail.com)
 *               Filipe Madeira  (filipe.s.c.madeira@gmail.com)
 *               Carlos Aleluia  (carlos.aleluia@tecnico.ulisboa.pt)
 *  License:     GNU General Public License v3.0
 *
 */

#include "resolve_get.h"
using namespace boost; 
using namespace boost::asio; 
using ip::tcp;

std::mutex mtx;
// shared vector between threads
std::vector<Data> inoData;


/** Class conn
 *  Gets client requests and sends the respective responses.
 *  Constructor:
 *         Instantiates a socket, a serial port and a timer;
 *         Opens the serial port and sets the baud rate at 115200;
 *         Initializes the stream variable.
 *  Private Methods:
 *         handle_client  : Reads what the client wrote to socket. 
 *                          Periodacally (5secs) checks if there is something to stream to client.
 *         handle_stream  : Checks per each arduino if there is something to stream, and streams it to client.
 *         handle_request : Decided the adequate response according to the request and responds.
 *         ack_command    : Sends acknowledges to client. 
 *  Public Methods:    
 *         create         : Returns a shared pointed with the connection.
 *         socket         : Returns the socket.
 *         start          : Outputs that the connection started and is ready for both the server and the client.
 */
class conn : public enable_shared_from_this<conn> {
    private:  
        tcp::socket sock_;
        boost::asio::steady_timer tim;
        boost::asio::streambuf input_buffer_;
        serial_port sp;
        std::ostringstream os;
        bool stream_on[N_inos][2]; // duty-cycle - 0, luminosity - 1

        conn(io_service& io) 
        :  sock_(io), sp(io), tim(io)  {
        	 sp.open("/dev/ttyACM0");    
           sp.set_option(serial_port_base::baud_rate(115200));
           for (int i = 0; i < N_inos; ++i) {
             stream_on[i][0] = 0;
             stream_on[i][1] = 0;
           }
        }
        
        void handle_client()   {

          tim.expires_from_now(std::chrono::milliseconds(5000));
          tim.async_wait(boost::bind(&conn::handle_stream, shared_from_this()));   
          
          os.str(std::string());
    		  boost::asio::async_read_until(sock_, input_buffer_, '\n',
            	boost::bind(&conn::handle_request, shared_from_this(), _1));
          
        }

        void handle_stream() {
          for(int i=0; i<N_inos; i++){
            if(stream_on[i][0] == 1){
              mtx.lock();
              os << "c d" << i << " " << inoData[i].GetDutyCycle() << " " << inoData[i].GetTimestamp() << "\n"; 
              mtx.unlock();
              async_write(sock_, buffer(os.str()), boost::bind(&conn::handle_client, shared_from_this())); 
            }
            else if(stream_on[i][1] == 1){
              mtx.lock();
              os << "c l" << i << " " << inoData[i].GetIlluminance() << " " << inoData[i].GetTimestamp() << "\n"; 
              mtx.unlock();
              async_write(sock_, buffer(os.str()), boost::bind(&conn::handle_client, shared_from_this())); 
            }
          }
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
                  mtx.lock();
                  inoData[i].SetRestartTime();
                  mtx.unlock();
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
                mtx.lock();
                std::string ret = resolve_get(comm);
                mtx.unlock();
                async_write(sock_, buffer(ret), boost::bind(&conn::handle_client, shared_from_this()));  
              }
              // get last minute buffer
              if(c1 == 98) {
                std::string comm = line.substr(2);
                mtx.lock();
                std::string ret = resolve_buffer(comm);
                mtx.unlock();
                async_write(sock_, buffer(ret), boost::bind(&conn::handle_client, shared_from_this()));
              }
              // stream on
              if(c1 == 99) {
                char var = line.at(2);
                int ino = stoi(line.substr(4));
                if(var == 100) {
                  stream_on[ino-1][0] = 1;
                }
                else {
                  stream_on[ino-1][1] = 1;
                } 
              }         
              // stream off          	         
              if(c1 == 100) {
                char var = line.at(2);
                int ino = stoi(line.substr(4));
                if(var == 100) {
                  stream_on[ino-1][0] = 0;
                }
                else {
                  stream_on[ino-1][1] = 0;
                }
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

/** Class tcp_server
 *  Accepts a connection and creates a session per client.
 *  Constructor:
 *         Instantiates an acceptor at localhost and port 10000;
 *         Runs start_accept().
 *  Private Methods:
 *         start_accept   : Creates a shared pointer for the connection and asynchronously accepts more connections.
 *         handle_accept  : Called when a connection is accepted, calls the method start from the conn class 
 *                          and goes back to start_accept.
 */
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

/** Class sniff
 *  Gets the information that was written to the fifo by the I2Csniffer.
 *  Constructor:
 *         Instantiates a fifo;
 *         Starts a clock;
 *         Initializes the vector inoData;
 *         Runs start_sniff.
 *  Private Methods:
 *         start_sniff    : Reads from the fifo until \n. 
 *         assign_vec     : Analyzes the fifo information and assigns it to the respective vector elements.
 */
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
      for(int j=0; j<N_inos; j++) {
        inoData.push_back(Data(j+1));
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

        // Analyzes a sequence from an arduino 
        // e.g. a1l100d50o1, arduino 1, luminosity 100, duty-cycle 50, occupancy 1
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
            mtx.lock();
            inoData[val_a-1].StoreNewData(timestamp, val_l, val_d, val_o);
            mtx.unlock();
          }
        }
        // Analyzes a sequence from the calibration/consensus
        // e.g. i1t50x40, arduino 1, reference 50, external illuminance 40
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
            std::cout << "Setup done\n";
            mtx.lock();
        	  inoData[val_a-1].SetExternalIlluminance(val_ie);
        	  inoData[val_a-1].SetReference(val_ref);
            mtx.unlock();
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
