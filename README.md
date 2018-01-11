#  Distributed Control of Real-Time Systems (SCDTR)

This project was developed within the [Distributed Control of Real-Time Systems course at IST](https://fenix.tecnico.ulisboa.pt/cursos/meec/disciplina-curricular/1529008375879). And has 3 contributors:
  
  - [Carlos Aleluia](https://www.linkedin.com/in/carlos-aleluia-tavares-ba2426150/)
  - Filipe Madeira
  - [Mariana Martins](http://web.ist.utl.pt/ist180856/)

We made a [cringy video](https://youtu.be/THZRVS4BNpQ) showing it working.

You can check its goal in [here](https://github.com/Mrrvm/SCDTR/blob/master/project_goal.pdf). Or read the following summary.

**SUMMARY**

Given a lighting system, the objective is to distributedly control it so it maximizes the user confort and minimizes the spent energy. Here the [consensus algorithm](https://github.com/Mrrvm/SCDTR/blob/master/papers/consensus.pdf) is applied to the control system to define its reference based on a network of agents. All this agents contribute to find the optimal solution or to determine infeasibility. A TCP/IP server is used to get statistics out of the network and send them to clients.

The optimization of a lighting system is a currently studied problem, you can check this references for further insight [[1]], [[2]], [[3]], [[4]]. To learn more about consensus, check [this](https://web.stanford.edu/~boyd/papers/pdf/admm_distr_stats.pdf).

As a lighting system, we are using 2 luminaires. Each luminaire is composed by a LED, a fotoresistor, an arduino (used as an agent of the network) and other necessary electronics like resistors and capacitors.

This agents are connected by I2C protocol used as multi-master, so they can communicate the results of the consensus with each other. 
This communication is sniffed by a Raspberry Pi, that hosts a C++ server to get statistics out of the system and send them to requesting clients. 

The client can also interact with the system by restaring it or setting occupancies, this is done via serial (USART converted from/to USB) connected between the raspberry Pi and one of the agents (one of the arduinos).

The network is completely scalable.

[1]: https://fenix.tecnico.ulisboa.pt/downloadFile/3779579952577/DistributedIlluminationControlWithLocalSensing.pdf
[2]: https://pure.tue.nl/ws/files/3799294/774336.pdf
[3]: https://fenix.tecnico.ulisboa.pt/downloadFile/3779579952576/1-s2.0-S0378778810004597-main.pdf
[4]: http://journals.sagepub.com/doi/abs/10.1177/1477153510374703
