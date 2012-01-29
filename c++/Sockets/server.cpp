/* A simple server in the internet domain using TCP
   The port number is passed as an argument */
#include <stdio.h>
#include <sys/types.h> 
#include <sys/socket.h>
#include <netinet/in.h>
#include <cstdlib>
#include <cstring> // includes bzero
#include <iterator>
#include <iostream>

int main(int argc, char *argv[])
{
     int sockfd, newsockfd, portno, clilen;
     char buffer[256];
     struct sockaddr_in serv_addr, cli_addr;
     int n;
     if (argc < 2) {
         fprintf(stderr,"ERROR, no port provided\n");
         exit(1);
     }
     sockfd = socket(AF_INET, SOCK_STREAM, 0);
     if (sockfd < 0)
     {
        std::cout << "ERROR opening socket" << std::endl;
        exit(-1);
     }
     bzero((char *) &serv_addr, sizeof(serv_addr));
     portno = atoi(argv[1]);
     serv_addr.sin_family = AF_INET;
     serv_addr.sin_addr.s_addr = INADDR_ANY;
     serv_addr.sin_port = htons(portno);
     if (bind(sockfd, (struct sockaddr *) &serv_addr,
              sizeof(serv_addr)) < 0)
     {
       std::cout << "ERROR on binding" << std::endl;
       exit(-1);
     }
     listen(sockfd,5);
     clilen = sizeof(cli_addr);
     newsockfd = accept(sockfd, 
                 (struct sockaddr *) &cli_addr, 
                 (socklen_t*)(&clilen));
     if (newsockfd < 0)
     {
          std::cout << "ERROR on accept" << std::endl;
          exit(-1);
     }
     bzero(buffer,256);
     n = read(newsockfd,buffer,255);
     if (n < 0)
     {
       std::cout << "ERROR reading from socket" << std::endl;
       exit(-1);
     }
     printf("Here is the message: %s\n",buffer);
     n = write(newsockfd,"I got your message",18);
     if (n < 0)
     {
       std::cout << "ERROR writing to socket" << std::endl;
       exit(-1);
     }
     return 0; 
}
