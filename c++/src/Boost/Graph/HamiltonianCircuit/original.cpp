#include <iostream>
#include <stdlib.h>
#include <cmath>
#include <boost/graph/adjacency_list.hpp>

/**
* Hamiltonian Circuit source code
* Dec. 2007
*
* See http://alanhogan.com/asu/hamiltonian-circuit/
*
* Requires the Boost library. Set appropriate header include paths in your compiler or IDE.
*
* Copyright Alan Hogan, Dec. 2007. Contact: http://alanhogan.com/contact
*/

using namespace boost;

int main(int argc, char *argv[])
{
   typedef adjacency_list<vecS, vecS, undirectedS, disallow_parallel_edge_tag> Graph;
      //disallow_parallel_edge_tag is doing nothing for us
   
   //declare global variables
   Graph G1; //User's graph
   Graph G2; //Naïve circuit
   int graphSize; //Vertices of user's graph (and thus G2)
   bool found = false;
   int tempIterator = 0; int temp2 = 0;
   int a, b, c, d, pastC;
   int prev, cur, next;
   
   
   //Get stuff from user
   
   /* new */
   std::cout << "Please enter the number of vertices in your graph "
      << "(must be between 4 and 1024, inclusive): ";
   std::cin >> graphSize;
   if(graphSize < 4 || graphSize > 1024) { std::cout << "Invalid number. Exiting.\n"; return 1; }
   
   while(tempIterator > -1){
      std::cout << "Please enter starting vertex of an edge. Should be an int betweeen 1 and "
         << graphSize << ", inclusive. (-1 when finshed adding edges.): ";
      std::cin >> tempIterator;
      if(tempIterator > 0 && tempIterator <= graphSize) {
         std::cout << "Please enter ending vertex of the edge: " ;
         std::cin >> temp2;
         if(temp2 > 0 && temp2 <= graphSize && temp2 != tempIterator) {
            add_edge(tempIterator, temp2, G1);
            continue;
         }
      std::cout << "Invalid vertex...\n";
      }
   }
   /* end new */
   
   /* old: Fake data
   graphSize = 8;
   std::cout << "Please enter the number of vertices in your graph "
      << "(must be between 4 and 1024, inclusive): " << "--auto: 8--"<< std::endl;
   std::cout << "Please enter starting vertex of an edge: " << "--note: auto-building edges--"
      << std::endl;
   std::cout << "Please enter ending vertex of the edge: " << "--auto, for now---" << std::endl;
   add_edge(1, 4, G1);
   add_edge(1, 5, G1);
   add_edge(1, 8, G1);
   add_edge(1, 7, G1);
   add_edge(2, 7, G1);
   add_edge(3, 2, G1);
   add_edge(3, 6, G1);
   add_edge(4, 3, G1);
   add_edge(4, 5, G1);
   add_edge(4, 6, G1);   
   add_edge(5, 2, G1);
   add_edge(5, 3, G1);
   add_edge(5, 6, G1);   
   add_edge(6, 7, G1);
   add_edge(6, 8, G1);   
   add_edge(7, 8, G1);
   add_edge(8, 2, G1);
   // end old */
   
      
   //Validate number of edges
   for(int i = 1; i <= graphSize; i++) {
      tempIterator = 0; //here count of edges
      for(int j = 1; j <= graphSize; j++) {
         if (i == j) continue;
         if((edge(i,j,G1)).second) tempIterator++;
      }
      if(tempIterator < (int) std::ceil(0.5*graphSize)) {
         std::cout << "Sorry, vertex " << i << " did not have enough edges.\n";
         return 1;
      }
   }
   
   //Add every edge
   //Err, well, we modified the algorithm so as to just assume that. The circuit is represented in G2.
   
   //Construct naïve circuit "G2"
   for(int i = 1; i < graphSize; i++) {
      add_edge(i, i+1, G2);
   }
   add_edge(graphSize, 1, G2);
   
   //Subtract edges not in G1
   //find first edge: always (1,2) in G2
   prev = 0; cur = 1; next = 2;
   tempIterator = 0; //Number of edges examined and potentially fixed
   while(tempIterator < graphSize) {
      a = cur;
      b = next;
      //std::printf("Examining edge (%d, %d)...\n",a,b); //debug
      if((edge(a, b, G1)).second == false) { //Needs fixed!
         //std::printf("Edge (%d, %d) needs fixed\n",a,b); //debug
         pastC = b;
         for(int offset = 0; offset < (graphSize-1); offset++) {
            c = (pastC + offset) % graphSize + 1;
            //std::printf("offset: %d; c: %d.\n",offset,c); //debug
            //Is this a proper next element in our circuit? (Should really go by edges from vertex)
            if(c != a && c != b && c != prev && (edge(c, pastC, G2)).second) { //yes, it is!
               pastC = c; //If this does not work as a c, at least continue from it
               offset = -1;
               //std::printf("Next c in path (works; G2): %d.\n",c); //debug
            } else { //no
               continue;
            }
            
            //So is it ok to use?
            if((edge(a, c, G1)).second == false) {
               //std::printf("a & c were not neighbors in G1... Time for a new c\n"); //debug
               //No good!
               continue;
            } else {
               //std::printf("a & c ARE neighbors in G1 match up!\n"); //debug
               //Attempt to find d
               for(int offset2 = 0; offset2 < (graphSize-1); offset2++) {
                  d = (c + offset2) % graphSize + 1;
                  if(c != d && d != b && d != a && (edge(d, c, G2)).second) {
                     //This IS the only d possible
                     break;
                  }
               } //end for (finding d)
               if((edge(d, b, G1)).second) {
                  //we found our c & d!
                  break;
               }
            }//else (it was a good c)
         }//for (finding c)
         
         next = c;
         //remove (a,b) and (c,d); add (a,c) and (b,d).
         add_edge(a, c, G2);
         add_edge(b, d, G2);
         remove_edge(c, d, G2);
         remove_edge(a, b, G2);
      } else { //end fixing
         //std::printf("Edge (%d, %d) was fine!",a,b); //debug
      }
      
      //"Next" will remain "b" (no fixing), or has be changed to "c" (fixed).
      //For the next loop, "cur" needs to take on the value of
      //"next" and the new "next" needs chosen.
      prev = cur; cur = next;
      for(int offset = 0; offset < (graphSize-1); offset++) {
         next = (cur + offset) % graphSize + 1;
         //std::cout << "testing " << next << "\n"; //debug
         if(next != prev && (edge(cur, next, G2)).second) {
            //found ideal next
            break;
         }
      }
      tempIterator++;
   }//while tempIterator

   
   
   //Return path to user!
   std::cout << "\nComplete. Hamiltonian circuit has been found:" << std::endl;
   //find first edge
   found = false; tempIterator = 2;
   while(!found) {
      if((edge(1, tempIterator, G2)).second) {
         found = true;
         cur = 1;
         next = tempIterator;
         std::cout << "1, " << tempIterator;
      }
      tempIterator++;
      if(tempIterator > graphSize) {
         std::cout << "Ooops, problem... Nothing attaches to the first vertex...\n";
         return 1;
      }
   }
   //Then print all the other edges
   while (next != 1) {
      //std::cout << "\nprev: " << prev << "; cur: " << cur << "; next: " << next << std::endl; //debug
      prev = cur;
      cur = next;
      //std::cout << "prev: " << prev << "; cur: " << cur << "; next: " << next << std::endl; //debug
      for(int offset = 0; offset < (graphSize-1); offset++) {
         next = (cur + offset) % graphSize + 1;
         //std::cout << "testing " << next << "\n"; //debug
         if(next != prev && (edge(cur, next, G2)).second) {
            std::cout << ", " << next;
            break;
         }
      }
      //std::cout << "prev: " << prev << "; cur: " << cur << "; next: " << next << std::endl; //debug
      //for errors:
      if(cur == next) {
         std::cout << "Ooops, problem... Nothing attaches to vertex " << cur << " except " << prev << "\n";
         return 1;
      }
   }
   std::cout << ".\nProgram complete." << std::endl;
return 0;
}
