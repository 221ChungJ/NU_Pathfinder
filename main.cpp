/*main.cpp*/

//
// Program to input Nodes (positions), Buildings and Footways
// from an Open Street Map file.
// 
// Prof. Joe Hummel
// Northwestern University
// CS 211: Winter 2023
// 

#include <iostream>
#include <string>
#include <iomanip>
#include <algorithm>
#include <limits>
#include <queue>
#include <stack>

#include "building.h"
#include "buildings.h"
#include "footway.h"
#include "footways.h"
#include "node.h"
#include "nodes.h"
#include "osm.h"
#include "tinyxml2.h"
#include "graph.h"
#include "dist.h"

using namespace std;
using namespace tinyxml2;

constexpr double INF = numeric_limits<double>::max();

//
// class to use to sort the priority queue
//
class prioritize
{
public:
  bool operator()(const pair<long long, double>& p1, const pair<long long, double>& p2) const
  {
     if(p1.second> p2.second)
       return true;
     else if(p1.second < p2.second)
       return false;
     else
       return p1.first > p2.first;
  }
};

//
// calculates Dick’s algorithm 
//
void Dijkstra(graph& G, long long startV, map<long long, double>& distances, map<long long, long long>& visited)
{
  
  priority_queue<
   pair<long long, double>,
   vector<pair<long long, double>>,
   prioritize> unvisitedQ; 
  
  unvisitedQ.push(make_pair(startV, 0));
  distances[startV] = 0;
  visited[startV] = startV; 
  
  while(!unvisitedQ.empty()){ //loops through the queue until there are no unvisited Nodes
     pair<long long, double> currentV = unvisitedQ.top();
     unvisitedQ.pop();
     
     set<long long> neighborz = G.neighbors(currentV.first); 
     for(long long adjv : neighborz){ // loops through the neighbors of the current vertice
        double edgeWeight = 0;
        G.getWeight(currentV.first, adjv, edgeWeight);
        double alternativePathDistance = distances[currentV.first] + edgeWeight; 

        if(distances.count(adjv) == 0){ // if the neighbor isn't in the map of distances, add it with a distance of INF
           distances[adjv] = INF; 
        }
        if(alternativePathDistance < distances[adjv]){ //if a shorter distance is found, update distances and add it to the vertices visited
           visited[adjv] = currentV.first; 
           distances[adjv] = alternativePathDistance; 
           unvisitedQ.push(make_pair(adjv, distances[adjv]));
        }
     }
  }
}


//
// finds and calculates the shortest distance from a footway's node to the building's center 
//
void findShortest(double avgLat, double avgLon, long long& closestFootwayID, long long& closestNodeID, double& shortestD, Footways footways, Nodes nodes)
{
  shortestD = INF; 
  for(auto& footway : footways.MapFootways){
    vector<long long> NodeIDs = footway.NodeIDs; //creates a vector of all the nodeIDs of the footway
    for(unsigned int i = 0; i < NodeIDs.size(); i++){ //loops through each nodeID
      bool dummy;

      double lat;
      double lon; 
      nodes.find(NodeIDs[i], lat, lon, dummy); //gets the information of the current node

      // calclates the distances between the building's center and the node
      double dist = distBetween2Points(avgLat, avgLon, lat, lon);

      if(dist < shortestD){
        closestFootwayID = footway.ID; 
        closestNodeID = NodeIDs[i];
        shortestD = dist;
      }
    }
  }
}

//
// figure out which building the user wants to navigate to
//
void navigate(Buildings buildings, Nodes nodes, Footways footways, graph G)
{
  string b1name;
  long long b1ClosestNodeID; 
  Building startB(0, "", ""); 
  bool found1 = false; 

  cout << "Enter start building name (partial or complete)> " << endl;
  getline(cin, b1name);
  
  for (Building& B : buildings.MapBuildings) // loops through all the buildings to find the closest node to the starting building
  {
    if (B.Name.find(b1name) != string::npos) { // contains name:
      cout << "  Name: " << B.Name << endl;

      double avgLat;
      double avgLon; 
      B.getLocation(nodes, avgLat, avgLon); // gets the center of the starting building
      cout << "  Approximate Location: " << "(" << avgLat << ", " << avgLon << ")" << endl; 

      long long closestFootwayID;
      long long closestNodeID;
      double shortestD;
      findShortest(avgLat, avgLon, closestFootwayID, closestNodeID, shortestD, footways, nodes); // finds the node closest to the start building
      cout << "  Closest footway ID " << closestFootwayID << ", node ID " << closestNodeID << ", distance " << shortestD << endl; 
      b1ClosestNodeID = closestNodeID; 

      startB = B; 
      found1 = true;
      break;
    }
  }
  if(!found1){
    cout << "**Start building not found" << endl; 
    return; 
  }

  string b2name;
  long long b2ClosestNodeID;
  Building endB(0, "", ""); 
  bool found2 = false; 

  cout << "Enter destination building name (partial or complete)> " << endl;
  getline(cin, b2name);
  
  for (Building& B : buildings.MapBuildings)
  {
    if (B.Name.find(b2name) != string::npos) { // contains name:
      cout << "  Name: " << B.Name << endl;

      double avgLat;
      double avgLon; 
      B.getLocation(nodes, avgLat, avgLon); // gets the center of the destination building
      cout << "  Approximate Location: " << "(" << avgLat << ", " << avgLon << ")" << endl; 

      long long closestFootwayID;
      long long closestNodeID;
      double shortestD;
      findShortest(avgLat, avgLon, closestFootwayID, closestNodeID, shortestD, footways, nodes); // finds the node closest to the destination building
      cout << "  Closest footway ID " << closestFootwayID << ", node ID " << closestNodeID << ", distance " << shortestD << endl;
      b2ClosestNodeID = closestNodeID;  

      endB = B; 
      found2 = true;
      break;
    }
  }
  if(!found2){
    cout << "**Destination building not found" << endl; 
    return;
  }

  map<long long, double> distances;
  map<long long, long long> visited;
  Dijkstra(G, b1ClosestNodeID, distances, visited); // calculates Dick's algorithm
  if(distances.count(b2ClosestNodeID) == 0){
    cout << "Shortest weighted path:" << endl; 
    cout << "**Sorry, destination unreachable" << endl;
  }
  else{
    cout << "Shortest weighted path:" << endl; 
    cout << "  # nodes visited: " << distances.size() << endl; 
    cout << "  Distance: " << distances[b2ClosestNodeID] << " miles" << endl; 
    cout << "  Path: "; 

    stack<long long> c; // creates a stack and pushes value, which becomes the key to push the next value
    long long curr = b2ClosestNodeID;
    while(true){
      if(curr == visited[curr]){
        break;
      }
      c.push(curr); 
      curr = visited[curr]; 
    }
    cout << b1ClosestNodeID; 
    while(!c.empty()){ // pops the stack and outputs the top until the stack is empty
      cout << "->" << c.top(); 
      c.pop();
    }
    cout << endl; 

  }
}

//
// performs a sanity check to ensure the graph was made correctly
//
void sanityCheck(graph G){
  double w1, w2, w3, w4, w5, w6; 

  G.getWeight(9119071425, 533996671, w1); 
  G.getWeight(533996671, 9119071425, w2); 
  G.getWeight(533996671, 533996672, w3);
  G.getWeight(533996672, 533996671, w4);
  G.getWeight(533996672, 2240260064, w5);
  G.getWeight(2240260064, 533996672, w6);

  cout << "Graph check of Footway ID 986532630" << endl; 
  cout << "  Edge: (9119071425, 533996671, " << w1 << ")" << endl;
  cout << "  Edge: (533996671, 9119071425, " << w2 << ")" << endl;
  cout << "  Edge: (533996671, 533996672, " << w3 << ")" << endl;
  cout << "  Edge: (533996672, 533996671, " << w4 << ")" << endl;
  cout << "  Edge: (533996672, 2240260064, " << w5 << ")" << endl;
  cout << "  Edge: (2240260064, 533996672, " << w6 << ")" << endl;
}

//
// traverses the nodes and footways to build the Graph
//
void buildGraph(graph& G, Nodes nodes, Footways footways)
{
  for(auto& pair : nodes){ // adds the nodeIDs as vertices to the graph
    G.addVertex(pair.first); 
  }

  for(auto& footway: footways.MapFootways){ // loops through each footway in the footways
    vector<long long> NodeIDs = footway.NodeIDs; //creates a vector of all the nodeIDs of the footway
    for(unsigned int i = 0; i < NodeIDs.size(); i++){ //loops through each nodeID
      bool dummy;

      double lat1;
      double long1; 
      double id1 = NodeIDs[i]; 
      nodes.find(NodeIDs[i], lat1, long1, dummy); //gets the information of the current node

      i++; //increments to get the info of the next node, breaks out of the loop if it hits the end of the vector
      if(i == NodeIDs.size()){
        break;
      }

      double lat2;
      double long2;
      double id2 = NodeIDs[i]; 
      nodes.find(NodeIDs[i], lat2, long2, dummy); //gets the information of the next node

      // calclates the distances both ways between each node
      double dist1 = distBetween2Points(lat1, long1, lat2, long2);
      double dist2 = distBetween2Points(lat2, long2, lat1, long1);

      // adds the edges based on the calculated distances
      G.addEdge(id1, id2, dist1);
      G.addEdge(id2, id1, dist2); 

      i--; 
    }
  }
}

//
// main
//
int main()
{
  cout << setprecision(12); 

  XMLDocument xmldoc;
  Nodes nodes;
  Buildings buildings;
  Footways footways;
  graph G; 
  
  cout << "** NU open street map **" << endl;

  string filename;

  cout << endl;
  cout << "Enter map filename> " << endl;
  getline(cin, filename);

  //
  // 1. load XML-based map file 
  //
  if (!osmLoadMapFile(filename, xmldoc))
  {
    // failed, error message already output
    return 0;
  }
  
  //
  // 2. read the nodes, which are the various known positions on the map:
  //
  nodes.readMapNodes(xmldoc);

  //
  // NOTE: let's sort so we can use binary search when we need 
  // to lookup nodes.
  //
  nodes.sortByID();

  //
  // 3. read the university buildings:
  //
  buildings.readMapBuildings(xmldoc);

  //
  // 4. read the footways, which are the walking paths:
  //
  footways.readMapFootways(xmldoc);

  //
  // 5. traverse the nodes and footways to build the Graph
  //
  buildGraph(G, nodes, footways); 
  

  //
  // 6. stats
  //
  cout << "# of nodes: " << nodes.getNumMapNodes() << endl;
  cout << "# of buildings: " << buildings.getNumMapBuildings() << endl;
  cout << "# of footways: " << footways.getNumMapFootways() << endl;
  cout << "# of graph vertices: " << G.NumVertices() << endl;
  cout << "# of graph edges: " << G.NumEdges()  << endl; 

  //
  // now let the user for search for 1 or more buildings:
  //
  while (true)
  {
    string name;

    cout << endl;
    cout << "Enter building name, * to list, @ to navigate, or $ to end> " << endl;

    getline(cin, name);

    if (name == "$") {
      break;
    }
    else if (name == "*") {
      buildings.print();
    }
    else if (name == "!") {
      sanityCheck(G); 
    }
    else if (name == "@") {
      navigate(buildings, nodes, footways, G);
    }
    else {
      buildings.findAndPrint(name, nodes, footways);
    }

  }//while

  //
  // done:
  //
  cout << endl;
  cout << "** Done  **" << endl;
  cout << "# of calls to getID(): " << Node::getCallsToGetID() << endl;
  cout << "# of Nodes created: " << Node::getCreated() << endl;
  cout << "# of Nodes copied: " << Node::getCopied() << endl;
  cout << endl;

  return 0;
}
