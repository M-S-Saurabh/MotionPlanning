import processing.core.PVector;
import java.util.ArrayList;

class GraphNode{
  private static int idInc = 0;
  final int id;
  boolean visited = false;
  PVector location;
  ArrayList<GraphNode> neighbors;
  
  GraphNode(PVector location){
    this.location = location.copy();
    neighbors = new ArrayList();
    this.id = idInc++;
  }
  
  void addNeighbor(GraphNode neighbor){
    this.neighbors.add(neighbor);
  }
  
  void removeNeighbor(GraphNode neighbor){
    this.neighbors.remove(neighbor);
  }
  void removeNeighbor(int index){
    this.neighbors.remove(index);
  }
  void setVisited(){
    this.visited = true;
  }
}
