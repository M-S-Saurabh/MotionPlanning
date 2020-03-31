import processing.core.PVector;
import java.util.ArrayList;
import java.util.Comparator;

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
    if(this.neighbors.indexOf(neighbor) < 0){
      this.neighbors.add(neighbor);
    }
    if(neighbor.neighbors.indexOf(this) < 0){
      neighbor.neighbors.add(this);
    }
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
  void resetVisited(){
    this.visited = false;
  }
}
