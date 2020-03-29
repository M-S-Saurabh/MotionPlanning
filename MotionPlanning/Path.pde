class Path implements Comparable<Path>{
  ArrayList<GraphNode> list;
  float cost;
  
  Path(ArrayList<GraphNode> list, float cost){
    this.list = list;
    this.cost = cost;
  }
  
  GraphNode getLastNode(){
    if(list.size() > 0){
      return list.get(list.size()-1);
    }else{
      return null;
    }
  }
  
  @Override
  public int compareTo(Path other) {
      return Float.valueOf(this.cost).compareTo(other.cost);
  }
}
