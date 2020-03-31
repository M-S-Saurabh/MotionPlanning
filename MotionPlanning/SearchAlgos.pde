
ArrayList<GraphNode> searchPathUCS(GraphNode start, GraphNode end){
  PriorityQueue<Path> ucsPathQueue = new PriorityQueue();
  ArrayList<GraphNode> startPath = new ArrayList();
  startPath.add(start);
  ucsPathQueue.add(new Path(startPath, 0));
  //print("Added "); printList(startPath); print("\n");
  
  while(!ucsPathQueue.isEmpty()){
    Path path = ucsPathQueue.remove();
    //print("Popped "); printList(path.list); print(":"+path.cost+"\n");
    GraphNode lastNode = path.getLastNode();
    if(lastNode == end){
      //println("returned"+start.location+" "+end.location);
      return path.list;
    }
    for(GraphNode neighbor : lastNode.neighbors){
      if(neighbor.visited){continue;}
      ArrayList<GraphNode> newPath = (ArrayList) path.list.clone();
      newPath.add(neighbor);
      float newCost = path.cost+lastNode.location.dist(neighbor.location);
      ucsPathQueue.add( new Path(newPath, newCost));
      //print("Added "); printList(newPath); print(":"+newCost+"\n");
    }
    lastNode.setVisited();
  }
  displayNoPath = true;
  return new ArrayList();
}

ArrayList<GraphNode> shortestPathBFS(GraphNode start, GraphNode end){
  Queue<ArrayList<GraphNode>> bfsPathQueue = new LinkedList();
  ArrayList<GraphNode> startPath = new ArrayList();
  startPath.add(start);
  bfsPathQueue.add(startPath);
  //print("Added "); printList(startPath);
  
  while(!bfsPathQueue.isEmpty()){
    ArrayList<GraphNode> path = bfsPathQueue.remove();
    //print("Popped "); printList(path);
    GraphNode lastNode = path.get(path.size()-1);
    if(lastNode == end){
      //println("returned"+start.location+" "+end.location);
      return path;
    }
    for(GraphNode neighbor : lastNode.neighbors){
      if(neighbor.visited){continue;}
      ArrayList<GraphNode> newPath = (ArrayList) path.clone();
      newPath.add(neighbor);
      bfsPathQueue.add(newPath);
       //print("Added "); printList(newPath);
    }
    lastNode.setVisited();
  }
  displayNoPath = true;
  return new ArrayList();
}

void printList(ArrayList<GraphNode> path){
  for(GraphNode node : path){
    print(" "+node.id);
  }
}

void resetGraphVisited(){
  for(GraphNode node : roadmap){
    node.resetVisited();
  }
}
