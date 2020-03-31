import processing.core.PVector;
import java.util.ArrayList;

class TreeNode extends GraphNode{
  PVector location;
  TreeNode parent;
  ArrayList<TreeNode> children;
  
  private static int idInc = 0;
  final int id;
  
  TreeNode(GraphNode node){
    super(node.location);
    location = node.location.copy();
    children = new ArrayList();
    this.id = idInc++;
  }
  
  TreeNode(PVector location, TreeNode parent){
    super(location);
    this.location = location.copy();
    this.children = new ArrayList();
    this.parent = parent;
    this.id = idInc++;
  }
}
