// Useful to sort lists by a custom key
import java.util.Comparator;
import java.util.Collections;
import java.util.*;

/// In this file you will implement your navmesh and pathfinding. 

/// This node representation is just a suggestion
class Node
{
   int id;
   ArrayList<Wall> polygon;
   PVector center;
   ArrayList<Node> neighbors;
   ArrayList<Wall> connections;
   
   Node(ArrayList<Wall> polygon) {
     this.polygon = polygon;
     get_center();
     neighbors = new ArrayList<Node>();
     connections = new ArrayList<Wall>();
   }
   
   // this only needs to be the visual center, right?
   // this function sets the center member field of this node to the average of all the stored polygon's vertices
   void get_center() {
     PVector out = new PVector(0,0);
     for(Wall side: polygon) {
       out.add(side.start);
     }
     center = out.div(polygon.size());
   }
   
   // add new_neighbor as this node's neighbor
   void add_neighbor(Node new_neighbor) {
     if(!neighbors.contains(new_neighbor)) {
       neighbors.add(new_neighbor);
       new_neighbor.neighbors.add(this);
       connections.add(new Wall(center, new_neighbor.center));
     }
   }
   
   Wall get_shared_edge(Node neighbor) {
     assert(neighbors.contains(neighbor));
     
     for(Wall n_wall: polygon) {
       for (Wall o_wall: neighbor.polygon) {
         if((n_wall.start == o_wall.start && n_wall.end == o_wall.end) || (n_wall.start == o_wall.end && n_wall.end == o_wall.start)) {
           return n_wall;
         }
       }
     }
     // the computer is too stupid to realize that it will never make it here
     println("something absolutely horrible has gone wrong here");
     return null;
   }
}



class NavMesh
{   
   ArrayList<Node> graph;
   
   ArrayList<Wall> funny_node;
  
   void bake(Map map)
   {
       /// generate the graph you need for pathfinding
       // using simple triangulation algorithm found at https://arxiv.org/ftp/arxiv/papers/1212/1212.6038.pdf:
       
       funny_node = new ArrayList<Wall>(); // temporary
       
       
       // get array of vertices
       ArrayList<PVector> vertices = new ArrayList<PVector>();
       // for a simple polygon, # of sides == # of vertices, so just getting the start point of each side works out
       for(int index = 0 ; index < map.walls.size(); index++) {
         vertices.add(map.walls.get(index).start);
       }
       //println(vertices);
       
       // determine which vertices are ear tips, and if they are, get their interior angle
       ArrayList<ArrayList<PVector>> ears = new ArrayList<ArrayList<PVector>>();
       ArrayList<Float> ear_tip_angles = new ArrayList<Float>();
       for(int index = 0; index < vertices.size(); index++) {
         // get this vertex, the previous vertex, and the next vertex
         PVector this_vertex,prev_vertex,next_vertex;
         
         // the first vertex in the list's previous vertex is the last, and the last vertex's next node is the first
         if(index == 0) {
           prev_vertex = vertices.get(vertices.size() - 1);
         }
         else {
           prev_vertex = vertices.get(index - 1);
         }
         
         if(index == vertices.size() - 1) {
           next_vertex = vertices.get(0);
         }
         else {
           next_vertex = vertices.get(index + 1);
         }
         
         this_vertex = vertices.get(index);
         
         // determine if vertex is an ear tip (the line segment between prev_vertex and next_vertex must be entirely within the map)
         // the collides_exclusive function on map (that I added) lets me exclude the sides that have the vertex as an endpoint
         // in order to make sure the vertices can actually "see" each other I also check if the line's midpoint is inside the map
         //print(!map.collides_exclusive(prev_vertex,next_vertex) && isPointInPolygon(PVector.mult(PVector.add(prev_vertex, next_vertex), 0.5),map.walls));
         if(!collides_exclusive(map.walls,prev_vertex,next_vertex) && isPointInPolygon(PVector.mult(PVector.add(prev_vertex, next_vertex), 0.5),map.walls)) {
           // this_vertex is an ear tip; now add a new ear to the list
           ArrayList<PVector> new_ear = new ArrayList<PVector>();
           new_ear.add(this_vertex);
           new_ear.add(prev_vertex);
           new_ear.add(next_vertex);
           ears.add(new_ear);
           // then calculate the angle at the vertex
           // given the location of three points, finding the angle can be done using law of cosines
           // given triangle with sides a, b, and c, and angles A, B, and C such that A is opposite A, and so on:
           // c^2 = a^2 + b^2 - 2ab*cos(C)
           // or C = arccos( (c^2 - a^2 - b^2) / -2ab )
           // let's say angle A is the angle at prev_vertex, angle B is the angle at next_vertex, and angle C is the angle at this_vertex (we want angle C)
           // find side lengths:
           float length_a = this_vertex.dist(next_vertex);
           float length_b = prev_vertex.dist(this_vertex);
           float length_c = prev_vertex.dist(next_vertex);
           float angle_at_vertex = acos( (sq(length_c) - sq(length_a) - sq(length_b)) / (-2 * length_a * length_b) );
           ear_tip_angles.add(angle_at_vertex);
           // hopefully each ear and the corresponding angle at the ear-tip will be at the same index... hopefully......
         }
       }
       //println(ears);
       //println(ear_tip_angles);
       
       // I understand this may be a crime and I will continue to do so anyway
       graph = new ArrayList<Node>();
       
       ArrayList<PVector> vertices_copy = new ArrayList<PVector>();
       vertices_copy.addAll(vertices);
       
       while(graph.size() < vertices.size() - 2) {
         println("graph size:" + graph.size() + ",original number of vertices:" + vertices.size());
         // find the ear-tip with the smallest interior angle and add its ear to the graph
         int smallest_angle_index = 0;
         float smallest_angle = ear_tip_angles.get(0);
         for (int index = 1; index < ear_tip_angles.size(); index++) {
           if(ear_tip_angles.get(index) < smallest_angle) {
             smallest_angle = ear_tip_angles.get(index);
             smallest_angle_index = index;
           }
         }
         
         ArrayList<Wall> new_polygon = new ArrayList<Wall>();
         PVector[] ear_array = new PVector[0];
         AddPolygon(new_polygon,ears.get(smallest_angle_index).toArray(ear_array));
         graph.add(new Node(new_polygon));
         
         // remove the added ear-tip; it has been clipped
         // (the ear-tip is at index 0 in the nested arraylist in the ear arraylist)
         // need to remember the removed index's vertex in order to handle the next bit
         println("current vertices: " + vertices_copy);
         println("current smallest angle: " + smallest_angle + ", index:" + smallest_angle_index);
         println("current list of ears: " + ears);
         println("ear-tip in question:" + ears.get(smallest_angle_index).get(0));
         int removed_vertex_index = vertices_copy.indexOf(ears.get(smallest_angle_index).get(0));
         println("removing vertex at " + removed_vertex_index);
         vertices_copy.remove(removed_vertex_index);
         ears.remove(smallest_angle_index);
         ear_tip_angles.remove(smallest_angle_index);
         
         // copy pasted loop
         ears = new ArrayList<ArrayList<PVector>>();
         ear_tip_angles = new ArrayList<Float>();
         for(int index = 0; index < vertices_copy.size(); index++) {
           // get this vertex, the previous vertex, and the next vertex
           PVector this_vertex,prev_vertex,next_vertex;
           
           // the first vertex in the list's previous vertex is the last, and the last vertex's next node is the first
           if(index == 0) {
             prev_vertex = vertices_copy.get(vertices_copy.size() - 1);
           }
           else {
             prev_vertex = vertices_copy.get(index - 1);
           }
           
           if(index == vertices_copy.size() - 1) {
             next_vertex = vertices_copy.get(0);
           }
           else {
             next_vertex = vertices_copy.get(index + 1);
           }
           
           this_vertex = vertices_copy.get(index);
           
           // determine if vertex is an ear tip (the line segment between prev_vertex and next_vertex must be entirely within the map)
           // the collides_exclusive function on map (that I added) lets me exclude the sides that have the vertex as an endpoint
           // in order to make sure the vertices can actually "see" each other I also check if the line's midpoint is inside the map
           //print(!map.collides_exclusive(prev_vertex,next_vertex) && isPointInPolygon(PVector.mult(PVector.add(prev_vertex, next_vertex), 0.5),map.walls));
           if(!collides_exclusive(map.walls,prev_vertex,next_vertex) && isPointInPolygon(PVector.mult(PVector.add(prev_vertex, next_vertex), 0.5),map.walls)) {
             // this_vertex is an ear tip; now add a new ear to the list
             ArrayList<PVector> new_ear = new ArrayList<PVector>();
             new_ear.add(this_vertex);
             new_ear.add(prev_vertex);
             new_ear.add(next_vertex);
             ears.add(new_ear);
             // then calculate the angle at the vertex
             // given the location of three points, finding the angle can be done using law of cosines
             // given triangle with sides a, b, and c, and angles A, B, and C such that A is opposite A, and so on:
             // c^2 = a^2 + b^2 - 2ab*cos(C)
             // or C = arccos( (c^2 - a^2 - b^2) / -2ab )
             // let's say angle A is the angle at prev_vertex, angle B is the angle at next_vertex, and angle C is the angle at this_vertex (we want angle C)
             // find side lengths:
             float length_a = this_vertex.dist(next_vertex);
             float length_b = prev_vertex.dist(this_vertex);
             float length_c = prev_vertex.dist(next_vertex);
             float angle_at_vertex = acos( (sq(length_c) - sq(length_a) - sq(length_b)) / (-2 * length_a * length_b) );
             ear_tip_angles.add(angle_at_vertex);
             // hopefully each ear and the corresponding angle at the ear-tip will be at the same index... hopefully......
           }
       }
         
         println();
       } // end while
       
       // now we have our graph filled with nodes containing polygons, but they haven't been linked to each other yet
       // to fix that is this following for loop:
       for(Node node: graph) {
         for(Node other: graph) {
           // we will not be linking a node to itself
           if(!node.equals(other)) {
             // this kinda looks expensive...
             // but hopefully in practice, because all of the polygons are triangles
             // the following double for loop at max only runs 9 times
             for(Wall n_wall: node.polygon) {
               for (Wall o_wall: other.polygon) {
                 if((n_wall.start == o_wall.start && n_wall.end == o_wall.end) || (n_wall.start == o_wall.end && n_wall.end == o_wall.start)) {
                   node.add_neighbor(other);
                 }
               }
             }
           }
         }
       }
   }
   
   void funny_glow(PVector start, PVector end) {
     funny_node.clear();
     AddPolygon(funny_node,findPath(start,end).toArray(new PVector[0]));
     funny_node.remove(funny_node.size() - 1);
   }
   
   ArrayList<PVector> findPath(PVector start, PVector destination)
   {
      /// implement A* to find a path
      // we want to run pathfinding on the graph, so find the polygons that the start and destination are in
      Node start_node = null,end_node = null;
      for(Node n: graph) {
        if(isPointInPolygon(start, n.polygon)) start_node = n;
        if(isPointInPolygon(destination,n.polygon)) end_node = n;
      }
      assert(start_node != null && end_node != null);
      
      PriorityQueue<QueueNode> pqueue = new PriorityQueue<QueueNode>();
      ArrayList<Node> visited = new ArrayList<Node>();
      
      pqueue.offer(new QueueNode(start_node,0,0,null));
      
      while(!pqueue.peek().data.equals(end_node)) { //<>//
        float current_distance_traveled = pqueue.peek().distance_traveled;
        QueueNode best_node = pqueue.poll();
        //println("best node: " + best_node.data.center + ", distance traveled: " + best_node.distance_traveled + ", heuristic: " + best_node.heuristic);
        // expand best node
        for(Node neighbor: best_node.data.neighbors) { //<>//
          if(!visited.contains(neighbor)) {
            //println("  neighbor added to queue: " + neighbor.center);
            QueueNode next_node = new QueueNode(neighbor,current_distance_traveled + best_node.data.center.dist(neighbor.center),neighbor.center.dist(destination),best_node);
          
            /*if(!pqueue.contains(next_node))*/ pqueue.offer(next_node);
            visited.add(neighbor);
          }
        }
      }
      
      ArrayList<Node> reversed_list = new ArrayList<Node>();
      QueueNode current_node = pqueue.peek();
      while(current_node != null) {
        reversed_list.add(current_node.data);
        current_node = current_node.parent;
      }
      
      
      // this next loop should simultaneously correctly reverse the data properly and convert from Nodes to usable vectors
      ArrayList<PVector> result = new ArrayList<PVector>();
      while(reversed_list.size() > 1) {
        Node node_2 = reversed_list.get(1);
        Node node_1 = reversed_list.remove(0);
        
        result.add(0,node_1.get_shared_edge(node_2).center());
      }
      // theoretically, result is now filled with the list of midpoints of edges of adjacent polygons
      // so now, you get a path through to each polygon and if you follow it you'll get to the polygon of end_node
      // now, all you'll need to do is...
      result.add(destination);
      return result;
   }
   
   
   void update(float dt)
   {
      draw();
   }
   
   void draw()
   {
      
      //fill(0,255,0);
      /// use this to draw the nav mesh graph
      for(Node n: graph) {
        stroke(#B00B69);
        for(Wall w: n.polygon) {
          w.draw();
        }
        stroke(#042069);
        fill(#042069);
        circle(n.center.x,n.center.y,5);
        for(Wall w: n.connections) {
          w.draw();
        }
        
        
        stroke(#420A55);
        fill(#420A55);
        for(Wall w: funny_node) {
          w.draw();
        }
      }
   }
   
   // same function as "collides" in the Map, but excludes any walls that have the endpoints of the target wall as an endpoint
   // (and additionally takes in a polygon as input)
   // this can return false for a line connecting two of the map's vertices (I hope)
   // but that includes lines that are "outside" the map as well, so bear that in mind
   boolean collides_exclusive(ArrayList<Wall> polygon, PVector from, PVector to)
   {
      for (Wall w : polygon)
      {
         if (w.start.dist(from) != 0 && w.start.dist(to) != 0 && w.end.dist(from) != 0 && w.end.dist(to) != 0)
         if (w.crosses(from, to)) return true;
      }
      return false;
   }
}

class QueueNode implements Comparable<QueueNode> {
    Node data;
    float distance_traveled;
    float heuristic;
    QueueNode parent;
    
    public QueueNode(Node data, float distance_traveled, float heuristic, QueueNode parent) {
      this.data = data;
      this.distance_traveled = distance_traveled;
      this.heuristic = heuristic;
      this.parent = parent;
    }
    
    @Override int compareTo(QueueNode other) {
        return Float.compare(distance_traveled + heuristic, other.distance_traveled + other.heuristic);
    }
    
    @Override boolean equals(Object other) {
      if(other == this) return true;
      if(!(other instanceof QueueNode)) return false;
      
      QueueNode o_node = (QueueNode) other;
      
      // if(data.center.x == o_node.data.center.x && data.center.y == o_node.data.center.y && distance_traveled == o_node.distance_traveled) println("QueueNode.equals returned true");
      return( data.center.x == o_node.data.center.x && data.center.y == o_node.data.center.y && distance_traveled == o_node.distance_traveled);
    }
}
