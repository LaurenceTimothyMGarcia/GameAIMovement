// Useful to sort lists by a custom key
import java.util.Comparator;


/// In this file you will implement your navmesh and pathfinding. 

/// This node representation is just a suggestion
class Node
{
   int id;
   ArrayList<Wall> polygon;
   PVector center;
   ArrayList<Node> neighbors;
   ArrayList<Wall> connections;
}



class NavMesh
{   
   ArrayList<PVector> reflex_vertices;
  
   void bake(Map map)
   {
       /// generate the graph you need for pathfinding
       // using simple triangulation algorithm found at https://arxiv.org/ftp/arxiv/papers/1212/1212.6038.pdf:
       
       // get array of vertices
       ArrayList<PVector> vertices = new ArrayList<PVector>();
       // ArrayList's foreach syntax makes me want to die; using a shitty for loop instead
       // for a simple polygon, # of sides == # of vertices, so just getting the start point of each side works out
       for(int index = 0 ; index < map.walls.size(); index++) {
         vertices.add(map.walls.get(index).start);
       }
       println(vertices);
       
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
         if(!map.collides_exclusive(prev_vertex,next_vertex) && isPointInPolygon(PVector.mult(PVector.add(prev_vertex, next_vertex), 0.5),map.walls)) {
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
       println(ears);
       println(ear_tip_angles);
   }
   
   ArrayList<PVector> findPath(PVector start, PVector destination)
   {
      /// implement A* to find a path
      ArrayList<PVector> result = null;
      return result;
   }
   
   
   void update(float dt)
   {
      draw();
   }
   
   void draw()
   {
      /// use this to draw the nav mesh graph
   }
}
