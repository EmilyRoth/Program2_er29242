package Program2;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.PriorityQueue;
/*
 * Name: <your name>
 * EID: <your EID>
 */

public class Graph implements Program2{
	// n is the number of ports
	private int n;
    
	// Edge is the class to represent an edge between two nodes
	// node is the destination node this edge connected to
	// time is the travel time of this edge
	// capacity is the capacity of this edge
	// Use of this class is optional. You may make your own, and comment
	// this one out.
	private class Edge{
		public int node;
		public int time;
		public int capacity;
		public Edge(int n, int t, int c){
			node = n;
			time = t;
			capacity = c;
		}

		// prints out an Edge.
		public String toString() {
			return "" + node;
		}
	}

	// Here you have to define your own data structure that you want to use
	// to represent the graph
	// Hint: This include an ArrayList or many ArrayLists?
	// ....


	HashMap<Integer, ArrayList<Edge>> undirectedGraph = new HashMap<>();
	// This function is the constructor of the Graph. Do not change the parameters
	// of this function.
	//Hint: Do you need other functions here?
	public Graph(int x) {
		n = x;
		for(int i = 0; i < n; i++){
			ArrayList<Edge> adj = new ArrayList<>();
			undirectedGraph.put(i, adj);
		}
	}


	// This function is called by Driver. The input is an edge in the graph.
	// Your job is to fix this function to generate your graph.
	// Do not change its parameters or return type.
	// Hint: Here is the place to build the graph with the data structure you defined.
	public void inputEdge(int port1, int port2, int time, int capacity) {
		Edge edge1 = new Edge(port2, time, capacity);
		undirectedGraph.get(port1).add(edge1);
		Edge edge2 = new Edge(port1, time, capacity);
		undirectedGraph.get(port2).add(edge2);
		return;
	}

	// This function is the solution for the Shortest Path problem.
	// The output of this function is an int which is the shortest travel time from source port to destination port
	// Do not change its parameters or return type.
    public int findTimeOptimalPath(int sourcePort, int destPort) {
        ArrayList<Vertex> distance = new ArrayList<>();
        for(int i = 0; i < n; i++){
            // all distances infinite
            Vertex newV = new Vertex(i, null);
            distance.add(newV);
            // all nodes in path infinity
        }
        // set distance source to source
        distance.get(sourcePort).distance = 0;
        Comparator<Vertex> vComp = new vertexComparator();
        PriorityQueue<Vertex> queue = new PriorityQueue<>(vComp);
        // get unvisited node with minimum weight
        queue.add(distance.get(sourcePort));
        while (!queue.isEmpty()){
            // gets the smalled distance on queue
            Vertex v = queue.poll();
            // gets the node of the smallest distance on queue
            if(v.node == destPort){
                return v.distance;
            }
            // get all adjacent nodes
            for(Edge adj : undirectedGraph.get(v.node)){
                // current min distance
                int min = adj.time + v.distance;
                // left off
                if(distance.get(adj.node).distance == null || distance.get(adj.node).distance > min){
                    // remove and update distance value
                    distance.get(adj.node).distance = min;
                    // updates distance value
                    queue.add(distance.get(adj.node));
                }
            }
        }

        return -1;
    }

    // This function is the solution for the Widest Path problem.
	// The output of this function is an int which is the maximum capacity from source port to destination port 
	// Do not change its parameters or return type.
	public int findCapOptimalPath(int sourcePort, int destPort) {
        ArrayList<Vertex> capacity = new ArrayList<>();
        for(int i = 0; i < n; i++){
            // all distances infinite
            Vertex newV = new Vertex(i, 0);
            capacity.add(newV);
            // all nodes in path infinity
        }
        // set distance source to source
        capacity.get(sourcePort).capacityVertex = Integer.MAX_VALUE;
        Comparator<Vertex> cComp = new capacityComparator();
        PriorityQueue<Vertex> queue = new PriorityQueue<>(cComp);
        // get unvisited node with minimum weight
        queue.add(capacity.get(sourcePort));
        while (!queue.isEmpty()){
            // gets the smalled distance on queue
            Vertex v = queue.poll();
            // gets the node of the smallest distance on queue
            if(v.node == destPort){
                return v.capacityVertex;
            }
            // get all adjacent nodes
            for(Edge adj : undirectedGraph.get(v.node)){
                // current min distance
                int max = adj.capacity;
                if(max > v.capacityVertex){
                    max = v.capacityVertex;
                }
                // left off
                if(capacity.get(adj.node).capacityVertex < max){
                    // remove and update distance value
                    capacity.get(adj.node).capacityVertex = max;
                    // updates distance value
                    queue.add(capacity.get(adj.node));
                }
            }
        }

        return -1;
    }

	// This function returns the neighboring ports of node.
	// This function is used to test if you have contructed the graph correct.
	public ArrayList<Integer> getNeighbors(int node) {
		ArrayList<Integer> edges = new ArrayList<Integer>();
		for(Edge adj : undirectedGraph.get(node)){
		    edges.add(adj.node);
        }
		return edges;
	}

	public int getNumPorts() {
		return n;
	}

	class Vertex{
	    int node;
	    Integer distance;
	    Integer capacityVertex;
	    public Vertex(){

        }
        public Vertex(int n, Integer v){
	        node = n;
	        distance = v;
            capacityVertex = v;
        }

    }

    public class vertexComparator implements Comparator<Vertex>{

        @Override
        public int compare(Vertex o1, Vertex o2) {
            return (o1.distance > o2.distance) ? 1 : -1;
        }
    }

    public class capacityComparator implements Comparator<Vertex>{

        @Override
        public int compare(Vertex o1, Vertex o2) {
            return (o1.capacityVertex < o2.capacityVertex) ? 1 : -1;
        }
    }
}
//return (st1.getCgpa() < st2.getCgpa()) ? 1 : -1;