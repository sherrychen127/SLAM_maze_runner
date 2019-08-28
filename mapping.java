

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.PriorityQueue;
import java.util.concurrent.TimeUnit;
import java.lang.Math;

import lejos.hardware.Button;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.Motor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.EV3ColorSensor;

public class mapping {
	static int asci_count = 0; // ASCII counter
	static int[] coord = new int [2]; // Keep track of coordinates
	static Map<Character, int[]> char_to_position; // Hash map maps node with given name to coordinate on map
	static char[][] my_map; // Stores maze map
	
	public static void main(String[] args) {  
		while(true){
			if (Button.DOWN.isDown()) break;
		}
		EV3UltrasonicSensor sonic = new EV3UltrasonicSensor(SensorPort.S1); // sonic @ S1
		EV3GyroSensor tilt = new EV3GyroSensor(SensorPort.S4); // gyro @ S4
		EV3ColorSensor color = new EV3ColorSensor(SensorPort.S2); // color @ S2
		
		float[] tiltSample = new float[1];
		float[] colorSample = new float[1];
		float[] sonicSample = new float[1];
		
		tilt.getAngleMode().fetchSample(tiltSample,0);
		sonic.fetchSample(sonicSample,0);
		color.getAmbientMode().fetchSample(colorSample, 0);
	
		int sizeMapX = 11; int sizeMapY = 11;
		char curPos = 'A'; // Start position of robot (to be updated)
		char curHead = 'D'; // Start orientation of robot (either 'U', 'D', 'L', 'R') (to be updated)
		char goalPos = 'Y'; // Final position the robot needs to reach
		char goalHead = 'U'; // Final orientation the robot needs to reach (either 'U', 'D', 'L', 'R')
		
		char prevPos = 'A';
		char prevHead = 'D';
		
		List<Character> optPath; // Optimal path
		
		initializeMap(); // Initialize map with no walls
		
		float prev_sonic_reading = sonicSample[0]*100 - 3;
		float prev_gyro_reading = tiltSample[0];
		
		
		double desired_color = 0.5;
		double k_p = 200;
		
		System.out.println("hello world");

		while(!Button.ENTER.isDown()){
			
			tilt.getAngleMode().fetchSample(tiltSample,0);
			sonic.fetchSample(sonicSample,0);
			color.getRedMode().fetchSample(colorSample, 0);
			
			
			float curr_sonic_reading = sonicSample[0]*100;
			float curr_gyro_reading = tiltSample[0];
			
			if (Math.abs(Math.abs(curr_gyro_reading - prev_gyro_reading) - 90)<= 5){
				int heading = 1;
				if (curr_gyro_reading - prev_gyro_reading < 0 ){
					heading = 0;
				}
				curHead = update_heading(curHead, heading);
				
				prev_gyro_reading = curr_gyro_reading;
				prev_sonic_reading = curr_sonic_reading;
				System.out.println("H" + " " + curPos + " " + curHead);

			}

			if (Math.abs(prev_sonic_reading - curr_sonic_reading - 30) <= 2.5){
				//System.out.println("update");
				curPos = update_position(curPos, curHead);
				
				System.out.println("P" + " " + curPos + " " +curHead);
				prev_sonic_reading = curr_sonic_reading;
				
			}
			
			if (curr_sonic_reading <=12.5){
				stop();
				updateMap(curPos,curHead);
			}
			
			if (curPos != prevPos || curHead != prevHead){
				Graph g = getGraph(my_map, sizeMapX, sizeMapY, char_to_position);
				optPath = g.getShortestPath(curPos, goalPos);
				char next_node= optPath.get(optPath.size() - 1);
				System.out.println("next_node: " + " " + next_node);
				stop();
				go_to_next_node(curPos, curHead, next_node, tilt, sonic);
				prevPos = curPos;
				prevHead = curHead;
				continue;
			}
			

			
			if (curPos == prevPos && curHead == prevHead){
				forward(color,sonic);
			}
			
		}

		//Graph g = getGraph(my_map, sizeMapX, sizeMapY, char_to_position); // Create graph out of initialized map
		//optPath = g.getShortestPath(curPos, goalPos); // Get optimal path from current position to goal
		//System.out.println("Optimal Path: " + optPath);
		//printMap(my_map); // Print map to see structure of map (can choose to print for debugging purposes)
		
		//my_map[1][6] = '1'; // Add a wall to the map (for demo)
		//g = getGraph(my_map, sizeMapX, sizeMapY, char_to_position); // Create graph out of initialized map
		//optPath = g.getShortestPath(curPos, goalPos); // Get optimal path from current position to goal
		//System.out.println("Optimal Path: " + optPath);
		//printMap(my_map); // Print map to see structure of map (can choose to print for debugging purposes)
		
		// Insert your code here...
	}

	public static char update_position(char curPos, char curHead){
		
		int row = -1;
		int col  = -1;
		for (int i = 0; i<11;i++){
			for (int j = 0; j<11; j++){
				if (my_map[i][j] == curPos){
					row = i;
					col = j;
				}
			}
		}

		int currPosition = curPos - 'A' + 1;  //'A' = 1
		if (curHead == 'U'){
			if (currPosition <=5){
				return curPos;
			}
			else{
				
				if (row >0 && col>0){
					return my_map[row - 2][col];
				}
			}
		}

		else if (curHead == 'D'){
			if (currPosition >=21){
				return curPos;
			}
			else{
				if (row >0 && col>0){
					return my_map[row + 2][col];
				}
			}
		}
		
		else if (curHead == 'L'){
			if (currPosition%5 == 1){
				return curPos;
			}
			else{
				if (row >0 && col>0){
					return my_map[row][col-2];
				}
			}
		}
		
		else if (curHead == 'R'){
			if (currPosition%5 == 0){
				return curPos;
			}
			else{
				if (row >0 && col>0){
					return my_map[row][col+2];
				}
			}
		}
		return curPos;
	}

	public static char update_heading(char currHead, int heading){
		if (heading == 1){
			if (currHead == 'U'){
				return 'L';
			}
			else if(currHead == 'D'){
				return 'R';
			}
			else if (currHead == 'L'){
				return 'D';
			}
			else{
				return 'U';
			}
		}
		else if (heading == 0){
			if (currHead == 'U'){
				return 'R';
			}
			else if(currHead == 'D'){
				return 'L';
			}
			else if (currHead == 'L'){
				return 'U';
			}
			else{
				return 'D';
			}
		}
		return currHead;
		
	}
	
	public static void initializeMap(){ 
		/* Map should look like:
		 *  ZZZZZZZZZZZ
			ZA0B0C0D0EZ
			Z0Z0Z0Z0Z0Z
			ZF0G0H0I0JZ
			Z0Z0Z0Z0Z0Z
			ZK0L0M0N0OZ
			Z0Z0Z0Z0Z0Z
			ZP0Q0R0S0TZ
			Z0Z0Z0Z0Z0Z
			ZU0V0W0X0YZ
			ZZZZZZZZZZZ
			
			Hash map char_to_postion is like a dictionary relating characters (e.g. 'A') to coordinates (e.g. [1,1]) in my_map
			
			Note that positive X is right and positive Y is down
			Z character is a null entry of the map
			Alphabetical characters from A to Y are potential positions the robot can be in
			Numerical characters can hold either 0 or 1 (to signify empty space or wall respectively between its neighbouring positions)
		 */
		
		char_to_position = new HashMap<Character, int[]>(); // Create hash from character to position in map
		my_map = new char[11][11]; // Create map from position to character (i.e. regular map)
		char letter; // Holds character corresponding to a position in the map
		// Populate entire array with Z
		for(int i = 0; i < 11; i++){
			for(int j =0; j < 11; j ++){
				my_map[i][j] = 'Z';
			}
		}
		// Populate inner map area with 0's to signify free path between robot positions
		for(int i = 1; i < 10; i++){
			for(int j =1; j < 10; j ++){
				my_map[i][j] = '0';
			}
		}
		// Populate cells from A-Y where robot will go
		for(int i = 1; i < 10; i+=2){
			for(int j =1; j < 10; j +=2){
				int[] coord = new int [2]; // Must create new array object so since hash map points all keys to same 
				letter = (char)(65+asci_count);
				my_map[i][j] = letter;
				coord [0] = i; coord[1] = j;
				char_to_position.put(letter, coord);
				asci_count++;
			}
		}
		
		//Rest of map is padded with Z character to make parsing the map easier to implement
		for(int i = 2; i < 11; i+=2){
			for(int j =2; j < 11; j +=2){
				my_map[i][j] = 'Z';
			}
		}
	}
	public static void left_turn(EV3GyroSensor tilt){
		float[] tiltSample = new float[1];
		tilt.getAngleMode().fetchSample(tiltSample,0);
		float prev_gyro_reading = tiltSample[0];
		float cur_gyro_reading = prev_gyro_reading;
		Motor.B.setSpeed(90);
		Motor.C.setSpeed(90);
		while(true){
			System.out.println("angle");
			tilt.getAngleMode().fetchSample(tiltSample,0);
			cur_gyro_reading = tiltSample[0];
			if (Math.abs(Math.abs(cur_gyro_reading - prev_gyro_reading) - 82) <= 2){
				
				System.out.println("finished turning");
				break;
			}
			Motor.B.backward();
			Motor.C.forward();
		}
		stop();
		// Motor.B.rotate(-220,true);
		// Motor.C.rotate(220);
		return;
	}
	public static void right_turn(EV3GyroSensor tilt){
		float[] tiltSample = new float[1];
		tilt.getAngleMode().fetchSample(tiltSample,0);
		float prev_gyro_reading = tiltSample[0];
		float cur_gyro_reading = prev_gyro_reading;
		Motor.B.setSpeed(90);
		Motor.C.setSpeed(90);
		while(true){
			tilt.getAngleMode().fetchSample(tiltSample,0);
			cur_gyro_reading = tiltSample[0];
			System.out.println(cur_gyro_reading - prev_gyro_reading);
			if (Math.abs(Math.abs(cur_gyro_reading - prev_gyro_reading) - 90) <= 2){
				break;
			}
			Motor.B.forward();
			Motor.C.backward();
		}
		stop();
		return;
	}
	
	public static void stop(){
		Motor.B.stop();
		Motor.C.stop();
		return;
	}
	public static void forward(EV3ColorSensor color_port, EV3UltrasonicSensor sonic){
	
		double desired_color = 0.5;
		double k_p = 50;
		
		float[] sonicSample = new float[1];
		sonic.fetchSample(sonicSample,0);
		
		float curr_sonic_reading = sonicSample[0]*100;
		float prev_sonic_reading = curr_sonic_reading;
		
		while(true){
			if (Button.ENTER.isDown()){
				break;
			}
			
			k_p = 50;
			if (curr_sonic_reading < 2){
				k_p = 0;
			}
			
			if (Math.abs(Math.abs(curr_sonic_reading - prev_sonic_reading) - 30) <= 1.5 ){
				break;
			}
			
			sonic.fetchSample(sonicSample,0);
			curr_sonic_reading = sonicSample[0] * 100;
			
			//System.out.println(color_reading(color_port));
			LCD.clear();
			double color = color_reading(color_port);
			double error = desired_color - color;
			double correction; 
			if(error<-0.05){
				correction = error * k_p;
			}
			else if(error>0.05){
				correction = error * k_p;
			}
			else{
				correction = error * k_p;
			}
			int c = (int)correction;
			Motor.B.setSpeed(50 - c);
			Motor.C.setSpeed(50 + c);
			Motor.B.forward();
			Motor.C.forward();

		}
		stop();
		return;
		
		
	}
	
	public static void go_to_next_node(char curPos, char curHead, char next_node, EV3GyroSensor tilt, EV3UltrasonicSensor sonic){
		System.out.println("go to next node");
		int cur_row = -1;
		int cur_col = -1;
		int next_row = -1;
		int next_col = -1;
		
		int curHead_index = -1;
		int nextHead_index = -1;
		for(int i = 1; i < 10; i+=2){
			for(int j =1; j < 10; j +=2){
				
				if (my_map[i][j] == curPos){
					cur_row = i;
					cur_col = j;				
				}
				if (my_map[i][j] == next_node){
					next_row = i;
					next_col = j;
				}

			}	
		}
		
		int[] desired_position = new int[2];
		char[] heading = new char[4];
		
		heading[0] = 'U';
		heading[1] = 'R';
		heading[2] = 'L';
		heading[3] = 'D';
		
		desired_position[0] = next_row - cur_row;
		desired_position[1] = next_col - cur_col;
		
		char desired_heading = get_heading(desired_position);
		
		for (int i = 0; i<4; i++){
			if (heading[i] == curHead){
				curHead_index = i;
			}
			if (heading[i] == desired_heading){
				nextHead_index = i;
			}
		}
		System.out.println("desired heading" + " " + desired_heading);
		System.out.println("curHead_index" + " " + curHead_index);
		System.out.println("nextHead_index" + " " + nextHead_index);
		if (desired_heading != 'N' && curHead_index >=0 && nextHead_index >= 0){
			System.out.println("start turning");
			if (heading[(curHead_index + 1)%4] == desired_heading){
				System.out.println("right turning");
				right_turn(tilt);
			}
			else if (heading[(curHead_index + 3)%4] == desired_heading){
				System.out.println("left turning");
				left_turn(tilt);
			}
			else if (heading[(curHead_index  + 2)%4] == desired_heading){

				right_turn(tilt);
				right_turn(tilt);
			}
			else{
				return;
			}
			
		}
		return;

	}
	public static char get_heading(int[] curPos){
		System.out.println("curPos" + " " + curPos[0] + " " + curPos[1]);
		if (curPos[0] == -2 && curPos[1] == 0){
			return 'U';
		}
		
		if (curPos[0] == 2 && curPos[1] == 0){
			return 'D';
		}
		if (curPos[0] == 0 && curPos[1] == 2){
			return 'R';
		}
		if (curPos[0] == 0 && curPos[1] == -2){
			return 'L';
		}

		return 'N';
	}
	
	public static double color_reading(EV3ColorSensor color_port){
			
			LCD.clear();
			int sampleSize = color_port.sampleSize();
			float[] redsample = new float[sampleSize];
			
			color_port.getRedMode().fetchSample(redsample, 0);
			double z = (redsample[0] - 0.1)*2.5; 
			return z;				
		}
	
	public static void updateMap(char curPos, char curHead){ 
		/***
		 * Inputs: current Position, current Heading
		 * Outputs: None
		 * Function: Use current position and heading to correctly add a wall to the map my_map 
		***/
		
		// Insert your code here...

		//ultrasonic sensor
		//sensor_reading
		int row = -1;
		int col = -1;
		for (int i = 0; i<11;i++){
			for (int j = 0; j<11; j++){
				if (my_map[i][j] == curPos){
					row = i;
					col = j;
				}
			}
		} 
		int currPosition = curPos - 'A' + 1;  //'A' = 1
		if (curHead == 'U'){
			if (my_map[row-1][col] == '0'){
				my_map[row-1][col] = '1';
			}
		}

		else if (curHead == 'D'){
			if (my_map[row+1][col] == '0'){
				my_map[row+1][col] = '1';
			
			}
		}

		else if (curHead == 'L'){
			if (my_map[row][col-1] == '0'){
				my_map[row][col-1] = '1';
			}
		}

		else if (curHead == 'R'){
			if (my_map[row][col+1] == '0'){
				my_map[row][col+1] = '1';
			}
		}
	
}
	
	public static Graph getGraph(char[][] map, int sizeX, int sizeY, Map<Character, int[]> char_to_position){ 
		// Iterate through each robot position of the map
		char[] neighbours;
		Graph g = new Graph();
		char letter;
		for(int i = 1; i < sizeX-1; i+=2){
			for(int j =1; j < sizeY-1; j +=2){
				letter = map[i][j]; // Get current letter we're on and create edges from this on the graph
				neighbours = getNeighbours(letter, map, char_to_position);
				ArrayList<Vertex> vertices = new ArrayList<Vertex>();
				for(int k=0; k < 4; k++){ // Iterate over all neighbours of current position in map
					if(neighbours[k] != 'Z'){
						vertices.add(new Vertex(neighbours[k],1));
					}else{
						break;
					}
				}
				g.addVertex(letter, vertices); // Add list of neighbouring vertices to graph
			}
		}
		return g;
	}
	
	public static char[] getNeighbours(char letter, char[][] map, Map<Character, int[]> char_to_position){
		/***
		 * Inputs: position (char identifier of position in map we want to get the neighbours of)
		 * 		   map (my_map variable above)
		 * 		   char_to_position (hash map, see explanation in initializaMap() )
		 * Outputs: character array size between 1 and 4 of the neighbours (e.g. if we query H, return char will be 'C','I','M','G')
		 * Function: Return neighbors to queried node 
		***/
		
		char[] neighbours = {'Z','Z','Z','Z'}; // Initialize neighbours to null type
		int[] coord = new int[2];
		coord = char_to_position.get(letter);
		int n_index = 0;
		
		//Check if any of the four neighbouring positions are free for the robot to travel to
		if(map[coord[0]-1][coord[1]] == '0'){
			neighbours[n_index] = map[coord[0]-2][coord[1]];
			n_index++;
		}
		if(map[coord[0]+1][coord[1]] == '0'){
			neighbours[n_index] = map[coord[0]+2][coord[1]];
			n_index++;
		}
		if(map[coord[0]][coord[1]-1] == '0'){
			neighbours[n_index] = map[coord[0]][coord[1]-2];
			n_index++;
		}
		if(map[coord[0]][coord[1]+1] == '0'){
			neighbours[n_index] = map[coord[0]][coord[1]+2];
		}
		
		return neighbours;
	}
	
	public static void printMap(char[][] map){
		for(int i = 0; i < 11; i++){
			for(int j =0; j < 11; j ++){
				System.out.print(map[i][j]);
			}
			System.out.println("");
		}
	}
}

// DO NOT CHANGE FOLLOWING CODE. Path planning implementation
class Vertex implements Comparable<Vertex> {
	
	private Character id;
	private Integer distance;
	
	public Vertex(Character id, Integer distance) {
		super();
		this.id = id;
		this.distance = distance;
	}

	public Character getId() {
		return id;
	}

	public Integer getDistance() {
		return distance;
	}

	public void setId(Character id) {
		this.id = id;
	}

	public void setDistance(Integer distance) {
		this.distance = distance;
	}

	@Override
	public int hashCode() {
		final int prime = 31;
		int result = 1;
		result = prime * result
				+ ((distance == null) ? 0 : distance.hashCode());
		result = prime * result + ((id == null) ? 0 : id.hashCode());
		return result;
	}

	@Override
	public boolean equals(Object obj) {
		if (this == obj)
			return true;
		if (obj == null)
			return false;
		if (getClass() != obj.getClass())
			return false;
		Vertex other = (Vertex) obj;
		if (distance == null) {
			if (other.distance != null)
				return false;
		} else if (!distance.equals(other.distance))
			return false;
		if (id == null) {
			if (other.id != null)
				return false;
		} else if (!id.equals(other.id))
			return false;
		return true;
	}

	@Override
	public String toString() {
		return "Vertex [id=" + id + ", distance=" + distance + "]";
	}

	@Override
	public int compareTo(Vertex o) {
		if (this.distance < o.distance)
			return -1;
		else if (this.distance > o.distance)
			return 1;
		else
			return this.getId().compareTo(o.getId());
	}
	
}

class Graph { 
	public final Map<Character, List<Vertex>> vertices;
	
	public Graph() {
		this.vertices = new HashMap<Character, List<Vertex>>();
	}
	
	public void addVertex(Character character, List<Vertex> vertex) {
		this.vertices.put(character, vertex);
	}
	
	public void createHashMap(){
		
	}
	
	public List<Character> getShortestPath(Character start, Character finish) {
		final Map<Character, Integer> distances = new HashMap<Character, Integer>();
		final Map<Character, Vertex> previous = new HashMap<Character, Vertex>();
		PriorityQueue<Vertex> nodes = new PriorityQueue<Vertex>();
		
		for(Character vertex : vertices.keySet()) {
			if (vertex == start) {
				distances.put(vertex, 0);
				nodes.add(new Vertex(vertex, 0));
			} else {
				distances.put(vertex, Integer.MAX_VALUE);
				nodes.add(new Vertex(vertex, Integer.MAX_VALUE));
			}
			previous.put(vertex, null);
		}
		
		while (!nodes.isEmpty()) {
			Vertex smallest = nodes.poll();
			if (smallest.getId() == finish) {
				final List<Character> path = new ArrayList<Character>();
				while (previous.get(smallest.getId()) != null) {
					path.add(smallest.getId());
					smallest = previous.get(smallest.getId());
				}
				return path;
			}

			if (distances.get(smallest.getId()) == Integer.MAX_VALUE) {
				break;
			}
						
			for (Vertex neighbor : vertices.get(smallest.getId())) {
				Integer alt = distances.get(smallest.getId()) + neighbor.getDistance();
				if (alt < distances.get(neighbor.getId())) {
					distances.put(neighbor.getId(), alt);
					previous.put(neighbor.getId(), smallest);
					
					forloop:
					for(Vertex n : nodes) {
						if (n.getId() == neighbor.getId()) {
							nodes.remove(n);
							n.setDistance(alt);
							nodes.add(n);
							break forloop;
						}
					}
				}
			}
		}
		
		return new ArrayList<Character>(distances.keySet());
	}
}
