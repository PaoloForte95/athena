package org.pofe.athena.plan;

import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.PrintStream;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import org.jgrapht.graph.DefaultDirectedGraph;
import org.jgrapht.graph.DefaultEdge;
import org.jgrapht.traverse.TopologicalOrderIterator;
import org.pofe.athena.util.Logging;

import org.pofe.athena.parser.Action;
import org.pofe.athena.verifier.PlanVerifier;


public class PartialOrderPlan extends AbstractPlan {

	protected LinkedHashMap <Integer, ArrayList<Action>> actions;
	
	//Verifier
	protected PlanVerifier planVerifier;

	public PartialOrderPlan(PlanVerifier verifier) {
	        super();
	        logger = Logging.getLogger(PartialOrderPlan.class);
	        actions =  new LinkedHashMap <Integer, ArrayList<Action>>();
	        planVerifier = verifier;
	        graphPlan =  new DefaultDirectedGraph<Action, DefaultEdge>(DefaultEdge.class);
	}
	
	public boolean add(int step, ArrayList<Action> actions) {
		if(this.actions.containsKey(step)) {
			ArrayList<Action> acts = this.actions.get(step);
			acts.addAll(actions);
			if(this.actions.replace(step, acts) != null) {
				return true;
			};
		}
		if(this.actions.put(step, actions)!= null) {
			return true;
		}
		
		return false;
	}
	
	@Override
	public boolean add(int step, Action action) {
		if(this.actions.containsKey(step)) {
			ArrayList<Action> actions = this.actions.get(step);
			actions.add(action);
			if(this.actions.replace(step, actions) != null) {
				return true;
			};
		}
		ArrayList<Action> actions = new ArrayList<Action>();
		actions.add(action);
		if(this.actions.put(step, actions)!= null) {
			return true;
		}
		
		return false;
	}

	@Override
	public boolean remove(int step, Action action) {
		if(this.actions.containsKey(step)) {
			ArrayList<Action> actions = this.actions.get(step);
			boolean removed = actions.remove(action);
			this.actions.replace(step, actions);
			return removed;
		}
		return false;
	}

	@Override
	public boolean remove(int step) {
		if(this.actions.containsKey(step)) {
			if(this.actions.remove(step) != null) {
				return true;
			};
		}
		return false;
	}


	@Override
	public boolean contains(int step, Action action) {
		if(this.actions.containsKey(step)) {
			for (Action act: this.actions.get(step)) {
				if(act.getName().contains(action.getName())) {
					return true;
				}
			}
			
		}
		return false;
	}

	@Override
	public boolean isEmpty() {
		return (this.actions.isEmpty());
	}

	@Override
	public Map<Integer, List<Action>> getActions() {
		HashMap<Integer, List<Action>> actions = new LinkedHashMap <Integer, List<Action>>();
		for (int step: this.actions.keySet() ) {
			actions.put(step,getActions(step));
		}
		return actions;
	}



	@Override
	public List<Action> getActions(int step) {
		ArrayList<Action> actions = new ArrayList<Action>();
		if(this.actions.containsKey(step)) {
			actions.addAll(this.actions.get(step));
			return actions;
		}
		return actions;
	}

	/**
	 * Create a partial order plan represented as a direct graph  
	 * @return A partial order plan represented as a directed graph
	 */

	public DefaultDirectedGraph<Action, DefaultEdge> createGraph() {
	
		//Firstly create a fully connected graph. The function will try to remove an edge at each iteration and verify if the plan is still valid
		DefaultDirectedGraph<Action, DefaultEdge> directedGraph =
	            new DefaultDirectedGraph<Action, DefaultEdge>(DefaultEdge.class);
		ArrayList<Action> actions = new ArrayList<Action>();
		this.actions.forEach(actions::addAll);
			
		for(Action action : actions) {

			directedGraph.addVertex(action);
			int previousMissionIndex = (actions.indexOf(action)-1);
			for(int index = previousMissionIndex; index >= 0 ; index--) {
				  Action previousAction = actions.get(index);
				  directedGraph.addEdge(previousAction, action);	  

			}
		}
	
		//Get all the edges of the plan
		ArrayList <DefaultEdge> edges = new ArrayList <DefaultEdge>(directedGraph.edgeSet());
		int iteration = 1;
		int i = 0;
		int edgesRemoved = 0; 
		while( i < iteration) {
			//Collections.shuffle(edges);
			//Collections.reverse(edges);
			//Try to remove an edge
			while(edgesRemoved < edges.size()) {
				DefaultEdge e = edges.get(edgesRemoved);
				directedGraph.removeEdge(e);
				logger.info("-------------");
				logger.info("Trying to remove edge between " + e);
				//Verify that the plan is still valid
				boolean planIsValid = planVerifier.planValidation(directedGraph);	
				logger.info("Is plan still valid? " + planIsValid);
				logger.info("-------------");
				//If the plan is not valid, restore the edge that has been removed 
				if(!planIsValid) {
					directedGraph.addEdge(directedGraph.getEdgeSource(e), directedGraph.getEdgeTarget(e),e);
				}
				edgesRemoved += 1;

				
			}
			
			i +=1;

		}
		this.graphPlan = directedGraph;
		//Save the plan in a file
		PrintStream fileStream = null;

		try {
			fileStream = new PrintStream((new FileOutputStream("POPlan.txt",false)));	
		} catch (FileNotFoundException e) {

			e.printStackTrace();
		}
		//Print the plan 
		TopologicalOrderIterator<Action, DefaultEdge> it = new TopologicalOrderIterator<Action, DefaultEdge>(graphPlan);
		while(it.hasNext()) {
			Action m1 = it.next();
			fileStream.append(m1.getID()+ ")" + m1.toString() + "\n");
			fileStream.append("Parents:");
			for(DefaultEdge precedenceConstraint : graphPlan.incomingEdgesOf(m1)) {  
				Action parentMission = graphPlan.getEdgeSource(precedenceConstraint);
				fileStream.append(parentMission.toString() + "\n");
			 }
			fileStream.append("\n");
		}	
		
		return graphPlan;
		
	}
	
	@Override
	public double makespan() {
		return this.actions.size();
	}


	@Override
	public double GetCost() {
		return this.actions.size();
	}
	
	/**
	 * Create a Partial Order Plan from another plan and represent it with a directed graph. The plan is created by removing unnecessary constraints 
	 * between the actions from the original plan.
	 * @param graphPlan -> a graph representing the plan
	 * @return A partial order plan represented with a directed graph
	 * @throws IOException
	 */
	public DefaultDirectedGraph<Action, DefaultEdge> createGraphState() throws IOException {

		DefaultDirectedGraph<Action, DefaultEdge> POPlan =
	            new DefaultDirectedGraph<Action, DefaultEdge>(DefaultEdge.class);
		ArrayList<Action> actions = new ArrayList<Action>();
		this.actions.forEach(actions::addAll);
			
		for(Action action : actions) {

			POPlan.addVertex(action);
			int previousMissionIndex = (actions.indexOf(action)-1);
			for(int index = previousMissionIndex; index >= 0 ; index--) {
				  Action previousAction = actions.get(index);
				  POPlan.addEdge(previousAction, action);	  

			}
		}
		

		double makespan = 1000;
		
		ArrayList<Double> values = new ArrayList<Double>();
		int iteration = 1;
		int i = 0;
		while( i < iteration) {
			int edgesRemoved = 0;
			ArrayList <DefaultEdge> edges = new ArrayList <DefaultEdge>(POPlan.edgeSet());
			while(edgesRemoved < edges.size()) {
				
				DefaultEdge e = edges.get(edgesRemoved);
				POPlan.removeEdge(e);
				logger.info("-------------");
				logger.info("Trying to remove edge between " + e);
				//Verify that the plan is still valid
				boolean planIsValid = planVerifier.StatePlanValidation(POPlan);
				logger.info("Is plan still valid? " + planIsValid);
				logger.info("-------------");
				//If the plan is not valid, restore the edge that has been removed 
				if(!planIsValid) {
					POPlan.addEdge(POPlan.getEdgeSource(e), POPlan.getEdgeTarget(e),e);
				}
				edgesRemoved += 1;
			}
			//Get the makespan of the actual plan
			double actualMakeSpan = this.makespan();
			values.add(actualMakeSpan);
			//If found a better plan, update the optimal plan
			if( actualMakeSpan < makespan) {
				makespan =  actualMakeSpan;
				this.graphPlan = (DefaultDirectedGraph<Action, DefaultEdge>) POPlan.clone();
			}
			i += 1;
			

		}
		PrintStream fileStream = null;
		String TOPlan = "POStatePlan.txt";
		try {
			fileStream = new PrintStream((new FileOutputStream(TOPlan,false)));	
		} catch (FileNotFoundException e) {

			e.printStackTrace();
		}
		//Print the plan
		logger.info("Printing Partial Order Plan...");
		TopologicalOrderIterator<Action, DefaultEdge> it = new TopologicalOrderIterator<Action, DefaultEdge>(this.graphPlan);
		while(it.hasNext()) {
			Action m1 = it.next();
			logger.info("Mission: " + m1.toString());
			fileStream.append(m1.toString() + "\n");
			logger.info("Parents: "  );
			fileStream.append("Parents \n");
			for(DefaultEdge precedenceConstraint : graphPlan.incomingEdgesOf(m1)) {  
				Action parentMission = graphPlan.getEdgeSource(precedenceConstraint);
				logger.info(parentMission.toString() );
				  fileStream.append(parentMission.toString() + "\n");
			 }
			fileStream.append("\n");
			logger.info("-------------");
		}	
		return this.graphPlan;
		
	}

	

}
    
    

