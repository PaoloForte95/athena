package org.pofe.athena.plan;


import java.io.FileNotFoundException;
import java.io.FileOutputStream;

import java.io.PrintStream;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;


import org.jgrapht.graph.DefaultDirectedGraph;
import org.jgrapht.graph.DefaultEdge;


import org.pofe.athena.util.Logging;
import org.pofe.athena.parser.Action;


public class SequentialPlan extends AbstractPlan {

	protected LinkedHashMap <Integer, Action> actions;
	

	public SequentialPlan() {
	        super();
	        logger = Logging.getLogger(SequentialPlan.class);
	        actions =  new LinkedHashMap <Integer, Action>();
	        graphPlan =  new DefaultDirectedGraph<Action, DefaultEdge>(DefaultEdge.class);
	}
	
	
	@Override
	public boolean add(int step, Action action) {
		if(this.actions.containsKey(step)) {
			if(this.actions.replace(step, action) != null) {
				return true;
			};
		}
		if(this.actions.put(step, action)!= null) {
			return true;
		}
		
		return false;
	}



	@Override
	public boolean remove(int step, Action action) {
		return this.remove(step);
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
			if(this.actions.get(step).getName().contains(action.getName())) {
				return true;
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
			actions.add(this.actions.get(step));
			return actions;
		}
		return actions;
	}

	/**
	 * Create a Total Order Plan graph Representation of the plan 
	 * @return A total order plan represented with a directed graph
	 */

	@Override
	public DefaultDirectedGraph<Action, DefaultEdge> createGraph() {
			PrintStream fileStream = null;
			String TOPlan = "TOPlan.txt";
			try {
				fileStream = new PrintStream((new FileOutputStream(TOPlan,false)));	
			} catch (FileNotFoundException e) {

				e.printStackTrace();
			}
			
			
			DefaultDirectedGraph<Action, DefaultEdge> directedGraph =
		            new DefaultDirectedGraph<Action, DefaultEdge>(DefaultEdge.class);
			ArrayList<Action> actions = new ArrayList<Action>();
			this.actions.forEach(actions::add);
				
			for(Action action : actions) {

				directedGraph.addVertex(action);
				fileStream.append(action.toString() + "\n");
				int previousMissionIndex = (actions.indexOf(action)-1);
				fileStream.append("Parents: \n");
				for(int index = previousMissionIndex; index >= 0 ; index--) {
					  Action previousAction = actions.get(index);
					  directedGraph.addEdge(previousAction, action);	  
					  fileStream.append(previousAction.toString() + "\n");
				}
				fileStream.append("\n");
			}
			
			
			this.graphPlan = directedGraph;
			return this.graphPlan;
		}


	@Override
	public double makespan() {
		return this.actions.size();
	}


	@Override
	public double GetCost() {
		return this.actions.size();
	}
	
}
    
    

